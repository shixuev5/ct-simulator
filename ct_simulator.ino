#include <Arduino.h>
#include <Wire.h>
#include <ModbusMaster.h>
#include <Adafruit_MCP4725.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h> // 引入队列库

// =====================================================================
// == 常量定义
// =====================================================================

// --- 硬件引脚定义 ---
#define ZERO_CROSS_PIN 4      // 过零检测引脚
#define RS485_RX_PIN 16       // RS485接收引脚
#define RS485_TX_PIN 17       // RS485发送引脚

// --- I2C 与 DAC 配置 ---
const uint32_t I2C_HZ = 400000;         // I2C时钟频率: 400 KHz
const uint32_t SAMPLE_RATE_HZ = 12800;   // DAC采样率: 20 kHz

// --- DDS 配置 ---
#define SINE_TABLE_SIZE 256
const float TARGET_FREQ_HZ = 50.0f;

// --- 系统参数 ---
#define DAC_RESOLUTION 4095
#define ZERO_OFFSET 2047
#define MAX_CURRENT_A 100

// --- 定点数数学参数 ---
#define AMPLITUDE_SCALE_BITS 10
#define AMPLITUDE_SCALE_FACTOR 1024

// --- 模式控制与打印 ---
#define MODBUS_READ_INTERVAL_MS 1000
#define STATUS_PRINT_INTERVAL_MS 5000

// --- 过零检测防抖 ---
#define DEBOUNCE_MICROS 4000

// --- Modbus通信参数 ---
#define MODBUS_SLAVE_ID 1
#define MODBUS_TIMEOUT_MS 500
#define MAX_MODBUS_DISCONNECT_ERRORS 10
#define REG_TOTAL_ACTIVE_POWER 0x52  
#define REG_PHASE_A_VOLTAGE 0x40     

// =====================================================================
// == 全局变量与对象
// =====================================================================

ModbusMaster modbus;
Adafruit_MCP4725 dac;
hw_timer_t* dacTimer = nullptr;

// --- 多任务与队列句柄 ---
QueueHandle_t dacQueue;         // 核心间的FIFO队列
TaskHandle_t dacTaskHandle = NULL; // Core 0 任务句柄

// --- DDS 核心变量 ---
uint16_t sineTable[SINE_TABLE_SIZE];
volatile uint32_t phase = 0;
uint32_t phaseStep = 0;

// --- 任务间共享数据 (Core 1 写, ISR 读) ---
portMUX_TYPE sharedDataMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t sharedAmplitudeScale = 0;
volatile bool sharedPhaseInvert = false;

// --- 应用状态变量 (Core 1 本地) ---
float currentPower = 0.0f;
float currentVoltage = 220.0f;
bool modbusConnected = false;
uint16_t modbusErrorCount = 0;

// --- 过零检测相关 ---
volatile unsigned long lastZeroCrossTime = 0;

// =====================================================================
// == 函数前向声明
// =====================================================================
void IRAM_ATTR onTimer();
void IRAM_ATTR onZeroCross();
void dacTask(void *parameter); // 新增任务函数

// =====================================================================
// == 核心 0 专用任务: I2C 发送
// =====================================================================

/**
 * @brief 运行在 Core 0 上的高优先级任务
 * 专门负责从队列取出计算好的数值并写入 DAC
 */
void dacTask(void *parameter) {
  uint16_t valueToSend;
  
  // 无限循环，任务永远不应返回
  for(;;) {
    // 阻塞等待队列数据 (portMAX_DELAY 表示如果队列空了就无限期挂起等待，不耗CPU)
    // 一旦 ISR 放入数据，此任务立即被唤醒
    if (xQueueReceive(dacQueue, &valueToSend, portMAX_DELAY) == pdTRUE) {
      dac.setVoltage(valueToSend, false);
    }
  }
}

// =====================================================================
// == 初始化函数 (Setup)
// =====================================================================

void initI2C_DAC() {
  Wire.begin();
  Wire.setClock(I2C_HZ);
  // 注意：DAC对象在Setup中初始化后，之后只能由 dacTask (Core 0) 访问 setVoltage
  if (!dac.begin(0x60)) {
    Serial.println("错误: MCP4725 DAC未找到!");
    while (1);
  }
  // 初始设为0点
  dac.setVoltage(ZERO_OFFSET, false);
  Serial.printf("MCP4725 DAC初始化完成 (I2C: %u Hz)\n", I2C_HZ);
}

void initModbus() {
  Serial2.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  modbus.begin(MODBUS_SLAVE_ID, Serial2);
  modbus.setTimeout(MODBUS_TIMEOUT_MS);
  Serial.println("Modbus通信初始化完成 (9600 8N1)");
}

void generateSineTable() {
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    float angle = (2.0 * PI * i) / SINE_TABLE_SIZE;
    sineTable[i] = (uint16_t)roundf(2047.0f + 2047.0f * sinf(angle));
  }
  Serial.printf("正弦波查找表生成完成 (%d 点)\n", SINE_TABLE_SIZE);
}

void calculatePhaseStep() {
  double ratio = (double)TARGET_FREQ_HZ / (double)SAMPLE_RATE_HZ;
  phaseStep = (uint32_t)llround(ratio * 4294967296.0); 
  Serial.printf("DDS相位步长已计算 (目标频率: %.1f Hz)\n", TARGET_FREQ_HZ);
}

void initZeroCrossInterrupt() {
  pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), onZeroCross, RISING);
  Serial.printf("过零检测中断已附加到 GPIO%d\n", ZERO_CROSS_PIN);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== CT模拟器 (双核架构版) ===");

  // 1. 初始化基础硬件
  initI2C_DAC();
  initModbus();
  generateSineTable();
  calculatePhaseStep();
  initZeroCrossInterrupt();

  // 2. 创建 FreeRTOS 队列
  // 深度为10足以缓冲ISR和I2C之间的微小抖动
  dacQueue = xQueueCreate(10, sizeof(uint16_t));
  if (dacQueue == NULL) {
    Serial.println("错误: 无法创建队列");
    while(1);
  }

  // 3. 创建 Core 0 任务 (DAC 驱动)
  // 优先级设为 10 (高), 确保 Modbus 或 Wifi 不会打断它
  xTaskCreatePinnedToCore(
    dacTask,        // 任务函数
    "DACTask",      // 任务名称
    4096,           // 栈大小
    NULL,           // 参数
    10,             // 优先级
    &dacTaskHandle, // 句柄
    0               // 核心编号 (Core 0)
  );
  Serial.println("DAC I2C任务已启动 (Core 0)");

  // 4. 最后启动定时器中断 (ISR 产生数据)
  uint32_t timer_alarm_us = 1000000U / SAMPLE_RATE_HZ;
  dacTimer = timerBegin(1000000U); 
  timerAttachInterrupt(dacTimer, &onTimer);
  timerAlarm(dacTimer, timer_alarm_us, true, 0);
  Serial.printf("硬件定时器已启动 (Core 1 ISR -> Queue -> Core 0 Task)\n");
}

// =====================================================================
// == 中断服务程序 (ISR)
// =====================================================================

void IRAM_ATTR onTimer() {
  // 1. DDS 计算 (与之前相同)
  phase += phaseStep;
  uint8_t index = (uint8_t)(phase >> 24);

  portENTER_CRITICAL_ISR(&sharedDataMux);
  uint16_t ampScale = sharedAmplitudeScale;
  bool phaseInvert = sharedPhaseInvert;
  portEXIT_CRITICAL_ISR(&sharedDataMux);

  uint16_t finalDacValue;

  if (ampScale == 0) {
    finalDacValue = ZERO_OFFSET;
  } else {
    if (phaseInvert) index += (SINE_TABLE_SIZE / 2);
    
    uint16_t baseDacValue = sineTable[index];
    int32_t delta = baseDacValue - ZERO_OFFSET;
    int32_t scaledDelta = (delta * ampScale) >> AMPLITUDE_SCALE_BITS;
    int32_t val = ZERO_OFFSET + scaledDelta;
    
    if (val < 0) val = 0;
    else if (val > DAC_RESOLUTION) val = DAC_RESOLUTION;
    finalDacValue = (uint16_t)val;
  }

  // 2. 发送数据到队列，而不是直接操作 DAC
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  // 尝试写入队列。如果队列满了(errQUEUE_FULL)，说明I2C太慢处理不过来，
  // 这里选择丢弃该点而不阻塞ISR，保证中断安全。
  xQueueSendFromISR(dacQueue, &finalDacValue, &xHigherPriorityTaskWoken);

  // 如果此操作唤醒了高优先级的 dacTask，则请求上下文切换，让 Core 0 立即处理
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR onZeroCross() {
  unsigned long now = micros();
  if (now - lastZeroCrossTime < DEBOUNCE_MICROS) return;
  lastZeroCrossTime = now;
  
  static bool ignorePulse = false;
  ignorePulse = !ignorePulse;
  if (ignorePulse) phase = 0;
}

// =====================================================================
// == 主循环 (Core 1 Loop)
// =====================================================================

void updateSharedData(float power) {
  float outputCurrent = (currentVoltage > 1.0f) ? (power / currentVoltage) : 0.0f;
  
  if (outputCurrent > MAX_CURRENT_A) outputCurrent = MAX_CURRENT_A;
  if (outputCurrent < -MAX_CURRENT_A) outputCurrent = -MAX_CURRENT_A;

  float amplitudeRatio = fabsf(outputCurrent) / MAX_CURRENT_A;
  
  portENTER_CRITICAL(&sharedDataMux);
  sharedAmplitudeScale = (uint16_t)(amplitudeRatio * AMPLITUDE_SCALE_FACTOR);
  sharedPhaseInvert = (power < 0);
  portEXIT_CRITICAL(&sharedDataMux);
}

bool readMeterData() {
  uint8_t result;
  bool success = true;
  
  // Modbus 读取可能会阻塞一段时间，但这在 Loop (Core 1) 是安全的，
  // 因为 DAC 的输出由 Core 0 和 中断 接管了。
  
  // 读取合相有功功率
  result = modbus.readHoldingRegisters(REG_TOTAL_ACTIVE_POWER, 2);
  if (result == modbus.ku8MBSuccess) {
    uint32_t powerRaw = ((uint32_t)modbus.getResponseBuffer(1) << 16) | modbus.getResponseBuffer(0);
    currentPower = (int32_t)powerRaw * 0.01;
    modbusErrorCount = 0;
    modbusConnected = true;
  } else {
    modbusErrorCount++;
    success = false;
  }
  
  // 读取A相电压
  result = modbus.readHoldingRegisters(REG_PHASE_A_VOLTAGE, 2);
  if (result == modbus.ku8MBSuccess) {
    uint32_t voltageRaw = ((uint32_t)modbus.getResponseBuffer(0) << 16) | modbus.getResponseBuffer(1);
    currentVoltage = voltageRaw * 0.01;
  } else {
    modbusErrorCount++;
    success = false;
  }
  
  if (modbusErrorCount > MAX_MODBUS_DISCONNECT_ERRORS) {
    modbusConnected = false;
  }
  
  return success;
}

void printStatus() {
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= STATUS_PRINT_INTERVAL_MS) {
    lastPrintTime = millis();
    
    portENTER_CRITICAL(&sharedDataMux);
    uint16_t ampScale = sharedAmplitudeScale;
    bool phaseInvert = sharedPhaseInvert;
    portEXIT_CRITICAL(&sharedDataMux);

    float outputCurrent = (currentVoltage > 1.0f) ? (currentPower / currentVoltage) : 0.0f;

    Serial.println("--- 系统状态 (Core 1) ---");
    Serial.printf("- Modbus: %s | 功率: %.2f W | 电压: %.2f V\n", 
                  modbusConnected ? "已连接" : "已断开", currentPower, currentVoltage);
    Serial.printf("- 计算电流: %.2f A\n", outputCurrent);
    Serial.printf("- DDS 幅度: %u (%.1f%%) | 相位: %s\n", 
                  ampScale, (float)ampScale / AMPLITUDE_SCALE_FACTOR * 100.0,
                  phaseInvert ? "反相" : "正相");
    
    // 调试队列状态 (可选): 检查是否有堆积
    // Serial.printf("- Queue 空闲空间: %d\n", uxQueueSpacesAvailable(dacQueue));
  }
}

void loop() {
  static unsigned long lastModbusReadTime = 0;

  // 1. Modbus 通信
  // 这里的阻塞完全不会影响 Core 0 的 DAC 输出
  if (millis() - lastModbusReadTime >= MODBUS_READ_INTERVAL_MS) {
    lastModbusReadTime = millis();
    if (!readMeterData() && !modbusConnected) {
      currentPower = 0.0f;
    }
    updateSharedData(currentPower);
  }

  // 2. 状态打印
  printStatus();

  // 3. 延时
  // Core 1 可以自由延时
  delay(10); 
}