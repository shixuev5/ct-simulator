#include <Arduino.h>
#include <Wire.h>
#include <ModbusMaster.h>
#include <Adafruit_MCP4725.h>
#include <math.h>

// =====================================================================
// == 常量定义
// =====================================================================

// --- 硬件引脚定义 ---
#define ZERO_CROSS_PIN 4      // 过零检测引脚
#define RS485_RX_PIN 16     // RS485接收引脚
#define RS485_TX_PIN 17     // RS485发送引脚

// --- I2C 与 DAC 配置 ---
const uint32_t I2C_HZ = 1000000;         // I2C时钟频率: 1 MHz (与代码一同步)
const uint32_t SAMPLE_RATE_HZ = 20000;   // DAC采样率: 20 kHz (与代码一同步)

// --- DDS 配置 ---
#define SINE_TABLE_SIZE 256              // 正弦查找表大小, 调整为256以匹配DDS的高效索引
const float TARGET_FREQ_HZ = 50.0f;    // 目标输出频率

// --- 系统参数 ---
#define DAC_RESOLUTION 4095
#define VOLTAGE_REF 5.0
#define ZERO_OFFSET 2048
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
#define MAX_MODBUS_ERRORS 5
#define MAX_MODBUS_DISCONNECT_ERRORS 10
// Modbus寄存器地址 (根据DTS5885电表文档)
#define REG_TOTAL_ACTIVE_POWER 0x52  // 合相有功功率 (82), Signed, 0.01W, 小端字节交换
#define REG_PHASE_A_VOLTAGE 0x40     // A相电压 (64), Unsigned, 0.01V, 标准大端模式

// =====================================================================
// == 全局变量与对象
// =====================================================================

ModbusMaster modbus;
Adafruit_MCP4725 dac;
hw_timer_t* dacTimer = nullptr;

// --- DDS 核心变量 ---
uint16_t sineTable[SINE_TABLE_SIZE];
volatile uint32_t phase = 0;
uint32_t phaseStep = 0;

// --- 任务间共享数据 ---
portMUX_TYPE sharedDataMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t sharedAmplitudeScale = 0;
volatile bool sharedPhaseInvert = false;

// --- 应用状态变量 ---
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
void updateSharedData(float power);
bool readMeterData();

// =====================================================================
// == 初始化函数 (Setup)
// =====================================================================

void initI2C_DAC() {
  Wire.begin();
  Wire.setClock(I2C_HZ);
  if (!dac.begin(0x60)) {
    Serial.println("错误: MCP4725 DAC未找到!");
    while (1);
  }
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
    // 原始值范围 [-2047.5, 2047.5], 加上偏移后 [0, 4095]
    sineTable[i] = (uint16_t)roundf(2047.5f + 2047.5f * sinf(angle));
  }
  Serial.printf("正弦波查找表生成完成 (%d 点)\n", SINE_TABLE_SIZE);
}

void calculatePhaseStep() {
  double ratio = (double)TARGET_FREQ_HZ / (double)SAMPLE_RATE_HZ;
  phaseStep = (uint32_t)llround(ratio * 4294967296.0); // 2^32
  Serial.printf("DDS相位步长已计算 (目标频率: %.1f Hz)\n", TARGET_FREQ_HZ);
}

void initZeroCrossInterrupt() {
  pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), onZeroCross, RISING);
  Serial.printf("过零检测中断已附加到 GPIO%d\n", ZERO_CROSS_PIN);
}

void initDacTimer() {
  uint32_t timer_alarm_us = 1000000U / SAMPLE_RATE_HZ;
  dacTimer = timerBegin(1000000U); // 1MHz 计数频率
  timerAttachInterrupt(dacTimer, &onTimer); // 将onTimer ISR附加到定时器
  timerAlarm(dacTimer, timer_alarm_us, true, 0); // 设置周期性警报
  Serial.printf("硬件定时器已初始化 (采样率: %u Hz, 间隔: %u us)\n", SAMPLE_RATE_HZ, timer_alarm_us);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== CT模拟器 100A/50mA ===");

  initI2C_DAC();
  initModbus();
  generateSineTable();
  calculatePhaseStep();
  initZeroCrossInterrupt();
  initDacTimer();

  updateSharedData(0.0f);
  
  Serial.println("=== 系统初始化完成, 开始运行 ===");
}

// =====================================================================
// == 中断服务程序 (ISR) - 已重写
// =====================================================================

void IRAM_ATTR onTimer() {
  // 1. 更新相位累加器
  phase += phaseStep;
  
  // 2. 从相位最高8位计算查找表索引
  uint8_t index = (uint8_t)(phase >> 24);

  // 3. 安全地从主循环获取幅度和相位数据
  portENTER_CRITICAL_ISR(&sharedDataMux);
  uint16_t ampScale = sharedAmplitudeScale;
  bool phaseInvert = sharedPhaseInvert;
  portEXIT_CRITICAL_ISR(&sharedDataMux);

  // 4. 如果幅度为0, 直接输出直流偏置并返回
  if (ampScale == 0) {
    dac.setVoltage(ZERO_OFFSET, false);
    return;
  }
  
  // 5. 如果需要反相, 将索引偏移半个周期
  if (phaseInvert) {
    index += (SINE_TABLE_SIZE / 2); // 索引自动回绕 (uint8_t)
  }
  
  // 6. 从查找表获取基准DAC值
  uint16_t baseDacValue = sineTable[index];
  
  // 7. 使用定点数进行幅度缩放 (高性能)
  int32_t delta = baseDacValue - ZERO_OFFSET;
  int32_t scaledDelta = (delta * ampScale) >> AMPLITUDE_SCALE_BITS;
  int32_t finalValue = ZERO_OFFSET + scaledDelta;
  
  // 8. 范围限制, 确保输出安全
  if (finalValue < 0) finalValue = 0;
  if (finalValue > DAC_RESOLUTION) finalValue = DAC_RESOLUTION;
  
  // 9. 设置DAC电压 (使用非阻塞快速写入)
  dac.setVoltage((uint16_t)finalValue, false);
}

void IRAM_ATTR onZeroCross() {
  unsigned long now = micros();
  // 防抖
  if (now - lastZeroCrossTime < DEBOUNCE_MICROS) {
    return;
  }
  lastZeroCrossTime = now;
  
  // 市电过零检测通常为100Hz, 我们需要50Hz同步, 所以忽略一半的脉冲
  static bool ignorePulse = false;
  ignorePulse = !ignorePulse;
  if (ignorePulse) {
    // 同步DDS相位累加器
    phase = 0;
  }
}


// =====================================================================
// == 主循环与应用逻辑 (Loop)
// =====================================================================

void updateSharedData(float power) {
  // 根据功率计算电流
  float outputCurrent = (currentVoltage > 1.0f) ? (power / currentVoltage) : 0.0f;
  
  // 限制最大电流
  if (outputCurrent > MAX_CURRENT_A) outputCurrent = MAX_CURRENT_A;
  if (outputCurrent < -MAX_CURRENT_A) outputCurrent = -MAX_CURRENT_A;

  // 计算幅度比例 (0.0 to 1.0)
  float amplitudeRatio = fabsf(outputCurrent) / MAX_CURRENT_A;
  
  // 使用临界区安全地更新共享给ISR的变量
  portENTER_CRITICAL(&sharedDataMux);
  sharedAmplitudeScale = (uint16_t)(amplitudeRatio * AMPLITUDE_SCALE_FACTOR);
  sharedPhaseInvert = (power < 0);
  portEXIT_CRITICAL(&sharedDataMux);
}

bool readMeterData() {
  uint8_t result;
  bool success = true;
  
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

    Serial.println("--- 系统状态 ---");
    Serial.printf("- Modbus: %s | 功率: %.2f W | 电压: %.2f V\n", 
                  modbusConnected ? "已连接" : "已断开", currentPower, currentVoltage);
    Serial.printf("- 计算电流: %.2f A\n", outputCurrent);
    Serial.printf("- DDS 幅度: %u/%u (%.1f%%) | 相位: %s\n", 
                  ampScale, AMPLITUDE_SCALE_FACTOR, 
                  (float)ampScale / AMPLITUDE_SCALE_FACTOR * 100.0,
                  phaseInvert ? "反相(180°)" : "正相(0°)");
  }
}

void loop() {
  static unsigned long lastModbusReadTime = 0;

  // 定期读取电表数据
  if (millis() - lastModbusReadTime >= MODBUS_READ_INTERVAL_MS) {
    lastModbusReadTime = millis();
    if (!readMeterData() && !modbusConnected) {
      // 如果通信持续失败, 将功率清零
      currentPower = 0.0f;
    }
    // 根据最新读取或已清零的功率更新DAC输出参数
    updateSharedData(currentPower);
  }

  // 定期打印状态
  printStatus();

  // 短暂延时, 让出CPU给空闲任务
  delay(10); 
}
