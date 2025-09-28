#include <Wire.h>
#include <ModbusMaster.h>
#include <Adafruit_MCP4725.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 硬件引脚定义
#define ZERO_CROSS_PIN 4    // 过零检测引脚
#define RS485_RX_PIN 16     // RS485接收引脚
#define RS485_TX_PIN 17     // RS485发送引脚

// 系统参数
#define DAC_RESOLUTION 4095 // 12位DAC分辨率
#define VOLTAGE_REF 5.0     // 参考电压5.0V (MCP4725输出范围)
#define ZERO_OFFSET 2048    // DAC零点偏移值(2.5V)
#define SINE_TABLE_SIZE 72  // 正弦波查找表大小 (实测验证最优: 72点)
#define MAX_CURRENT_A 100   // 最大电流限制100A

// 系统时序参数 (过零同步模式 - 实测验证最优配置)
#define DAC_UPDATE_INTERVAL_US 260    // DAC更新间隔260μs (实测验证最优: 72点*260μs=18.72ms周期，实测~50Hz)
#define MODBUS_READ_INTERVAL_MS 1000  // Modbus读取间隔1秒
#define STATUS_PRINT_INTERVAL_MS 3000 // 状态打印间隔3秒
#define DATA_PRINT_INTERVAL_MS 5000   // 电表数据打印间隔5秒

// 定点数数学参数 (优化浮点运算)
#define AMPLITUDE_SCALE_BITS 10      // 幅值缩放位数 (1024 = 2^10)
#define AMPLITUDE_SCALE_FACTOR 1024  // 幅值缩放因子 (2^10)

// 过零检测防抖参数
#define DEBOUNCE_MICROS 4000 // 防抖时间4ms

// Modbus通信参数
#define MODBUS_SLAVE_ID 1            // 电表默认地址
#define MODBUS_TIMEOUT_MS 500        // Modbus超时时间
#define MAX_MODBUS_ERRORS 5          // 最大Modbus错误次数
#define MAX_MODBUS_DISCONNECT_ERRORS 10 // Modbus断开连接错误阈值

// Modbus寄存器地址 (根据DTS5885电表文档)
#define REG_TOTAL_ACTIVE_POWER 0x52  // 合相有功功率 (82), Signed, 0.01W, 小端字节交换
#define REG_PHASE_A_VOLTAGE 0x40     // A相电压 (64), Unsigned, 0.01V, 标准大端模式

// 全局对象
ModbusMaster modbus;
Adafruit_MCP4725 dac;

// 双核任务间共享数据 (需要临界区保护)
volatile uint16_t sharedAmplitudeScale = 0; // 共享幅值缩放 (定点数)
volatile bool sharedPhaseInvert = false;    // 共享相位反相标志

// 实时任务专用变量 (核心0)
uint16_t sineTable[SINE_TABLE_SIZE];
volatile uint8_t sineIndex = 0;
uint16_t dacValue = ZERO_OFFSET;

// 应用任务专用变量 (核心1)
float currentPower = 0;      // 当前总有功功率 (W)
float currentVoltage = 220;  // 当前A相电压 (V) - 默认220V
float outputCurrent = 0;     // 计算出的输出电流 (A)
bool modbusConnected = false;
uint16_t modbusErrorCount = 0;
unsigned long lastModbusRead = 0;

// 过零检测相关变量
volatile unsigned long lastZeroCrossTime = 0; // 上一次有效过零触发时间

// 双核同步和互斥锁
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t sineWaveTaskHandle = NULL;

// 硬件定时器 (FreeRTOS任务通知方案)
hw_timer_t * dacTimer = NULL;

// 为中断服务程序(ISR)添加前向声明
void IRAM_ATTR dacTimerISR();
void IRAM_ATTR onZeroCross();

// 函数声明
void initModbus();
void initDAC();
void initSineTable();
void initZeroCrossDetection();
void initDacTimer();
bool readMeterData();
void calculateOutputCurrent();
void updateSharedData();
void sineWaveTask(void* parameter);
void generateSineWave();

void setup() {
  Serial.begin(115200);
  Serial.println("=== CT模拟器电表数据驱动版本 (双核架构优化版本) ===");
  Serial.println("功能: 基于双核架构的电表数据驱动CT模拟");
  Serial.println("数据源: DTS5885三相电表 (Modbus RTU)");
  
  // 初始化硬件和通信
  initModbus();
  initDAC();
  initSineTable();
  initZeroCrossDetection();
  
  // 计算初始电流并更新共享数据
  calculateOutputCurrent();
  updateSharedData();
  
  // 创建核心0高优先级实时任务 (专门负责DAC更新)
  xTaskCreatePinnedToCore(
    sineWaveTask,           // 任务函数
    "SineWaveTask",         // 任务名称
    4096,                   // 堆栈大小
    NULL,                   // 任务参数
    configMAX_PRIORITIES-1, // 最高优先级
    &sineWaveTaskHandle,    // 任务句柄
    0                       // 绑定到核心0 (实时核心)
  );

  // 初始化硬件定时器
  initDacTimer();
  
  Serial.println("=== 系统初始化完成 ===");
  Serial.println("架构: ESP32双核分离 - 核心0(实时DAC) + 核心1(Modbus+过零检测)");
  Serial.printf("CT变比: 2000:1\n");
  Serial.printf("最大电流限制: ±%d A\n", MAX_CURRENT_A);
  Serial.printf("DAC输出范围: 0-%.2fV (正向放大模式)\n", VOLTAGE_REF);
  Serial.printf("正弦波查找表: %d 点 (实测验证最优)\n", SINE_TABLE_SIZE);
  Serial.printf("DAC更新频率: %d Hz (每%d μs)\n", 1000000/DAC_UPDATE_INTERVAL_US, DAC_UPDATE_INTERVAL_US);
  Serial.printf("理论周期: %d点 × %dμs = %.1fms (%.2fHz)\n", 
                SINE_TABLE_SIZE, DAC_UPDATE_INTERVAL_US, 
                (SINE_TABLE_SIZE * DAC_UPDATE_INTERVAL_US) / 1000.0,
                1000000.0 / (SINE_TABLE_SIZE * DAC_UPDATE_INTERVAL_US));
  Serial.println("✓ 实测验证: 该配置最接近50Hz输出");
  Serial.printf("过零检测引脚: GPIO%d\n", ZERO_CROSS_PIN);
  Serial.printf("电表地址: %d, 超时: %dms\n", MODBUS_SLAVE_ID, MODBUS_TIMEOUT_MS);
  Serial.println("等待电表数据和过零信号...");
}

void loop() {
  static unsigned long lastPrint = 0;
  
  // 每1秒读取一次电表数据
  if (millis() - lastModbusRead >= MODBUS_READ_INTERVAL_MS) {
    lastModbusRead = millis();
    
    // 读取电表数据
    if (readMeterData()) {
      // 计算输出电流
      calculateOutputCurrent();
      // 更新共享数据供实时任务使用
      updateSharedData();
    } else {
      // 通信失败时的处理
      if (modbusErrorCount > MAX_MODBUS_ERRORS) {
        currentPower = 0;
        outputCurrent = 0;
        updateSharedData(); // 更新共享数据为0
        if (!modbusConnected) {
          Serial.println("Modbus通信失败，输出电流设为0");
        }
      }
    }
  }
  
  // 每3秒打印一次运行状态
  if (millis() - lastPrint >= STATUS_PRINT_INTERVAL_MS) {
    lastPrint = millis();
    
    float dacVoltage = (dacValue * VOLTAGE_REF) / DAC_RESOLUTION;
    
    Serial.printf("运行状态:\n");
    Serial.printf("- 电表功率: %.1f W, 电压: %.1f V, 计算电流: %.2f A\n", 
                currentPower, currentVoltage, outputCurrent);
    Serial.printf("- DAC输出: %.2f V, 正弦索引: %d\n", dacVoltage, sineIndex);
    Serial.printf("- 幅值比例: %.1f%%, Modbus状态: %s\n", 
                (abs(outputCurrent) / MAX_CURRENT_A) * 100,
                modbusConnected ? "连接" : "断开");
    
    // 显示相位信息
    portENTER_CRITICAL(&timerMux);
    bool phaseInvert = sharedPhaseInvert;
    portEXIT_CRITICAL(&timerMux);
    
    Serial.printf("- 相位: %s\n", 
                phaseInvert ? "反相 (180°)" : "正相 (0°)");
    Serial.println("---");
  }
  
  // 短暂延时，让出CPU给其他任务
  delay(100);
}

void initModbus() {
  // 初始化UART2用于RS485通信 (9600bps, 8N1)
  Serial2.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  
  // 初始化Modbus主机，连接到DTS5885电表
  modbus.begin(MODBUS_SLAVE_ID, Serial2);
  modbus.setTimeout(MODBUS_TIMEOUT_MS);
  
  Serial.println("Modbus通信初始化完成");
  Serial.printf("电表地址: %d\n", MODBUS_SLAVE_ID);
  Serial.println("通信参数: 9600bps, 8N1");
  Serial.printf("超时设置: %dms\n", MODBUS_TIMEOUT_MS);
  Serial.println("RS485模块: 自动收发");
}

void initDAC() {
  // 初始化I2C总线
  Wire.begin();
  Wire.setClock(400000); // 设置I2C为400kHz快速模式
  
  // 初始化MCP4725 DAC
  if (!dac.begin(0x60)) {
    Serial.println("MCP4725 DAC初始化失败!");
    while (1);
  }
  
  // 设置DAC初始值为零点
  dac.setVoltage(ZERO_OFFSET, false);
  
  Serial.println("MCP4725 DAC初始化完成 (I2C: 400kHz)");
}

void initSineTable() {
  // 预计算正弦波查找表 (正向放大模式)
  Serial.println("生成正弦波查找表 (正向放大模式)...");
  
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    float angle = (2.0 * PI * i) / SINE_TABLE_SIZE;
    float sineValue = sin(angle);
    
    // *** 正向放大模式: Vout = 2 * Vin - 5V ***
    // 计算对应的DAC电压值 (0-5V范围，2.5V为零点)
    // 正弦值范围[-1,1]映射到电压范围[0V,5V]
    float outputVoltage = (VOLTAGE_REF / 2.0) + sineValue * (VOLTAGE_REF / 2.0);
    
    // 转换为DAC数字值
    sineTable[i] = (uint16_t)((outputVoltage / VOLTAGE_REF) * DAC_RESOLUTION);
    
    // 限制范围
    if (sineTable[i] > DAC_RESOLUTION) sineTable[i] = DAC_RESOLUTION;
    if (sineTable[i] < 0) sineTable[i] = 0;
  }
  
  Serial.printf("正弦波查找表生成完成 (正向放大模式): %d 点\n", SINE_TABLE_SIZE);
}

void initZeroCrossDetection() {
  // 初始化过零检测引脚和中断
  pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), onZeroCross, RISING);
  Serial.printf("过零检测中断已附加到 GPIO%d (上升沿触发)\n", ZERO_CROSS_PIN);
}

void initDacTimer() {
  uint32_t frequency = 1000000 / DAC_UPDATE_INTERVAL_US;

  // 初始化硬件定时器，直接设置中断频率
  dacTimer = timerBegin(1000000);
  
  // 将中断服务程序(ISR)附加到定时器
  timerAttachInterrupt(dacTimer, &dacTimerISR);

  // 统一配置并启用警报
  timerAlarm(dacTimer, DAC_UPDATE_INTERVAL_US, true, 0);
  
  Serial.printf("硬件定时器已初始化，中断频率: %u Hz (每 %d μs)\n", frequency, DAC_UPDATE_INTERVAL_US);
}

/**
 * @brief 过零中断处理程序 - 包含防抖、50Hz同步
 */
void IRAM_ATTR onZeroCross() {
  unsigned long now = micros();

  // 步骤1: 防抖检查
  if (now - lastZeroCrossTime < DEBOUNCE_MICROS) {
    return; // 抖动或噪声，忽略
  }
  lastZeroCrossTime = now; // 更新有效触发的时间戳

  // 步骤2: 50Hz同步逻辑 (处理100Hz脉冲)
  static bool ignorePulse = false;
  ignorePulse = !ignorePulse;
  if (ignorePulse) {
    // 在每个完整周期的开始重置相位 (直接重置，无需额外同步标志)
    sineIndex = 0; // 重置正弦波索引到起始点
  }
}

void IRAM_ATTR dacTimerISR() {
  // 轻量级ISR，仅发送任务通知
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(sineWaveTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool readMeterData() {
  uint8_t result;
  bool success = true;
  
  // 读取合相有功功率 (寄存器地址0x52, 2个寄存器, Signed, 0.01W)
  result = modbus.readHoldingRegisters(REG_TOTAL_ACTIVE_POWER, 2);
  if (result == modbus.ku8MBSuccess) {
    // 根据文档，功率数据需要小端模式字节交换：低字节在前，高字节在后
    uint32_t powerRaw = ((uint32_t)modbus.getResponseBuffer(1) << 16) | modbus.getResponseBuffer(0);
    int32_t powerSigned = (int32_t)powerRaw;
    currentPower = powerSigned * 0.01; // 分辨率0.01W
    
    modbusConnected = true;
    modbusErrorCount = 0;
  } else {
    Serial.printf("读取功率失败，错误码: %d\n", result);
    success = false;
    modbusErrorCount++;
  }
  
  // 读取A相电压 (寄存器地址0x40, 2个寄存器, Unsigned, 0.01V) 
  result = modbus.readHoldingRegisters(REG_PHASE_A_VOLTAGE, 2);
  if (result == modbus.ku8MBSuccess) {
    // 电压数据使用标准大端模式：高字节在前，低字节在后
    uint32_t voltageRaw = ((uint32_t)modbus.getResponseBuffer(0) << 16) | modbus.getResponseBuffer(1);
    currentVoltage = voltageRaw * 0.01; // 分辨率0.01V
  } else {
    Serial.printf("读取电压失败，错误码: %d\n", result);
    success = false;
    modbusErrorCount++;
  }
  
  // 检查连接状态
  if (modbusErrorCount > MAX_MODBUS_DISCONNECT_ERRORS) {
    modbusConnected = false;
  }
  
  // 打印读取的数据 (降低频率)
  if (success) {
    static unsigned long lastDataPrint = 0;
    if (millis() - lastDataPrint > DATA_PRINT_INTERVAL_MS) {
      lastDataPrint = millis();
      Serial.printf("电表数据 - 功率: %.2f W, 电压: %.2f V\n", currentPower, currentVoltage);
    }
  }
  
  return success;
}

void calculateOutputCurrent() {
  // 根据功率和电压计算电流
  // P = U * I * cos(φ), 假设功率因数为1
  if (currentVoltage > 0) {
    outputCurrent = currentPower / currentVoltage;
  } else {
    outputCurrent = 0;
  }
  
  // 限制最大电流值为100A
  if (outputCurrent > MAX_CURRENT_A) {
    outputCurrent = MAX_CURRENT_A;
  } else if (outputCurrent < -MAX_CURRENT_A) {
    outputCurrent = -MAX_CURRENT_A;
  }
}

void updateSharedData() {
  // 计算定点数幅值缩放 (优化浮点运算)
  float currentAmplitude = abs(outputCurrent);
  float amplitudeRatio = currentAmplitude / MAX_CURRENT_A; // 0-1比例
  if (amplitudeRatio > 1.0) amplitudeRatio = 1.0;
  uint16_t amplitudeScale = (uint16_t)(amplitudeRatio * AMPLITUDE_SCALE_FACTOR);
  
  // 使用临界区安全更新共享数据
  portENTER_CRITICAL(&timerMux);
  sharedAmplitudeScale = amplitudeScale;
  sharedPhaseInvert = (currentPower < 0); // 负功率时相位反相
  portEXIT_CRITICAL(&timerMux);
}

void sineWaveTask(void* parameter) {
  Serial.println("核心0实时任务启动 - 基于硬件定时器+任务通知的精确DAC更新 (过零同步)");
  
  while (true) {
    // 等待硬件定时器的任务通知 (阻塞等待，无CPU占用)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // 更新正弦波索引 (过零中断会直接重置sineIndex，这里只需正常递增)
    sineIndex = (sineIndex + 1) % SINE_TABLE_SIZE;
    
    // 生成当前正弦波输出
    generateSineWave();
  }
}

void generateSineWave() {
  // 安全读取共享数据 (临界区保护)
  uint16_t amplitudeScale;
  bool phaseInvert;
  
  portENTER_CRITICAL(&timerMux);
  amplitudeScale = sharedAmplitudeScale;
  phaseInvert = sharedPhaseInvert;
  portEXIT_CRITICAL(&timerMux);
  
  // 无功率时输出零点2.5V
  if (amplitudeScale == 0) {
    dac.setVoltage(ZERO_OFFSET, false);
    dacValue = ZERO_OFFSET;
    return;
  }
  
  // 确定正弦波索引 (负功率时相位反相)
  uint8_t actualIndex = sineIndex;
  if (phaseInvert) {
    // 负功率时相位反相180度
    actualIndex = (sineIndex + SINE_TABLE_SIZE / 2) % SINE_TABLE_SIZE;
  }
  
  // 从查找表获取基础正弦DAC值
  uint16_t baseDacValue = sineTable[actualIndex];
  
  // 使用定点数数学进行幅值缩放 (避免浮点运算)
  // scaledValue = ZERO_OFFSET + (baseDacValue - ZERO_OFFSET) * amplitudeScale / 1024
  int32_t deltaValue = (int32_t)(baseDacValue - ZERO_OFFSET);
  int32_t scaledDelta = (deltaValue * amplitudeScale) >> AMPLITUDE_SCALE_BITS; // 除以1024
  int32_t scaledValue = ZERO_OFFSET + scaledDelta;
  
  // 限制DAC输出范围
  if (scaledValue > DAC_RESOLUTION) scaledValue = DAC_RESOLUTION;
  if (scaledValue < 0) scaledValue = 0;
  
  // 更新DAC输出
  dacValue = (uint16_t)scaledValue;
  dac.setVoltage(dacValue, false);
}