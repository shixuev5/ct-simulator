/*
 * CT模拟器 - ESP32代码 (优化版本 - 双核架构)
 * 功能：通过Modbus协议读取三相电表数据，计算并输出模拟CT电流信号
 * 
 * 架构优化：
 * - 核心0 (实时核心): 专门负责DAC正弦波生成，确保200μs精确更新
 * - 核心1 (应用核心): 处理Modbus通信、数据计算、状态输出等阻塞操作
 * - 临界区保护: 防止并发访问共享变量导致的数据竞争
 * 
 * 硬件连接：
 * - GPIO4: 过零检测输入
 * - GPIO16: RS485 RX (UART2)
 * - GPIO17: RS485 TX (UART2)
 * - SDA(GPIO21): MCP4725 DAC数据线
 * - SCL(GPIO22): MCP4725 DAC时钟线
 */

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

#define DAC_RESOLUTION 4095 // 12位DAC分辨率
#define VOLTAGE_REF 5.0     // 参考电压5.0V (MCP4725输出范围)
#define ZERO_OFFSET 2048    // DAC零点偏移值(2.5V)
#define SINE_TABLE_SIZE 100 // 正弦波查找表大小 (平衡性能和精度)
#define MAX_CURRENT_A 100   // 最大电流限制100A
#define DEFAULT_PERIOD_MICROS 20000 // 50Hz默认周期 (20ms)

// 系统时序参数
#define DAC_UPDATE_INTERVAL_US 200  // DAC更新间隔200μs (50Hz×100点=5kHz)
#define MODBUS_READ_INTERVAL_MS 1000 // Modbus读取间隔1秒
#define STATUS_PRINT_INTERVAL_MS 3000 // 状态打印间隔3秒
#define DATA_PRINT_INTERVAL_MS 5000   // 数据打印间隔5秒

// 过零检测参数 (消除魔法数字)
#define ISR_DEBOUNCE_MICROS 5000     // ISR防抖时间5ms (200Hz，适应100Hz过零频率)
#define MIN_GRID_PERIOD_MICROS 16000 // 最小市电周期16ms (62.5Hz)
#define MAX_GRID_PERIOD_MICROS 25000 // 最大市电周期25ms (40Hz)
#define TIMER_UPDATE_THRESHOLD_US 50 // 定时器更新阈值50μs (避免频繁重置)
#define MAX_MODBUS_ERRORS 5          // 最大Modbus错误次数
#define MAX_MODBUS_DISCONNECT_ERRORS 10 // Modbus断开连接错误阈值

// 定点数数学参数 (优化浮点运算)
#define AMPLITUDE_SCALE_BITS 10      // 幅值缩放位数 (1024 = 2^10)
#define AMPLITUDE_SCALE_FACTOR 1024  // 幅值缩放因子 (2^10)

// Modbus寄存器地址 (根据DTS5885电表文档)
#define REG_TOTAL_ACTIVE_POWER 0x52  // 合相有功功率 (82), Signed, 0.01W, 小端字节交换
#define REG_PHASE_A_VOLTAGE 0x40     // A相电压 (64), Unsigned, 0.01V, 标准大端模式
#define MODBUS_SLAVE_ID 1            // 电表默认地址

// 全局变量
ModbusMaster modbus;
Adafruit_MCP4725 dac;

// 双核任务间共享数据 (需要临界区保护)
volatile float sharedPower = 0;           // 共享功率数据
volatile float sharedVoltage = 0;         // 共享电压数据  
volatile uint16_t sharedAmplitudeScale = 0; // 共享幅值缩放 (定点数)
volatile bool sharedPhaseInvert = false;  // 共享相位反相标志

// 过零检测和频率同步 (ISR访问)
volatile unsigned long lastZeroCrossTime = 0;
volatile bool zeroCrossDetected = false;
volatile bool lastCrossingDirection = true; // true=正向过零, false=负向过零
volatile unsigned long currentDacInterval = DAC_UPDATE_INTERVAL_US; // 当前DAC更新间隔
volatile unsigned long pendingDacInterval = 0; // 待更新的DAC间隔 (非零表示需要更新)

// 实时任务专用变量 (核心0)
uint16_t sineTable[SINE_TABLE_SIZE];
volatile uint8_t sineIndex = 0;
uint16_t dacValue = ZERO_OFFSET;

// 应用任务专用变量 (核心1)
float currentPower = 0;      // 当前总有功功率 (W)
float currentVoltage = 0;    // 当前A相电压 (V)
float outputCurrent = 0;     // 计算出的输出电流 (A)
bool modbusConnected = false;
uint16_t modbusErrorCount = 0;

// 双核同步和互斥锁
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t sineWaveTaskHandle = NULL;
TaskHandle_t mainLoopTaskHandle = NULL;

// 硬件定时器 (FreeRTOS任务通知方案)
hw_timer_t * dacTimer = NULL;

// 为中断服务程序(ISR)添加前向声明
void IRAM_ATTR dacTimerISR();

// 函数声明
void initHardware();
void initModbus();
void initDAC();
void initSineTable();
void initDacTimer();                 // 初始化DAC硬件定时器
void configureDacTimer(uint32_t intervalMicros); // 通用定时器配置函数
void IRAM_ATTR zeroCrossISR();
void IRAM_ATTR dacTimerISR();        // DAC定时器中断服务程序
bool readMeterData();
void calculateOutputCurrent();
void updateSharedData();
void sineWaveTask(void* parameter);  // 核心0实时任务
void generateSineWave();             // 优化后的正弦波生成

void setup() {
  Serial.begin(115200);
  Serial.println("CT模拟器启动中 (双核架构优化版本)...");
  
  // 初始化硬件
  initHardware();
  initModbus();
  initDAC();
  initSineTable();
  
  // 获取主循环任务句柄 (当前任务运行在核心1)
  mainLoopTaskHandle = xTaskGetCurrentTaskHandle();
  
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
  
  Serial.println("=== CT模拟器系统初始化完成 ===");
  Serial.println("架构: ESP32双核分离 - 核心0(实时DAC) + 核心1(Modbus通信)");
  Serial.printf("最大电流限制: ±%d A\n", MAX_CURRENT_A);
  Serial.printf("DAC输出范围: 0-%.2fV (正向放大模式)\n", VOLTAGE_REF);
  Serial.printf("正弦波查找表: %d 点\n", SINE_TABLE_SIZE);
  Serial.printf("DAC更新频率: %d Hz (每%d μs)\n", 1000000/DAC_UPDATE_INTERVAL_US, DAC_UPDATE_INTERVAL_US);
  Serial.printf("默认频率: %.1f Hz (动态同步到实际市电频率)\n", 1000000.0/DEFAULT_PERIOD_MICROS);
  Serial.println("等待电表数据和过零信号...");
}

void loop() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastPrint = 0;
  
  // 处理待更新的定时器 (在主循环中安全执行)
  if (pendingDacInterval != 0) {
    unsigned long newInterval = pendingDacInterval;
    pendingDacInterval = 0; // 清除待更新标志
    
    configureDacTimer(newInterval);
    currentDacInterval = newInterval;
    
    Serial.printf("定时器频率更新: %.2f Hz (间隔: %lu μs)\n", 
                 1000000.0 / newInterval, newInterval);
  }
  
  // 每1秒读取一次电表数据 (避免过于频繁的Modbus通信)
  if (millis() - lastUpdate >= MODBUS_READ_INTERVAL_MS) {
    lastUpdate = millis();
    
    // 读取电表数据
    if (readMeterData()) {
      // 计算输出电流
      calculateOutputCurrent();
      // 更新共享数据供实时任务使用
      updateSharedData();
    } else {
      // 通信失败时保持之前的电流值，或设为0
      if (modbusErrorCount > MAX_MODBUS_ERRORS) {
        currentPower = 0;
        currentVoltage = 0;
        outputCurrent = 0;
        updateSharedData(); // 更新共享数据为0
        Serial.println("通信失败，输出电流设为0");
      }
    }
  }
  
  // 打印运行状态 (非阻塞，降低频率)
  if (millis() - lastPrint >= STATUS_PRINT_INTERVAL_MS) {
    lastPrint = millis();
    
    Serial.printf("运行状态 - 功率: %.1f W, 电压: %.1f V, 电流: %.2f A\n", 
                 currentPower, currentVoltage, outputCurrent);
    Serial.printf("DAC输出: %.2f V, 正弦索引: %d\n", 
                 (dacValue * VOLTAGE_REF) / DAC_RESOLUTION, sineIndex);
    Serial.printf("Modbus状态: %s, 硬件定时器: 启用\n", 
                 modbusConnected ? "连接" : "断开");
  }
  
  // 使用任务通知进行智能等待
  if (pendingDacInterval == 0) {
    // 没有待处理的定时器更新时，等待通知或超时 (10ms)
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
  }
  // 有待处理任务时立即继续，不等待
}

void initHardware() {
  // 配置过零检测引脚
  pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), zeroCrossISR, FALLING);
  
  Serial.println("硬件引脚初始化完成");
}

void initModbus() {
  // 初始化UART2用于RS485通信 (9600bps, 8N1)
  Serial2.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  
  // 初始化Modbus主机，连接到DTS5885电表
  modbus.begin(MODBUS_SLAVE_ID, Serial2);
  modbus.setTimeout(500); // 设置500ms超时，防止卡死
  
  Serial.println("Modbus通信初始化完成");
  Serial.printf("电表地址: %d\n", MODBUS_SLAVE_ID);
  Serial.println("通信参数: 9600bps, 8N1, 超时: 500ms");
  Serial.println("RS485模块: 自动收发");
}

void initDAC() {
  // 初始化I2C总线
  Wire.begin();
  Wire.setClock(400000); // 设置I2C为400kHz快速模式，提高DAC更新速度
  
  // 初始化MCP4725 DAC
  if (!dac.begin(0x60)) {
    Serial.println("MCP4725 DAC初始化失败!");
    while (1);
  }
  
  // 设置DAC初始值为零点
  dac.setVoltage(ZERO_OFFSET, false);
  
  Serial.println("MCP4725 DAC初始化完成 (I2C: 400kHz)");
}

// 通用定时器配置函数
void configureDacTimer(uint32_t intervalMicros) {
  // 计算所需的中断频率
  uint32_t frequency = 1000000 / intervalMicros;
  
  // 如果定时器已存在，先停止它
  if (dacTimer != NULL) {
    timerEnd(dacTimer);
  }
  
  // 初始化硬件定时器，直接设置中断频率
  dacTimer = timerBegin(frequency);
  
  // 将中断服务程序(ISR)附加到定时器
  timerAttachInterrupt(dacTimer, &dacTimerISR);
}

// *** FIX: 更新为兼容ESP32 Core v3.x+ 的新版定时器API ***
void initDacTimer() {
  configureDacTimer(DAC_UPDATE_INTERVAL_US);
  
  uint32_t frequency = 1000000 / DAC_UPDATE_INTERVAL_US;
  Serial.printf("硬件定时器已初始化，中断频率: %u Hz (每 %d μs)\n", frequency, DAC_UPDATE_INTERVAL_US);
}

void IRAM_ATTR dacTimerISR() {
  // 轻量级ISR，仅发送任务通知
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(sineWaveTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR zeroCrossISR() {
  unsigned long now = micros();
  
  // 防抖动，最小间隔5ms (对应200Hz，适应100Hz过零频率)
  if (now - lastZeroCrossTime > ISR_DEBOUNCE_MICROS) {
    // 计算过零点间隔时间 (100Hz过零，每10ms一次)
    unsigned long zeroCrossInterval = now - lastZeroCrossTime;
    
    // 完整市电周期 = 2个过零点间隔 (50Hz市电 = 20ms周期)
    unsigned long measuredPeriod = zeroCrossInterval * 2;
    
    // 验证周期是否在合理范围内 (40Hz-62.5Hz)
    if (measuredPeriod > MIN_GRID_PERIOD_MICROS && measuredPeriod < MAX_GRID_PERIOD_MICROS) {
      // 计算新的DAC更新间隔
      unsigned long newDacInterval = measuredPeriod / SINE_TABLE_SIZE;
      
      // 只有在频率变化超过阈值时才更新定时器 (避免频繁重置)
      unsigned long intervalDiff = (newDacInterval > currentDacInterval) ? 
                                   (newDacInterval - currentDacInterval) : 
                                   (currentDacInterval - newDacInterval);
      
      if (intervalDiff > TIMER_UPDATE_THRESHOLD_US) {
        // 在ISR中只设置待更新标志，避免在中断中执行复杂操作
        pendingDacInterval = newDacInterval;
        
        // 通知主循环任务立即处理定时器更新
        if (mainLoopTaskHandle != NULL) {
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
          vTaskNotifyGiveFromISR(mainLoopTaskHandle, &xHigherPriorityTaskWoken);
          portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
      }
    }
    
    lastZeroCrossTime = now;
    lastCrossingDirection = !lastCrossingDirection;
    
    // 只在正向过零时设置重置标志（每20ms一次，对应完整周期）
    if (lastCrossingDirection) {
      zeroCrossDetected = true;
    }
  }
}

bool readMeterData() {
  uint8_t result;
  bool success = true;
  
  // 注意：功率寄存器(0x52)和电压寄存器(0x40)地址不连续，相差18个寄存器
  // 考虑到电表中间可能有其他重要数据，分别读取更安全可靠
  // 如需进一步优化，可查阅电表完整寄存器映射表，寻找连续的寄存器块
  
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
    Serial.println("Modbus通信连接丢失");
  }
  
  // 打印读取的数据
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
  sharedPower = currentPower;
  sharedVoltage = currentVoltage;
  sharedAmplitudeScale = amplitudeScale;
  sharedPhaseInvert = (currentPower < 0); // 负功率时相位反相
  portEXIT_CRITICAL(&timerMux);
}

void sineWaveTask(void* parameter) {
  Serial.println("核心0实时任务启动 - 基于硬件定时器+任务通知的精确DAC更新");
  
  while (true) {
    // 等待硬件定时器的任务通知 (阻塞等待，无CPU占用)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // 过零检测同步：在过零点精确重置相位
    if (zeroCrossDetected) {
      sineIndex = 0;
      zeroCrossDetected = false;
    } else {
      // 正常递增索引
      sineIndex = (sineIndex + 1) % SINE_TABLE_SIZE;
    }
    
    // 生成当前正弦波输出 (优化后的版本)
    generateSineWave();
  }
}


void initSineTable() {
  // 预计算正弦波查找表，直接生成DAC值 (正向放大模式)
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
  
  Serial.println("正弦波查找表初始化完成 (正向放大模式)");
  Serial.printf("查找表大小: %d 点\n", SINE_TABLE_SIZE);
  Serial.printf("电压范围: 0-%.2fV (正向放大模式)\n", VOLTAGE_REF);
  Serial.printf("零点DAC值: %d (%.3fV)\n", ZERO_OFFSET, (VOLTAGE_REF / 2.0));
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
