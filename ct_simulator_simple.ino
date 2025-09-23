/*
 * CT模拟器 - 简化测试版本 (优化版本 - 双核架构)
 * 功能：独立验证DAC正弦波输出功能，无需外部模块
 * 测试：不依赖智能电表和过零检测模块
 * 
 * 架构优化：
 * - 核心0 (实时核心): 专门负责DAC正弦波生成，确保200μs精确更新
 * - 核心1 (应用核心): 处理测试模式切换、状态打印等非实时任务
 * - 定点数数学: 避免实时路径中的浮点运算
 * 
 */

#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 系统参数 (注：CT_RATIO仅用于显示，不参与实际计算)
#define DAC_RESOLUTION 4095 // 12位DAC分辨率
#define VOLTAGE_REF 5.0     // 参考电压5.0V (MCP4725输出范围)
#define ZERO_OFFSET 2048    // DAC零点偏移值(2.5V)
#define SINE_TABLE_SIZE 100 // 正弦波查找表大小 (平衡性能和精度)
#define MAX_CURRENT_A 100   // 最大电流限制100A

// 系统时序参数 (消除魔法数字)
#define DAC_UPDATE_INTERVAL_US 200    // DAC更新间隔200μs (50Hz×100点=5kHz)
#define STATUS_PRINT_INTERVAL_MS 3000 // 状态打印间隔3秒
#define MODE_SWITCH_INTERVAL_MS 10000 // 模式切换间隔10秒

// 定点数数学参数 (优化浮点运算)
#define AMPLITUDE_SCALE_BITS 10      // 幅值缩放位数 (1024 = 2^10)
#define AMPLITUDE_SCALE_FACTOR 1024  // 幅值缩放因子 (2^10)

// 模拟测试参数
#define TEST_POWER_1 11000    // 测试功率1: 11kW
#define TEST_POWER_2 -22000   // 测试功率2: -22kW (负功率)
#define TEST_POWER_3 5500     // 测试功率3: 5.5kW
#define TEST_VOLTAGE 220      // 测试电压: 220V

// 全局变量
Adafruit_MCP4725 dac;

// 双核任务间共享数据 (需要临界区保护)
volatile uint16_t sharedAmplitudeScale = 0; // 共享幅值缩放 (定点数)
volatile bool sharedPhaseInvert = false;    // 共享相位反相标志

// 实时任务专用变量 (核心0)
uint16_t sineTable[SINE_TABLE_SIZE];
volatile uint8_t sineIndex = 0;
uint16_t dacValue = ZERO_OFFSET;

// 应用任务专用变量 (核心1)
float currentPower = TEST_POWER_1;  // 当前功率
float currentVoltage = TEST_VOLTAGE; // 当前电压
float outputCurrent = 0;            // 计算出的电流
uint8_t testMode = 0;  // 0=11kW, 1=-22kW, 2=5.5kW
unsigned long lastModeSwitch = 0;

// 双核同步和互斥锁
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t sineWaveTaskHandle = NULL;

// 硬件定时器 (FreeRTOS任务通知方案)
hw_timer_t * dacTimer = NULL;

// 为中断服务程序(ISR)添加前向声明
void IRAM_ATTR dacTimerISR();

void setup() {
  Serial.begin(115200);
  Serial.println("=== CT模拟器简化测试版本 (双核架构优化版本) ===");
  Serial.println("功能: 独立验证DAC正弦波输出");
  Serial.println("无需: 智能电表、过零检测模块");
  
  // 初始化I2C总线
  Wire.begin();
  Wire.setClock(400000); // 设置I2C为400kHz快速模式
  
  // 初始化MCP4725 DAC
  if (!dac.begin(0x60)) {
    Serial.println("MCP4725 DAC初始化失败!");
    while (1);
  }
  Serial.println("MCP4725 DAC初始化完成 (I2C: 400kHz)");
  
  // 生成正弦波查找表
  initSineTable();
  
  // 初始化硬件定时器
  initDacTimer();
  
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
  
  Serial.println("=== 系统初始化完成 ===");
  Serial.println("架构: ESP32双核分离 - 核心0(实时DAC) + 核心1(测试控制)");
  Serial.printf("CT变比: 2000:1\n");
  Serial.printf("最大电流限制: ±%d A\n", MAX_CURRENT_A);
  Serial.printf("DAC输出范围: 0-%.2fV (正向放大模式)\n", VOLTAGE_REF);
  Serial.printf("正弦波查找表: %d 点\n", SINE_TABLE_SIZE);
  Serial.printf("DAC更新频率: %d Hz (每%d μs)\n", 1000000/DAC_UPDATE_INTERVAL_US, DAC_UPDATE_INTERVAL_US);
  Serial.println("开始测试循环...");
}

void loop() {
  static unsigned long lastPrint = 0;
  
  // 每3秒打印一次状态
  if (millis() - lastPrint >= STATUS_PRINT_INTERVAL_MS) {
    lastPrint = millis();
    
    float dacVoltage = (dacValue * VOLTAGE_REF) / DAC_RESOLUTION;
    
    Serial.printf("测试状态 [模式%d]:\n", testMode);
    Serial.printf("- 功率: %.1f W, 电压: %.1f V, 电流: %.1f A\n", 
                 currentPower, currentVoltage, outputCurrent);
    Serial.printf("- DAC输出: %.2f V, 正弦索引: %d\n", dacVoltage, sineIndex);
    Serial.printf("- 幅值比例: %.1f%%\n", (abs(outputCurrent) / MAX_CURRENT_A) * 100);
    Serial.println("---");
  }
  
  // 每10秒切换测试模式
  if (millis() - lastModeSwitch >= MODE_SWITCH_INTERVAL_MS) {
    lastModeSwitch = millis();
    switchTestMode();
  }
  
  // 短暂延时，让出CPU给其他任务
  delay(100);
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

// *** FIX: 更新为兼容ESP32 Core v3.x+ 的新版定时器API ***
void initDacTimer() {
  // 计算所需的中断频率。200µs间隔 = 1,000,000 / 200 = 5000 Hz
  uint32_t frequency = 1000000 / DAC_UPDATE_INTERVAL_US;

  // 初始化硬件定时器，直接设置中断频率
  dacTimer = timerBegin(frequency);
  
  // 将中断服务程序(ISR)附加到定时器
  timerAttachInterrupt(dacTimer, &dacTimerISR);
  
  Serial.printf("硬件定时器已初始化，中断频率: %u Hz (每 %d μs)\n", frequency, DAC_UPDATE_INTERVAL_US);
}


void IRAM_ATTR dacTimerISR() {
  // 轻量级ISR，仅发送任务通知
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(sineWaveTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void calculateOutputCurrent() {
  // 根据功率和电压计算电流
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
  Serial.println("核心0实时任务启动 - 基于硬件定时器+任务通知的精确DAC更新");
  
  while (true) {
    // 等待硬件定时器的任务通知 (阻塞等待，无CPU占用)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // 更新正弦波索引
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

void switchTestMode() {
  testMode = (testMode + 1) % 3;
  
  switch (testMode) {
    case 0:
      currentPower = TEST_POWER_1;  // 11kW
      Serial.printf("切换到测试模式0: %.1f kW\n", currentPower/1000.0);
      break;
    case 1:
      currentPower = TEST_POWER_2;  // -22kW
      Serial.printf("切换到测试模式1: %.1f kW (负功率)\n", currentPower/1000.0);
      break;
    case 2:
      currentPower = TEST_POWER_3;  // 5.5kW
      Serial.printf("切换到测试模式2: %.1f kW\n", currentPower/1000.0);
      break;
  }
  
  calculateOutputCurrent();
  updateSharedData(); // 更新共享数据供实时任务使用
}
