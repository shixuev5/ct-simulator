#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 硬件引脚定义
#define ZERO_CROSS_PIN 4      // 过零检测引脚

// 系统参数
#define DAC_RESOLUTION 4095 // 12位DAC分辨率
#define VOLTAGE_REF 5.0     // 参考电压5.0V (MCP4725输出范围)
#define ZERO_OFFSET 2048    // DAC零点偏移值(2.5V)
#define SINE_TABLE_SIZE 72  // 正弦波查找表大小 (实测验证最优: 72点)
#define MAX_CURRENT_A 100   // 最大电流限制100A

// 系统时序参数 (过零同步模式 - 根据实测频率调整)
#define DAC_UPDATE_INTERVAL_US 260    // DAC更新间隔260μs (实测验证最优: 72点*260μs=18.72ms周期，实测~50Hz)
#define STATUS_PRINT_INTERVAL_MS 5000 // 状态打印间隔5秒
#define MODE_SWITCH_INTERVAL_MS 30000 // 模式切换间隔30秒

// 定点数数学参数 (优化浮点运算)
#define AMPLITUDE_SCALE_BITS 10      // 幅值缩放位数 (1024 = 2^10)
#define AMPLITUDE_SCALE_FACTOR 1024  // 幅值缩放因子 (2^10)

// 过零检测防抖参数
#define DEBOUNCE_MICROS 4000 // 防抖时间4ms

// 模拟测试参数 (5种功率模式)
const float TEST_POWERS[] = {-22000.0, -11000.0, 0.0, 11000.0, 22000.0};
const int NUM_TEST_MODES = sizeof(TEST_POWERS) / sizeof(float);
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
float currentPower = TEST_POWERS[0];  // 当前功率
float currentVoltage = TEST_VOLTAGE;  // 当前电压
float outputCurrent = 0;              // 计算出的电流
uint8_t testMode = 0;                 // 当前测试模式
unsigned long lastModeSwitch = 0;

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

void setup() {
  Serial.begin(115200);
  Serial.println("=== CT模拟器过零检测同步版本 (双核架构优化版本) ===");
  Serial.println("功能: 基于双核架构的过零同步DAC正弦波输出");
  Serial.print("测试功率序列: ");
  for(int i=0; i<NUM_TEST_MODES; i++){
    Serial.printf("%.1fkW ", TEST_POWERS[i]/1000.0);
  }
  Serial.println();
  
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
  
  // 初始化过零检测
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
  Serial.println("架构: ESP32双核分离 - 核心0(实时DAC) + 核心1(过零检测+测试控制)");
  Serial.printf("CT变比: 2000:1\n");
  Serial.printf("最大电流限制: ±%d A\n", MAX_CURRENT_A);
  Serial.printf("DAC输出范围: 0-%.2fV (正向放大模式)\n", VOLTAGE_REF);
  Serial.printf("DAC更新频率: %d Hz (每%d μs)\n", 1000000/DAC_UPDATE_INTERVAL_US, DAC_UPDATE_INTERVAL_US);
  Serial.printf("过零检测引脚: GPIO%d\n", ZERO_CROSS_PIN);
  Serial.println("开始过零同步测试循环...");
}

void loop() {
  static unsigned long lastPrint = 0;
  
  // 每5秒打印一次状态
  if (millis() - lastPrint >= STATUS_PRINT_INTERVAL_MS) {
    lastPrint = millis();
    
    float dacVoltage = (dacValue * VOLTAGE_REF) / DAC_RESOLUTION;
    
    Serial.printf("测试状态 [模式%d]:\n", testMode);
    Serial.printf("- 功率: %.1f W, 电压: %.1f V, 电流: %.1f A\n", 
                currentPower, currentVoltage, outputCurrent);
    Serial.printf("- DAC输出: %.2f V, 正弦索引: %d\n", dacVoltage, sineIndex);
    Serial.printf("- 幅值比例: %.1f%%\n", (abs(outputCurrent) / MAX_CURRENT_A) * 100);
    
    // 显示相位信息
    portENTER_CRITICAL(&timerMux);
    bool phaseInvert = sharedPhaseInvert;
    portEXIT_CRITICAL(&timerMux);
    
    Serial.printf("- 相位: %s\n", 
                phaseInvert ? "反相 (180°)" : "正相 (0°)");
    Serial.println("---");
  }
  
  // 每30秒切换测试模式
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
 * @brief 过零中断处理程序 - 包含防抖、50Hz同步和日志记录
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

void switchTestMode() {
  testMode = (testMode + 1) % NUM_TEST_MODES;
  currentPower = TEST_POWERS[testMode];
  
  Serial.println("\n>>> 切换测试模式 <<<");
  Serial.printf("切换到测试模式%d: %.1f kW\n", testMode, currentPower/1000.0);
  
  calculateOutputCurrent();
  updateSharedData(); // 更新共享数据供实时任务使用
}