#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// =====================================================================
// == 常量定义
// =====================================================================
#define ZERO_CROSS_PIN 4      // 过零检测引脚

// --- I2C 与 DAC 配置 ---
const uint32_t I2C_HZ = 1000000;         // I2C时钟频率: 1 MHz
const uint32_t SAMPLE_RATE_HZ = 20000;   // DAC采样率: 20 kHz

// --- DDS 配置 ---
#define LUT_SIZE 256
const float TARGET_FREQ_HZ = 50.0f;

// --- 系统参数 ---
#define DAC_RESOLUTION 4095
#define VOLTAGE_REF 5.0
#define ZERO_OFFSET 2048

// --- 定点数数学参数 ---
#define AMPLITUDE_SCALE_BITS 10
#define AMPLITUDE_SCALE_FACTOR 1024

// --- 模式控制与打印 ---
#define STATUS_PRINT_INTERVAL_MS 5000
#define MODE_SWITCH_INTERVAL_MS 30000

// --- 过零检测防抖 ---
#define DEBOUNCE_MICROS 4000

// --- 模拟测试参数 ---
const float TEST_POWERS[] = {-22000.0, -11000.0, 0.0, 11000.0, 22000.0};
const int NUM_TEST_MODES = sizeof(TEST_POWERS) / sizeof(float);
#define TEST_VOLTAGE 220
#define MAX_CURRENT_A 100

// =====================================================================
// == 全局变量与对象
// =====================================================================
Adafruit_MCP4725 dac;
hw_timer_t* dacTimer = nullptr;

// --- DDS 核心变量 ---
uint16_t sineLUT[LUT_SIZE];
volatile uint32_t phase = 0;
uint32_t phaseStep = 0;

// --- 任务间共享数据 ---
portMUX_TYPE sharedDataMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t sharedAmplitudeScale = 0;
volatile bool sharedPhaseInvert = false;

// --- 应用状态变量 ---
uint8_t testMode = 0;

// --- 过零检测相关 ---
volatile unsigned long lastZeroCrossTime = 0;

// =====================================================================
// == 函数前向声明
// =====================================================================
void IRAM_ATTR onTimer();
void IRAM_ATTR onZeroCross();
void updateSharedData(float power);

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
  Serial.printf("MCP4725 DAC初始化完成 (I2C: %u Hz)\n", I2C_HZ);
}

void generateSineLUT() {
  for (int i = 0; i < LUT_SIZE; i++) {
    float angle = (2.0 * PI * i) / LUT_SIZE;
    sineLUT[i] = (uint16_t)roundf(2047.5f + 2047.5f * sinf(angle));
  }
  Serial.printf("正弦波查找表生成完成 (%d 点)\n", LUT_SIZE);
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
  dacTimer = timerBegin(1000000U); // 1MHz计数频率
  timerAttachInterrupt(dacTimer, &onTimer);
  timerAlarm(dacTimer, timer_alarm_us, true, 0);
  Serial.printf("硬件定时器已初始化 (采样率: %u Hz, 间隔: %u us)\n", SAMPLE_RATE_HZ, timer_alarm_us);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== CT模拟器 - 过零同步版本 ===");

  initI2C_DAC();
  generateSineLUT();
  calculatePhaseStep();
  initZeroCrossInterrupt();
  initDacTimer();

  // 设置初始工作模式
  updateSharedData(TEST_POWERS[testMode]);
  
  Serial.println("=== 系统初始化完成, 开始运行 ===");
}

// =====================================================================
// == 中断服务程序 (ISR)
// =====================================================================

void IRAM_ATTR onTimer() {
  phase += phaseStep;
  uint8_t index = (uint8_t)(phase >> 24);

  portENTER_CRITICAL_ISR(&sharedDataMux);
  uint16_t ampScale = sharedAmplitudeScale;
  bool phaseInvert = sharedPhaseInvert;
  portEXIT_CRITICAL_ISR(&sharedDataMux);

  if (ampScale == 0) {
    dac.setVoltage(ZERO_OFFSET, false);
    return;
  }
  
  if (phaseInvert) {
    index += (LUT_SIZE / 2);
  }
  
  uint16_t baseDacValue = sineLUT[index];
  
  int32_t delta = baseDacValue - ZERO_OFFSET;
  int32_t scaledDelta = (delta * ampScale) >> AMPLITUDE_SCALE_BITS;
  int32_t finalValue = ZERO_OFFSET + scaledDelta;
  
  if (finalValue < 0) finalValue = 0;
  if (finalValue > DAC_RESOLUTION) finalValue = DAC_RESOLUTION;
  
  dac.setVoltage((uint16_t)finalValue, false);
}

void IRAM_ATTR onZeroCross() {
  unsigned long now = micros();
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

/**
 * @brief 根据功率计算并安全地更新共享变量 (幅度和相位)
 * @param power 目标功率 (W)
 */
void updateSharedData(float power) {
  float outputCurrent = power / TEST_VOLTAGE;
  if (outputCurrent > MAX_CURRENT_A) outputCurrent = MAX_CURRENT_A;
  if (outputCurrent < -MAX_CURRENT_A) outputCurrent = -MAX_CURRENT_A;
  float amplitudeRatio = abs(outputCurrent) / MAX_CURRENT_A;
  
  portENTER_CRITICAL(&sharedDataMux);
  sharedAmplitudeScale = (uint16_t)(amplitudeRatio * AMPLITUDE_SCALE_FACTOR);
  sharedPhaseInvert = (power < 0);
  portEXIT_CRITICAL(&sharedDataMux);
}

/**
 * @brief 检查是否需要切换测试模式并执行
 */
void handleModeSwitch() {
  static unsigned long lastModeSwitchTime = 0;
  if (millis() - lastModeSwitchTime >= MODE_SWITCH_INTERVAL_MS) {
    lastModeSwitchTime = millis();
    
    testMode = (testMode + 1) % NUM_TEST_MODES;
    float newPower = TEST_POWERS[testMode];
    updateSharedData(newPower);
    
    Serial.printf("\n>>> 自动切换到模式 %d: %.1f kW <<<\n", testMode, newPower / 1000.0);
  }
}

/**
 * @brief 定期打印当前系统状态
 */
void printStatus() {
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= STATUS_PRINT_INTERVAL_MS) {
    lastPrintTime = millis();
    
    portENTER_CRITICAL(&sharedDataMux);
    uint16_t ampScale = sharedAmplitudeScale;
    bool phaseInvert = sharedPhaseInvert;
    float currentPower = TEST_POWERS[testMode];
    portEXIT_CRITICAL(&sharedDataMux);

    Serial.printf("--- 状态 [模式%d] ---\n", testMode);
    Serial.printf("- 功率: %.1f W | 相位: %s\n", currentPower, phaseInvert ? "反相(180°)" : "正相(0°)");
    Serial.printf("- 幅度: %u/1024 (%.1f%%)\n", ampScale, (float)ampScale / AMPLITUDE_SCALE_FACTOR * 100.0);
  }
}

void loop() {
  handleModeSwitch();
  printStatus();
  delay(10); // 短暂延时，让出CPU给空闲任务
}
