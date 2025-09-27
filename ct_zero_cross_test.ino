/*
 * CT模拟器 - 最终功能完整版 v2.2
 * 功能: 
 * - 监听50Hz市电过零信号，包含防抖和100Hz脉冲处理。
 * - 通过MCP4725生成同相位的正弦波。
 * - 每30秒自动切换模式，模拟 -22kW, -11kW, 0, 11kW, 22kW 功率。
 * - 动态计算并调整输出正弦波的幅值和相位。
 * - 包含一个安全的中断触发日志，用于调试。
 * - 已适配 ESP32 Arduino Core v3.x+ 的定时器API。
 * 
 * 硬件连接:
 * - GPIO4: 过零检测输入 (上升沿触发)
 * - SDA(GPIO21): MCP4725 DAC数据线
 * - SCL(GPIO22): MCP4725 DAC时钟线
 */

#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <math.h>
#include <freertos/FreeRTOS.h> // 用于临界区保护

// 硬件引脚定义
#define ZERO_CROSS_PIN 4      // 过零检测引脚

// 正弦波和DAC参数
#define SINE_TABLE_SIZE 100   // 正弦波查找表大小
#define DAC_RESOLUTION 4095   // MCP4725是12位DAC
#define DAC_ZERO_OFFSET 2048  // 12位DAC的中间值 (2.5V)

// 50Hz市电固定时序参数
#define DAC_UPDATE_INTERVAL_US 200 // 50Hz * 100点 = 5kHz更新率

// 模拟测试参数
#define MAX_CURRENT_A 100.0   // 22000W / 220V = 100A
#define TEST_VOLTAGE 220.0    // 假设的市电电压
#define MODE_SWITCH_INTERVAL_MS 30000 // 模式切换间隔30秒
#define STATUS_PRINT_INTERVAL_MS 5000 // 状态打印间隔5秒

// 定义不同的测试功率 (单位: W)
const float TEST_POWERS[] = {-22000.0, -11000.0, 0.0, 11000.0, 22000.0};
const int NUM_TEST_MODES = sizeof(TEST_POWERS) / sizeof(float);

// 过零检测防抖参数
const unsigned long DEBOUNCE_MICROS = 4000; // 防抖时间4ms
volatile unsigned long lastZeroCrossTime = 0; // 存储上一次有效触发的时间戳

// 全局对象和变量
Adafruit_MCP4725 dac;
hw_timer_t *dacTimer = NULL;

uint16_t sineTable[SINE_TABLE_SIZE];
volatile int sineIndex = 0;

uint8_t testMode = 0;
float currentPower = TEST_POWERS[0];
float outputCurrent = 0;
unsigned long lastModeSwitchTime = 0;

// 主循环与ISR共享的数据
portMUX_TYPE sharedDataMux = portMUX_INITIALIZER_UNLOCKED;
volatile float sharedAmplitudeRatio = 0.0;
volatile bool sharedPhaseInvert = false;

// 用于ISR日志的变量
volatile long zeroCrossTriggerCount = 0;
volatile unsigned long lastIsrTimestamp = 0;


// 函数声明
void calculateAndApplySettings();
void switchTestMode();

/**
 * @brief 过零中断 (最终版) - 包含防抖、50Hz同步和日志记录
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
    sineIndex = 0; // 在每个完整周期的开始重置相位
  }

  // 步骤3: 安全日志记录 (只做最少的工作)
  zeroCrossTriggerCount++;
  lastIsrTimestamp = now;
}

/**
 * @brief 定时器中断 - 根据共享数据生成波形
 */
void IRAM_ATTR onTimer() {
  float amplitudeRatio;
  bool phaseInvert;
  
  portENTER_CRITICAL_ISR(&sharedDataMux);
  amplitudeRatio = sharedAmplitudeRatio;
  phaseInvert = sharedPhaseInvert;
  portEXIT_CRITICAL_ISR(&sharedDataMux);

  if (amplitudeRatio == 0.0) {
    dac.setVoltage(DAC_ZERO_OFFSET, false);
    sineIndex = (sineIndex + 1) % SINE_TABLE_SIZE;
    return;
  }

  int actualIndex = sineIndex;
  if (phaseInvert) {
    actualIndex = (sineIndex + SINE_TABLE_SIZE / 2) % SINE_TABLE_SIZE;
  }

  uint16_t baseDacValue = sineTable[actualIndex];
  
  int32_t delta = baseDacValue - DAC_ZERO_OFFSET;
  float scaledDelta = delta * amplitudeRatio;
  int32_t finalDacValue = DAC_ZERO_OFFSET + (int32_t)scaledDelta;

  if (finalDacValue > DAC_RESOLUTION) finalDacValue = DAC_RESOLUTION;
  if (finalDacValue < 0) finalDacValue = 0;
  
  dac.setVoltage((uint16_t)finalDacValue, false);

  sineIndex = (sineIndex + 1) % SINE_TABLE_SIZE;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== CT模拟器 - 最终功能完整版 v2.2 ===");
  Serial.print("测试功率序列: ");
  for(int i=0; i<NUM_TEST_MODES; i++){
    Serial.printf("%.1fkW ", TEST_POWERS[i]/1000.0);
  }
  Serial.println();

  Wire.begin();
  Wire.setClock(400000); // 设置I2C为400kHz快速模式

  if (!dac.begin(0x60)) {
    Serial.println("错误: MCP4725 DAC未找到! 请检查接线。");
    while (1);
  }
  Serial.println("MCP4725 DAC 初始化成功 (快速模式: 400kHz).");

  Serial.println("正在生成正弦波查找表...");
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    float angle = (2.0 * PI * i) / SINE_TABLE_SIZE;
    sineTable[i] = (uint16_t)((sin(angle) + 1.0) * (DAC_RESOLUTION / 2.0));
  }

  pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), onZeroCross, RISING);
  Serial.printf("过零检测中断已附加到 GPIO%d\n", ZERO_CROSS_PIN);

  // 1. 初始化定时器，并设置其计数频率为 1MHz (即计数器每 1 微秒加 1)
  dacTimer = timerBegin(1000000); 
  
  // 2. 将 onTimer 中断服务程序附加到定时器
  timerAttachInterrupt(dacTimer, &onTimer);
  
  // 3. 设置警报：当计数器达到 200 (即 200µs) 时触发中断，并自动重载
  timerAlarm(dacTimer, DAC_UPDATE_INTERVAL_US, true, 0);

  Serial.printf("硬件定时器已正确启动，中断频率: %u Hz (每 %d 微秒)\n", 1000000 / DAC_UPDATE_INTERVAL_US, DAC_UPDATE_INTERVAL_US);
  
  calculateAndApplySettings();
  lastModeSwitchTime = millis();
  
  Serial.println("=== 初始化完成，系统正在运行 ===");
}

void loop() {
  // 安全的ISR日志输出逻辑
  static long lastLoggedCount = 0;
  if (zeroCrossTriggerCount != lastLoggedCount) {
    long currentCount = zeroCrossTriggerCount;
    unsigned long triggerTime = lastIsrTimestamp;
    Serial.printf("[ISR LOG] 过零检测触发! 总次数: %ld, 时间戳: %lu µs\n", currentCount, triggerTime);
    lastLoggedCount = currentCount;
  }

  // 模式切换逻辑
  if (millis() - lastModeSwitchTime >= MODE_SWITCH_INTERVAL_MS) {
    lastModeSwitchTime = millis();
    switchTestMode();
  }
  
  // 定时打印状态信息
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > STATUS_PRINT_INTERVAL_MS) {
    lastPrintTime = millis();
    Serial.println("--------------------");
    Serial.printf("模式: %d, 目标功率: %.1f kW\n", testMode, currentPower / 1000.0);
    Serial.printf("计算电流: %.2f A\n", outputCurrent);
    portENTER_CRITICAL(&sharedDataMux);
    float ratio = sharedAmplitudeRatio;
    bool invert = sharedPhaseInvert;
    portEXIT_CRITICAL(&sharedDataMux);
    Serial.printf("输出幅值比例: %.1f%%\n", ratio * 100.0);
    Serial.printf("相位: %s\n", invert ? "反相 (180°)" : "正相 (0°)");
  }
  
  delay(10);
}

void calculateAndApplySettings() {
  if (TEST_VOLTAGE > 0) {
    outputCurrent = currentPower / TEST_VOLTAGE;
  } else {
    outputCurrent = 0;
  }
  
  if (outputCurrent > MAX_CURRENT_A) outputCurrent = MAX_CURRENT_A;
  if (outputCurrent < -MAX_CURRENT_A) outputCurrent = -MAX_CURRENT_A;

  float ampRatio = abs(outputCurrent) / MAX_CURRENT_A;
  bool phaseInvert = (currentPower < 0);

  portENTER_CRITICAL(&sharedDataMux);
  sharedAmplitudeRatio = ampRatio;
  sharedPhaseInvert = phaseInvert;
  portEXIT_CRITICAL(&sharedDataMux);
}

void switchTestMode() {
  testMode = (testMode + 1) % NUM_TEST_MODES;
  currentPower = TEST_POWERS[testMode];
  
  Serial.println("\n>>> 切换测试模式 <<<");
  
  calculateAndApplySettings();
}
