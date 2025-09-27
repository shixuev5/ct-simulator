/*
 * CT模拟器 - 融合功能版 v2
 * 功能: 
 * - 监听50Hz市电过零信号 (处理100Hz脉冲)。
 * - 通过MCP4725生成同相位的正弦波。
 * - 每30秒自动切换模式，模拟 -22kW, -11kW, 0, 11kW, 22kW 功率。
 * - 动态计算并调整输出正弦波的幅值和相位。
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

// =========================================================
// ===             新修改的模拟测试参数                  ===
// =========================================================
#define MAX_CURRENT_A 100.0   // 22000W / 220V = 100A
#define TEST_VOLTAGE 220.0    // 假设的市电电压
#define MODE_SWITCH_INTERVAL_MS 30000 // 模式切换间隔30秒
#define STATUS_PRINT_INTERVAL_MS 2000 // 状态打印间隔2秒

// 定义不同的测试功率 (单位: W)
const float TEST_POWERS[] = {-22000.0, -11000.0, 0.0, 11000.0, 22000.0};
const int NUM_TEST_MODES = sizeof(TEST_POWERS) / sizeof(float);
// =========================================================


// 全局对象和变量
Adafruit_MCP4725 dac;
hw_timer_t *dacTimer = NULL;

uint16_t sineTable[SINE_TABLE_SIZE];
volatile int sineIndex = 0;

// 防抖时间，单位微秒。4ms足以屏蔽噪声，远小于10ms的脉冲间隔。
const unsigned long DEBOUNCE_MICROS = 4000; 
// 用于存储上一次有效触发的时间戳
volatile unsigned long lastZeroCrossTime = 0;

uint8_t testMode = 0;
float currentPower = TEST_POWERS[0];
float outputCurrent = 0;
unsigned long lastModeSwitchTime = 0;

portMUX_TYPE sharedDataMux = portMUX_INITIALIZER_UNLOCKED;
volatile float sharedAmplitudeRatio = 0.0;
volatile bool sharedPhaseInvert = false;

// 函数声明
void calculateAndApplySettings();
void switchTestMode();

/**
 * @brief 过零中断 - 包含防抖和50Hz同步逻辑
 */
void IRAM_ATTR onZeroCross() {
  unsigned long now = micros();

  // 检查当前时间与上一次有效触发的时间间隔是否足够长
  if (now - lastZeroCrossTime < DEBOUNCE_MICROS) {
    // 时间间隔太短，这很可能是噪声或抖动，直接忽略并返回
    return;
  }

  // 如果程序能执行到这里，说明这是一次有效的、无抖动的触发
  lastZeroCrossTime = now; // 更新时间戳

  // 使用静态变量来忽略掉100Hz脉冲中的一半，得到50Hz的同步信号
  static bool ignorePulse = false;
  ignorePulse = !ignorePulse;

  if (ignorePulse) {
    // 只有在不忽略的脉冲上，才重置相位
    sineIndex = 0;
  }
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
  Serial.println("=== CT模拟器 - 融合功能版 v2.1 (API已更新) ===");
  Serial.print("测试功率序列: ");
  for(int i=0; i<NUM_TEST_MODES; i++){
    Serial.printf("%.1fkW ", TEST_POWERS[i]/1000.0);
  }
  Serial.println();


  Wire.begin();
  Wire.setClock(400000); // 设置I2C为400kHz快速模式

  if (!dac.begin(0x60)) {
    Serial.println("错误: MCP4725 DAC未找到!");
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

  // 新的API直接使用频率(Hz)进行初始化。
  // 我们的更新间隔是200µs，所以频率是 1,000,000 / 200 = 5000 Hz。
  const uint32_t frequency = 1000000 / DAC_UPDATE_INTERVAL_US;

  // 1. 直接用频率初始化定时器。它会自动启动。
  dacTimer = timerBegin(frequency);
  // 2. 附加中断服务程序。
  timerAttachInterrupt(dacTimer, &onTimer);
  // =====================================================================

  Serial.printf("硬件定时器已启动，中断频率: %u Hz (每 %d 微秒)\n", frequency, DAC_UPDATE_INTERVAL_US);
  
  calculateAndApplySettings();
  lastModeSwitchTime = millis();
  
  Serial.println("=== 初始化完成，系统正在运行 ===");
}

void loop() {
  if (millis() - lastModeSwitchTime >= MODE_SWITCH_INTERVAL_MS) {
    lastModeSwitchTime = millis();
    switchTestMode();
  }
  
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
