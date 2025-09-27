/*
 * CT模拟器 - 最终功能完整版 v2.3 (采用原生I2C指令)
 * 修正: 解决了DAC输出固定在2.5V的问题。
 * ... (其余注释不变) ...
 */

#include <Wire.h> // 只需Wire库
#include <math.h>
#include <freertos/FreeRTOS.h>

// I2C地址定义
#define MCP4725_ADDR 0x60

// ... (所有其他宏定义和全局变量不变) ...
//硬件引脚定义
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

// onZeroCross 函数保持不变
void IRAM_ATTR onZeroCross() {
  unsigned long now = micros();
  if (now - lastZeroCrossTime < DEBOUNCE_MICROS) { return; }
  lastZeroCrossTime = now;
  static bool ignorePulse = false;
  ignorePulse = !ignorePulse;
  if (ignorePulse) { sineIndex = 0; }
  zeroCrossTriggerCount++;
  lastIsrTimestamp = now;
}

/**
 * @brief 定时器中断 (修正版) - 使用原生Wire指令与MCP4725通信
 */
void IRAM_ATTR onTimer() {
    float amplitudeRatio;
    bool phaseInvert;
    
    portENTER_CRITICAL_ISR(&sharedDataMux);
    amplitudeRatio = sharedAmplitudeRatio;
    phaseInvert = sharedPhaseInvert;
    portEXIT_CRITICAL_ISR(&sharedDataMux);

    // 计算最终的12位DAC值 (这部分逻辑是正确的，保持不变)
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

    // =========================================================================
    // ===                 核心修正：替换 dac.setVoltage()                   ===
    // 使用从SparkFun示例学到的、对ISR友好的原生I2C指令
    // =========================================================================
    Wire.beginTransmission(MCP4725_ADDR);
    Wire.write(0x40); // 命令字节: 更新DAC寄存器
    Wire.write((uint8_t)(finalDacValue >> 4));        // 发送数据的高8位
    Wire.write((uint8_t)((finalDacValue & 15) << 4)); // 发送数据的低4位
    Wire.endTransmission();
    // =========================================================================

    // 索引递增逻辑保持不变
    sineIndex = (sineIndex + 1) % SINE_TABLE_SIZE;
}


void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== CT模拟器 - v2.3 (原生I2C指令) ===");
  // ... (其余日志不变) ...

  Wire.begin();
  Wire.setClock(400000); 

  // 由于不再使用Adafruit库，移除其初始化和检查代码
  Serial.println("I2C总线已初始化 (快速模式: 400kHz).");

  // ... (其余setup代码，如查找表生成、过零中断、定时器设置，完全保持不变) ...
  Serial.print("测试功率序列: ");
  for(int i=0; i<NUM_TEST_MODES; i++){
    Serial.printf("%.1fkW ", TEST_POWERS[i]/1000.0);
  }
  Serial.println();
  Serial.println("正在生成正弦波查找表...");
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    float angle = (2.0 * PI * i) / SINE_TABLE_SIZE;
    sineTable[i] = (uint16_t)((sin(angle) + 1.0) * (DAC_RESOLUTION / 2.0));
  }
  
  pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), onZeroCross, RISING);
  Serial.printf("过零检测中断已附加到 GPIO%d\n", ZERO_CROSS_PIN);

  const uint32_t frequency = 1000000 / DAC_UPDATE_INTERVAL_US;
  dacTimer = timerBegin(frequency);
  timerAttachInterrupt(dacTimer, &onTimer);
  Serial.printf("硬件定时器已启动，中断频率: %u Hz (每 %d 微秒)\n", frequency, DAC_UPDATE_INTERVAL_US);
  
  calculateAndApplySettings();
  lastModeSwitchTime = millis();
  
  Serial.println("=== 初始化完成，系统正在运行 ===");
}

// loop函数和辅助函数calculateAndApplySettings, switchTestMode完全保持不变
void loop() {
  static long lastLoggedCount = 0;
  if (zeroCrossTriggerCount != lastLoggedCount) {
    long currentCount = zeroCrossTriggerCount;
    unsigned long triggerTime = lastIsrTimestamp;
    Serial.printf("[ISR LOG] 过零检测触发! 总次数: %ld, 时间戳: %lu µs\n", currentCount, triggerTime);
    lastLoggedCount = currentCount;
  }

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
