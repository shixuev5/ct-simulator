#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h> // 引入队列库

// =====================================================================
// == 常量定义
// =====================================================================
#define ZERO_CROSS_PIN 4

// --- I2C 与 DAC 配置 ---
const uint32_t I2C_HZ = 400000;         // I2C时钟频率: 400 KHz
const uint32_t SAMPLE_RATE_HZ = 12800;   // DAC采样率: 20 kHz

// --- DDS ---
#define LUT_SIZE 256
const float TARGET_FREQ_HZ = 50.0f;

// --- 系统参数 ---
#define DAC_RESOLUTION 4095
#define ZERO_OFFSET 2047
#define AMPLITUDE_SCALE_BITS 10
#define AMPLITUDE_SCALE_FACTOR 1024

// --- 模拟测试参数 ---
const float TEST_POWERS[] = {-22000.0, -11000.0, 0.0, 11000.0, 22000.0};
const int NUM_TEST_MODES = sizeof(TEST_POWERS) / sizeof(float);
#define TEST_VOLTAGE 220
#define MAX_CURRENT_A 100

// =====================================================================
// == 全局对象
// =====================================================================
Adafruit_MCP4725 dac;
hw_timer_t* dacTimer = nullptr;
QueueHandle_t dacQueue; // 定义队列句柄

// --- 任务句柄 ---
TaskHandle_t dacTaskHandle = NULL;

// --- DDS 变量 ---
uint16_t sineLUT[LUT_SIZE];
volatile uint32_t phase = 0;
uint32_t phaseStep = 0;

// --- 共享数据 (Core 1 loop 写, ISR 读) ---
portMUX_TYPE sharedDataMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t sharedAmplitudeScale = 0;
volatile bool sharedPhaseInvert = false;

// --- 应用状态 ---
uint8_t testMode = 0;
volatile unsigned long lastZeroCrossTime = 0;

// =====================================================================
// == 核心功能函数
// =====================================================================

// --- Core 0 专用任务: 处理 I2C 发送 ---
void dacTask(void *parameter) {
  uint16_t valueToSend;
  
  // 无限循环，类似 loop()
  for(;;) {
    // 从队列接收数据
    // portMAX_DELAY 表示如果队列空了，就一直阻塞等待，不占用CPU资源
    if (xQueueReceive(dacQueue, &valueToSend, portMAX_DELAY) == pdTRUE) {
      // 只有收到数据才会执行到这里
      dac.setVoltage(valueToSend, false);
    }
  }
}

// --- 中断服务程序 (ISR) ---
void IRAM_ATTR onTimer() {
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
    if (phaseInvert) index += (LUT_SIZE / 2);
    uint16_t baseDacValue = sineLUT[index];
    int32_t delta = baseDacValue - ZERO_OFFSET;
    int32_t scaledDelta = (delta * ampScale) >> AMPLITUDE_SCALE_BITS;
    int32_t val = ZERO_OFFSET + scaledDelta;
    
    if (val < 0) val = 0;
    else if (val > DAC_RESOLUTION) val = DAC_RESOLUTION;
    finalDacValue = (uint16_t)val;
  }

  // --- 发送到队列 ---
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // 将计算好的值发给 Core 0 的任务
  // 如果队列满了（说明I2C来不及发），这里会丢弃最新的点，保证ISR不阻塞
  xQueueSendFromISR(dacQueue, &finalDacValue, &xHigherPriorityTaskWoken);
  
  // 如果唤醒了高优先级任务（dacTask），进行上下文切换
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR onZeroCross() {
  unsigned long now = micros();
  if (now - lastZeroCrossTime < 4000) return;
  lastZeroCrossTime = now;
  
  static bool ignorePulse = false;
  ignorePulse = !ignorePulse;
  if (ignorePulse) phase = 0;
}

// --- 初始化辅助函数 ---
void initI2C_DAC() {
  Wire.begin();
  Wire.setClock(I2C_HZ);
  if (!dac.begin(0x60)) {
    Serial.println("DAC Init Failed");
    while (1);
  }
}

void generateSineLUT() {
  for (int i = 0; i < LUT_SIZE; i++) {
    float angle = (2.0 * PI * i) / LUT_SIZE;
    sineLUT[i] = (uint16_t)roundf(2047.0f + 2047.0f * sinf(angle));
  }
}

// =====================================================================
// == Setup & Loop
// =====================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("System Starting...");

  initI2C_DAC();
  generateSineLUT();
  
  // 计算相位步长
  double ratio = (double)TARGET_FREQ_HZ / (double)SAMPLE_RATE_HZ;
  phaseStep = (uint32_t)llround(ratio * 4294967296.0);

  // 1. 创建队列：长度为 10，每个单元存储 uint16_t
  dacQueue = xQueueCreate(10, sizeof(uint16_t));
  if (dacQueue == NULL) {
    Serial.println("Error creating queue");
    while(1);
  }

  // 2. 创建 Core 0 任务
  // 参数: 函数名, 任务名, 栈大小, 参数, 优先级, 句柄指针, 核心编号(0)
  // 优先级设为 10 (高优先级)，确保 I2C 发送优先于其他系统任务
  xTaskCreatePinnedToCore(
    dacTask,      
    "DACTask",    
    4096,         
    NULL,         
    10,           
    &dacTaskHandle,
    0             
  );
  Serial.println("DAC Task started on Core 0");

  // 3. 设置过零中断
  pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), onZeroCross, RISING);

  // 4. 启动硬件定时器 (必须最后启动)
  dacTimer = timerBegin(1000000U);
  timerAttachInterrupt(dacTimer, &onTimer);
  timerAlarm(dacTimer, 1000000U / SAMPLE_RATE_HZ, true, 0);

  Serial.println("Timer started");
}

void updateSharedData(float power) {
  float current = power / TEST_VOLTAGE;
  if (current > MAX_CURRENT_A) current = MAX_CURRENT_A;
  if (current < -MAX_CURRENT_A) current = -MAX_CURRENT_A;
  
  portENTER_CRITICAL(&sharedDataMux);
  sharedAmplitudeScale = (uint16_t)((abs(current)/MAX_CURRENT_A) * AMPLITUDE_SCALE_FACTOR);
  sharedPhaseInvert = (power < 0);
  portEXIT_CRITICAL(&sharedDataMux);
}

void loop() {
  // --- Core 1 逻辑 ---
  // 这里是原来的主循环，现在可以随意阻塞、延时、打印，完全不会影响波形输出
  
  static unsigned long lastSwitch = 0;
  static unsigned long lastPrint = 0;
  
  // 模式切换逻辑
  if (millis() - lastSwitch > 30000) {
    lastSwitch = millis();
    testMode = (testMode + 1) % NUM_TEST_MODES;
    updateSharedData(TEST_POWERS[testMode]);
    Serial.printf("Mode Switched to %d\n", testMode);
  }

  // 打印逻辑
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.printf("[Core 1] Running... Mode: %d, Power: %.0f\n", testMode, TEST_POWERS[testMode]);
    // 甚至可以检查 Core 0 任务的状态
    // UBaseType_t spaces = uxQueueSpacesAvailable(dacQueue);
    // Serial.printf("Queue spaces: %d\n", spaces);
  }

  // 适当延时，防止 Core 1 看门狗触发（虽然 Arduino loop 本身有处理）
  delay(10); 
}