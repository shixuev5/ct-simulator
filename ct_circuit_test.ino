/*
 * CT模拟器 - 电路测试版本
 * 功能：输出固定电压值，方便使用万用表测量电路是否正确
 * 测试：验证MCP4725 DAC输出和I2C通信
 * 
 * 测试模式：
 * - 模式0: 0V (0 数字值)
 * - 模式1: 2.5V (2048 数字值, 零点)  
 * - 模式2: 5V (4095 数字值, 最大值)
 * - 模式3: 1.25V (1024 数字值)
 * - 模式4: 3.75V (3072 数字值)
 * 
 */

#include <Wire.h>
#include <Adafruit_MCP4725.h>

// 系统参数
#define DAC_RESOLUTION 4095 // 12位DAC分辨率 (0-4095)
#define VOLTAGE_REF 5.0     // 参考电压5.0V (MCP4725输出0-5V)
#define ZERO_OFFSET 2048    // DAC零点偏移值(2.5V)

// CT参数
#define CT_RATIO 2000       // CT变比 2000:1
#define MAX_CURRENT_A 100   // 最大电流限制100A

// 时序参数
#define STATUS_PRINT_INTERVAL_MS 2000 // 状态打印间隔2秒
#define MODE_SWITCH_INTERVAL_MS 30000 // 模式切换间隔30秒

// 测试电压值 (DAC数字值)
#define TEST_VOLTAGE_0V    0     // 0V
#define TEST_VOLTAGE_1_25V 1024  // 1.25V
#define TEST_VOLTAGE_2_5V  2048  // 2.5V (零点)
#define TEST_VOLTAGE_3_75V 3072  // 3.75V
#define TEST_VOLTAGE_5V    4095  // 5V (最大值)

// 全局变量
Adafruit_MCP4725 dac;

// 测试模式变量
uint8_t testMode = 0;  // 0-4 对应5种测试电压
unsigned long lastModeSwitch = 0;
uint16_t currentDacValue = TEST_VOLTAGE_2_5V; // 默认从2.5V开始
float simulatedCurrent = 0; // 模拟的最终电流值

// 测试电压数组
const uint16_t testVoltages[5] = {
  TEST_VOLTAGE_0V,     // 模式0: 0V
  TEST_VOLTAGE_2_5V,   // 模式1: 2.5V (零点)
  TEST_VOLTAGE_5V,     // 模式2: 5V (最大值)
  TEST_VOLTAGE_1_25V,  // 模式3: 1.25V
  TEST_VOLTAGE_3_75V   // 模式4: 3.75V
};

const char* testVoltageNames[5] = {
  "0V (最小值)",
  "2.5V (零点)",
  "5V (最大值)", 
  "1.25V (1/4量程)",
  "3.75V (3/4量程)"
};

void setup() {
  Serial.begin(115200);
  Serial.println("=== CT模拟器电路测试版本 ===");
  Serial.println("功能: 输出固定电压值，方便万用表测量");
  Serial.println("测试: MCP4725 DAC输出和I2C通信");
  Serial.println("");
  
  // 初始化I2C总线
  Wire.begin();
  Wire.setClock(400000); // 设置I2C为400kHz快速模式
  Serial.println("I2C总线初始化完成 (400kHz)");
  
  // 初始化MCP4725 DAC (默认I2C地址0x60)
  if (!dac.begin()) {
    Serial.println("❌ MCP4725 DAC初始化失败!");
    Serial.println("请检查:");
    Serial.println("1. MCP4725模块连接是否正确");
    Serial.println("2. I2C地址是否正确 (默认0x60)");
    Serial.println("3. 电源供电是否正常");
    while (1) {
      delay(1000);
      Serial.println("等待硬件修复...");
    }
  }
  Serial.println("✅ MCP4725 DAC初始化成功");
  
  // 设置初始输出电压 (2.5V零点)
  setTestVoltage(1); // 从模式1 (2.5V) 开始
  
  Serial.println("");
  Serial.println("=== 系统初始化完成 ===");
  Serial.printf("DAC分辨率: %d 位 (0-%d)\n", 12, DAC_RESOLUTION);
  Serial.printf("输出电压范围: 0-%.1fV\n", VOLTAGE_REF);
  Serial.printf("模式切换间隔: %d 秒\n", MODE_SWITCH_INTERVAL_MS/1000);
  Serial.printf("CT变比: %d:1\n", CT_RATIO);
  Serial.printf("最大电流限制: ±%d A\n", MAX_CURRENT_A);
  Serial.printf("状态更新间隔: %d 秒\n", STATUS_PRINT_INTERVAL_MS/1000);
  Serial.println("");
  Serial.println("📏 请使用万用表测量DAC输出引脚电压");
  Serial.println("🔄 系统将自动切换测试模式");
  Serial.println("开始电路测试...");
  Serial.println("==========================================");
}

void loop() {
  static unsigned long lastPrint = 0;
  
  // 每2秒打印一次状态
  if (millis() - lastPrint >= STATUS_PRINT_INTERVAL_MS) {
    lastPrint = millis();
    printStatus();
  }
  
  // 每30秒切换测试模式
  if (millis() - lastModeSwitch >= MODE_SWITCH_INTERVAL_MS) {
    lastModeSwitch = millis();
    switchTestMode();
  }
  
  // 短暂延时
  delay(100);
}

void setTestVoltage(uint8_t mode) {
  if (mode >= 5) mode = 0; // 防止数组越界
  
  testMode = mode;
  currentDacValue = testVoltages[mode];
  
  // 设置DAC输出
  dac.setVoltage(currentDacValue, false);
  
  // 计算实际输出电压
  float actualVoltage = (currentDacValue * VOLTAGE_REF) / DAC_RESOLUTION;
  
  // 计算模拟的最终电流值 (基于DAC电压输出)
  // 假设: DAC电压 -> 运放放大 -> CT次级电流
  // 2.5V为零点，偏离零点的电压对应电流
  float voltageOffset = actualVoltage - (VOLTAGE_REF / 2.0); // 相对于2.5V的偏移
  simulatedCurrent = (voltageOffset / (VOLTAGE_REF / 2.0)) * MAX_CURRENT_A; // 归一化到最大电流
  
  Serial.println("==========================================");
  Serial.printf("🔧 切换到测试模式 %d: %s\n", mode, testVoltageNames[mode]);
  Serial.printf("📊 DAC数字值: %d (0x%03X)\n", currentDacValue, currentDacValue);
  Serial.printf("⚡ 理论输出: %.3f V\n", actualVoltage);
  Serial.printf("🔌 模拟电流: %.2f A (CT次级)\n", simulatedCurrent);
  Serial.printf("📐 等效一次电流: %.2f A\n", simulatedCurrent * CT_RATIO);
  Serial.println("📏 请用万用表测量实际输出电压");
  Serial.println("==========================================");
}

void printStatus() {
  float theoreticalVoltage = (currentDacValue * VOLTAGE_REF) / DAC_RESOLUTION;
  float percentage = (currentDacValue * 100.0) / DAC_RESOLUTION;
  
  Serial.printf("📈 当前状态 [模式%d]: %s\n", testMode, testVoltageNames[testMode]);
  Serial.printf("   - DAC数字值: %d / %d (%.1f%%)\n", currentDacValue, DAC_RESOLUTION, percentage);
  Serial.printf("   - 理论电压: %.3f V\n", theoreticalVoltage);
  Serial.printf("   - 模拟电流: %.2f A (CT次级)\n", simulatedCurrent);
  Serial.printf("   - 等效一次电流: %.2f A\n", simulatedCurrent * CT_RATIO);
  Serial.printf("   - 下次切换: %d 秒后\n", (MODE_SWITCH_INTERVAL_MS - (millis() - lastModeSwitch))/1000);
  Serial.println("---");
}

void switchTestMode() {
  // 循环切换测试模式 0->1->2->3->4->0...
  uint8_t nextMode = (testMode + 1) % 5;
  setTestVoltage(nextMode);
}

// 串口命令处理 (可选功能)
void serialEvent() {
  while (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
        setTestVoltage(cmd - '0');
        Serial.printf("🎯 手动切换到模式 %c\n", cmd);
        break;
        
      case 'h':
      case 'H':
        printHelp();
        break;
        
      case 's':
      case 'S':
        printStatus();
        break;
        
      default:
        Serial.printf("❓ 未知命令: %c (输入 'h' 查看帮助)\n", cmd);
        break;
    }
  }
}

void printHelp() {
  Serial.println("");
  Serial.println("=== 手动控制命令 ===");
  Serial.println("0 - 设置 0V");
  Serial.println("1 - 设置 2.5V (零点)");
  Serial.println("2 - 设置 5V (最大值)");
  Serial.println("3 - 设置 1.25V");
  Serial.println("4 - 设置 3.75V");
  Serial.println("s - 显示当前状态");
  Serial.println("h - 显示此帮助信息");
  Serial.println("=====================");
}
