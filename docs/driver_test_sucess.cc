/**
 * ZY-ESP32 双协议舵机控制器
 * 支持：
 *   幻尔LX-16A总线舵机 (ID 21~34) - 二进制协议
 *   众灵舵机 (ID 35~43) - ASCII字符串协议 (#001P0500T0500!)
 * 
 * 硬件连接：
 *   舵机总线 UART2 -> GPIO16(TX), GPIO17(RX)
 *   使用硬件转单线电路（无需方向控制）
 */

#include <HardwareSerial.h>

// ==================== 舵机参数 ====================
// 幻尔LX-16A舵机 (ID 21~34)
const int HIWONDER_START_ID = 21;
const int HIWONDER_END_ID = 34;
const int HIWONDER_COUNT = 14;
const uint8_t hiwonderIds[HIWONDER_COUNT] = {21,22,23,24,25,26,27,28,29,30,31,32,33,34};

// 众灵舵机 (ID 35~43)
const int ZONGLING_START_ID = 35;
const int ZONGLING_END_ID = 43;
const int ZONGLING_COUNT = 9;
const uint8_t zonglingIds[ZONGLING_COUNT] = {35,36,37,38,39,40,41,42,43};

// ==================== LX-16A指令码 ====================
#define CMD_MOVE_TIME_WRITE      0x01
#define CMD_TORQUE_SWITCH        0x1E

// 舵机总线 - 使用UART2
HardwareSerial ServoSerial(2);

// ==================== LX-16A协议函数 ====================

/**
 * 计算校验和
 * 规则：~(ID + 长度 + 指令 + 参数...) 的低8位
 */
uint8_t lx16a_checksum(uint8_t* buf, int len) {
  uint16_t sum = 0;
  for (int i = 2; i < len - 1; i++) sum += buf[i];
  return (uint8_t)(~sum);
}

/**
 * 发送数据包（硬件自动处理收发切换）
 */
void sendPacket(uint8_t* packet, int len) {
  ServoSerial.write(packet, len);
  ServoSerial.flush();
}

/**
 * 控制幻尔LX-16A舵机移动到指定位置
 * @param id: 舵机ID 21~34
 * @param position: 位置 0~1000（对应0~240度）
 * @param time_ms: 移动时间 0~30000ms
 */
bool hiwonderMove(uint8_t id, uint16_t position, uint16_t time_ms) {
  uint8_t packet[10];
  packet[0] = 0x55; packet[1] = 0x55;
  packet[2] = id;
  packet[3] = 7;
  packet[4] = CMD_MOVE_TIME_WRITE;
  packet[5] = position & 0xFF;
  packet[6] = (position >> 8) & 0xFF;
  packet[7] = time_ms & 0xFF;
  packet[8] = (time_ms >> 8) & 0xFF;
  packet[9] = lx16a_checksum(packet, 10);
  sendPacket(packet, 10);
  delay(5); // 短暂延时避免总线拥堵
  return true;
}

/**
 * 设置幻尔舵机扭矩开关
 */
bool hiwonderSetTorque(uint8_t id, bool enable) {
  uint8_t packet[7];
  packet[0] = 0x55; packet[1] = 0x55;
  packet[2] = id;
  packet[3] = 4;
  packet[4] = CMD_TORQUE_SWITCH;
  packet[5] = enable ? 1 : 0;
  packet[6] = lx16a_checksum(packet, 7);
  sendPacket(packet, 7);
  delay(5);
  return true;
}

// ==================== 众灵舵机协议函数 ====================

/**
 * 格式化众灵舵机指令
 * 格式: #001P1500T1000!
 * ID必须3位，PWM和TIME必须4位，不足补0
 */
void formatZonglingCmd(char* buffer, uint8_t id, uint16_t pwm, uint16_t time_ms) {
  // 使用sprintf格式化，%03d表示3位数字补0，%04d表示4位数字补0
  sprintf(buffer, "#%03dP%04dT%04d!", id, pwm, time_ms);
}

/**
 * 控制众灵舵机移动到指定位置
 * @param id: 舵机ID 35~43
 * @param pwm: PWM值 500~2500（对应0~270度）
 * @param time_ms: 移动时间 0~9999ms
 */
bool zonglingMove(uint8_t id, uint16_t pwm, uint16_t time_ms) {
  char cmd[20]; // 足够容纳指令 "#035P1500T1000!" 约18字节
  formatZonglingCmd(cmd, id, pwm, time_ms);
  
  // 发送字符串指令
  ServoSerial.print(cmd);
  
  // 可选：串口打印调试
  // Serial.printf("发送众灵指令: %s\n", cmd);
  
  delay(5); // 短暂延时避免总线拥堵
  return true;
}

/**
 * 同时控制多个众灵舵机（使用{}包裹多条指令）
 * @param count: 舵机数量
 * @param ids: 舵机ID数组
 * @param pwms: PWM值数组
 * @param times: 时间数组
 */
void zonglingMultiMove(int count, uint8_t* ids, uint16_t* pwms, uint16_t* times) {
  // 估算所需缓冲区大小：每个指令约18字节 + 2字节花括号 + 1字节结束符
  char buffer[200] = "{";
  char cmd[20];
  
  for (int i = 0; i < count; i++) {
    formatZonglingCmd(cmd, ids[i], pwms[i], times[i]);
    strcat(buffer, cmd);
  }
  strcat(buffer, "}");
  
  ServoSerial.print(buffer);
  delay(10);
}

// ==================== 角度转换函数 ====================

/**
 * 将幻尔角度值转换为PWM值（用于众灵舵机）
 * 幻尔: 0~1000 对应 0~240度
 * 众灵: 500~2500 对应 0~270度
 * 为了方便演示，统一使用幻尔位置值，然后映射到众灵PWM
 */
uint16_t hiwonderPosToZonglingPWM(uint16_t hiwonderPos) {
  // hiwonderPos范围: 0~1000
  // 映射到众灵PWM范围: 500~2500
  // 公式: pwm = 500 + (hiwonderPos * 2000 / 1000)
  return 500 + (hiwonderPos * 2); // 简化计算：hiwonderPos*2 正好 0~1000 -> 500~2500
}

// ==================== 初始化 ====================

void setup() {
  Serial.begin(115200);
  Serial.println("ZY-ESP32 双协议舵机控制器启动");
  
  // 初始化舵机总线
  ServoSerial.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("舵机总线初始化完成 (UART2, GPIO16 TX, GPIO17 RX)");
  
  // 使能所有幻尔舵机扭矩
  Serial.println("正在初始化幻尔LX-16A舵机 (ID 21~34)...");
  for (int i = 0; i < HIWONDER_COUNT; i++) {
    if (hiwonderSetTorque(hiwonderIds[i], true)) {
      Serial.printf("幻尔舵机 %d 扭矩使能成功\n", hiwonderIds[i]);
    } else {
      Serial.printf("幻尔舵机 %d 扭矩使能失败\n", hiwonderIds[i]);
    }
    delay(10);
  }
  
  Serial.println("众灵舵机初始化完成 (ID 35~43)");
  Serial.println("系统就绪，开始运动演示");
  Serial.println("=====================================");
}

// ==================== 主循环 ====================

void loop() {
  // 演示1：幻尔舵机逐个摆动
  Serial.println("\n--- 幻尔LX-16A舵机运动演示 (ID 21~34) ---");
  for (int i = 0; i < HIWONDER_COUNT; i++) {
    uint8_t id = hiwonderIds[i];
    
    // 转到0度 (位置0)
    Serial.printf("幻尔舵机 %d -> 0度\n", id);
    hiwonderMove(id, 0, 1000);
    delay(1000);
    
    // 转到240度 (位置1000)
    Serial.printf("幻尔舵机 %d -> 240度\n", id);
    hiwonderMove(id, 1000, 1000);
    delay(1000);
    
    // 回到中间120度 (位置500)
    Serial.printf("幻尔舵机 %d -> 120度\n", id);
    hiwonderMove(id, 500, 800);
    delay(800);
  }
  
  // 演示2：众灵舵机逐个摆动
  Serial.println("\n--- 众灵舵机运动演示 (ID 35~43) ---");
  for (int i = 0; i < ZONGLING_COUNT; i++) {
    uint8_t id = zonglingIds[i];
    
    // 转到0度 (PWM 500)
    Serial.printf("众灵舵机 %d -> 0度\n", id);
    zonglingMove(id, 500, 1000);
    delay(1000);
    
    // 转到270度 (PWM 2500)
    Serial.printf("众灵舵机 %d -> 270度\n", id);
    zonglingMove(id, 2500, 1000);
    delay(1000);
    
    // 回到中间 (PWM 1500)
    Serial.printf("众灵舵机 %d -> 135度\n", id);
    zonglingMove(id, 1500, 800);
    delay(800);
  }
  
  // 演示3：所有舵机同步动作（使用幻尔位置值映射到众灵PWM）
  Serial.println("\n--- 所有舵机同步运动 ---");
  
  // 所有舵机转到0度
  Serial.println("所有舵机 -> 0度");
  
  // 幻尔舵机
  for (int i = 0; i < HIWONDER_COUNT; i++) {
    hiwonderMove(hiwonderIds[i], 0, 2000);
    delay(2);
  }
  
  // 众灵舵机
  for (int i = 0; i < ZONGLING_COUNT; i++) {
    zonglingMove(zonglingIds[i], 500, 2000); // 500对应0度
    delay(2);
  }
  delay(2000);
  
  // 所有舵机转到中间位置
  Serial.println("所有舵机 -> 中间位置");
  
  // 幻尔舵机到120度 (位置500)
  for (int i = 0; i < HIWONDER_COUNT; i++) {
    hiwonderMove(hiwonderIds[i], 500, 1500);
    delay(2);
  }
  
  // 众灵舵机到135度 (PWM 1500)
  for (int i = 0; i < ZONGLING_COUNT; i++) {
    zonglingMove(zonglingIds[i], 1500, 1500);
    delay(2);
  }
  delay(1500);
  
  // 所有舵机转到最大角度
  Serial.println("所有舵机 -> 最大角度");
  
  // 幻尔舵机到240度 (位置1000)
  for (int i = 0; i < HIWONDER_COUNT; i++) {
    hiwonderMove(hiwonderIds[i], 1000, 2000);
    delay(2);
  }
  
  // 众灵舵机到270度 (PWM 2500)
  for (int i = 0; i < ZONGLING_COUNT; i++) {
    zonglingMove(zonglingIds[i], 2500, 2000);
    delay(2);
  }
  delay(2000);
  
  // 演示4：众灵舵机组合指令示例（同时控制多个）
  Serial.println("\n--- 众灵舵机组合指令演示 (同时控制3个舵机) ---");
  
  uint8_t multiIds[3] = {35, 36, 37};
  uint16_t multiPwms[3] = {500, 1500, 2500};  // 0度, 135度, 270度
  uint16_t multiTimes[3] = {1000, 1000, 1000};
  
  zonglingMultiMove(3, multiIds, multiPwms, multiTimes);
  delay(1000);
  
  multiPwms[0] = 2500; multiPwms[1] = 500; multiPwms[2] = 1500;
  zonglingMultiMove(3, multiIds, multiPwms, multiTimes);
  delay(1000);
  
  Serial.println("\n=====================================");
  Serial.println("一轮演示完成，10秒后开始下一轮");
  delay(10000);
}

