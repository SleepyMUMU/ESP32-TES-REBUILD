#include "FashionStar_UartServoProtocol.h" // 串口总线舵机通信协议
#include "FashionStar_UartServo.h"         // Fashion Star串口总线舵机的依赖
#include <BluetoothSerial.h>
#include <HardwareSerial.h>
#include <stdlib.h>
#include <string.h>

#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_system.h>

// 电磁炮充电配置
#define CHARGE_PORT 22          // 继电器充电控制PIN
#define DISCHARGE_PORT 23       // 继电器放电控制PIN
#define CLEAR_PORT 13           // 继电器清除剩余电量端口
#define DISCHARGE_SAFE_TIME 800 // 安全充电时间
bool CHARGE_CHECK_BOOL = 0;     // 电路状态BOOL
bool DISCHARGE_CHECK_BOOL = 0;
bool CLEAR_CHECK_BOOL = 0;
// CMD分析
uint16_t ANGLE_QUE_FLAG = 0; // 角度更新flag

uint8_t CMD[32] = {};      // 储存CMD
uint8_t CMD_Head[16] = {}; // 存储命令头
int CMD_Param = 0;         // 储存参数头

// 通讯
BluetoothSerial SerialBT;                      // 蓝牙协议
bool isConnected = false;                      // 蓝牙连接状态
#define DEBUG_SERIAL Serial                    // DEBUG串口
#define COM Serial1                            // 多机通讯串口
#define COM_SERIAL_BAUDRATE (uint32_t)115200   // COM波特率
#define DEBUG_SERIAL_BAUDRATE (uint32_t)115200 // DEBUG波特率

// 舵机配置
#define SERVO_X_ID 1                       // 舵机ID号
#define SERVO_Y_ID 2                       // 舵机ID号
#define BAUDRATE 115200                    // 舵机通讯波特率
FSUS_Protocol protocol(BAUDRATE);          // 协议
FSUS_Servo Servo_X(SERVO_X_ID, &protocol); // 创建舵机
FSUS_Servo Servo_Y(SERVO_Y_ID, &protocol); // 创建舵机
#define RAW_ANGLE_X 0                      // X轴初始值
#define RAW_ANGLE_Y 44                     // Y轴初始值

uint64_t elapsedTime = 0;

void CMD_CRL(void);
void BluTooTh_communication(void);
void CMD_Analysis(void);
void Security_detection(void);
void Velocity_Detection_init();

void IRAM_ATTR interruptHandler(void *arg)
{
    static uint64_t startTime = 0;
    uint64_t currentTime = esp_timer_get_time();

    // 检测上升沿触发
    if (gpio_get_level(GPIO_NUM_34) == 1)
    {
        startTime = currentTime; // 记录起始时间
        // delay(10);
    }
    else
    {
        if (startTime != 0)
        {
            elapsedTime = currentTime - startTime; // 计算通过时间差
            // 在这里处理通过时间，可以将其发送到串口、保存到变量等
            DEBUG_SERIAL.println(elapsedTime);
            startTime = 0; // 重置起始时间
            // Velocity_Detection_init();
        }
    }
}

void Velocity_Detection_init()
{
    // 配置外部中断引脚
    gpio_config_t gpioConfig;
    gpioConfig.pin_bit_mask = GPIO_SEL_34; // 外部中断引脚
    gpioConfig.mode = GPIO_MODE_INPUT;
    gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig.intr_type = GPIO_INTR_ANYEDGE; // 任何边沿触发
    gpio_config(&gpioConfig);

    // 安装中断服务程序
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_34, interruptHandler, NULL);
}

void setup()
{
    Velocity_Detection_init();

    // 通讯初始化d
    COM.begin(COM_SERIAL_BAUDRATE, SERIAL_8N1, 18, 19);
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // DEBUG串口初始化
    SerialBT.begin("ELCGUN");                  // 如果没有参数传入则默认是蓝牙名称是: "ESP32"
    SerialBT.setPin("1234");                   // 蓝牙连接的配对码
    DEBUG_SERIAL.println("UART connected.");
    COM.println("@COM connected.\r");
    // 舵机初始化
    DEBUG_SERIAL.println("Servo initing...");
    protocol.init(&Serial2, BAUDRATE); // 舵机通信协议初始化
    Servo_X.init();                    // 串口总线舵机初始化
    Servo_Y.init();                    // 串口总线舵机初始化
    Servo_X.setAngleRange(-120, 120);
    DEBUG_SERIAL.println("Servo inited.");

    // 电磁炮初始化
    DEBUG_SERIAL.println("ELCGUN initing...");
    pinMode(CHARGE_PORT, OUTPUT);    // 充电控制引脚配置
    pinMode(DISCHARGE_PORT, OUTPUT); // 放电控制引脚配置
    pinMode(CLEAR_PORT, OUTPUT);     // 去电控制引脚配置
    digitalWrite(CHARGE_PORT, LOW);
    digitalWrite(DISCHARGE_PORT, LOW);
    digitalWrite(CLEAR_PORT, LOW);
    CHARGE_CHECK_BOOL = 1;
    DEBUG_SERIAL.println("ELCGUN inited.");

    // 恢复初始姿态
    DEBUG_SERIAL.println("Backup Now...");
    Servo_Y.setRawAngle(RAW_ANGLE_Y, 1500); // backup
    delay(10);
    Servo_X.setRawAngle(RAW_ANGLE_X, 1500);
    delay(1500);
    DEBUG_SERIAL.println("Already Backup.");
}
void BT_status()
{
    if (SerialBT.connected() && !isConnected)
    {
        isConnected = true; // 更新连接状态为已连接
        DEBUG_SERIAL.println("Bluetooth connected.");
    }

    if (!SerialBT.connected() && isConnected)
    {
        isConnected = false; // 更新连接状态为未连接
        Serial.println("Bluetooth disconnected.");
    }
    if (!SerialBT.connected())
    {
        return;
    }
}
void loop()
{
    BT_status();
    BluTooTh_communication();
    Security_detection();
}

// void BlinkLED()
// {

// }

void BluTooTh_communication(void) // 接收蓝牙传入字符串
{
    if (SerialBT.available())
    {
        uint8_t i = 0;
        memset(CMD, 0, sizeof CMD); // 清空数组

        while (SerialBT.available() > 0)
        {
            CMD[i] = SerialBT.read();
            i++;
            delay(5);
        }

        CMD_Analysis();
    }

    // if (COM.available())
    // {
    //     uint8_t i = 0;
    //     memset(CMD, 0, sizeof CMD); // 清空数组

    //     while (COM.available() > 0)
    //     {
    //         CMD[i] = COM.read();
    //         i++;
    //         delay(5);
    //     }
    //     CMD_Analysis();
    // }

    if (DEBUG_SERIAL.available())
    {
        uint8_t i = 0;
        memset(CMD, 0, sizeof CMD); // 清空数组

        while (DEBUG_SERIAL.available() > 0)
        {
            CMD[i] = DEBUG_SERIAL.read();
            i++;
            delay(5);
        }
        CMD_Analysis();
    }

    if (COM.available())
    {
        // DEBUG_SERIAL.print("resv:");
        uint8_t i = 0;
        memset(CMD, 0, sizeof CMD); // 清空数组

        while (COM.available() > 0)
        {
            CMD[i] = COM.read();
            i++;
            delay(5);
        }
        //  String de1=(*char)CMD;
        // DEBUG_SERIAL.println((char *)CMD);
        CMD_Analysis();
    }
}

void COM_communication(void) // 接收蓝牙传入字符串
{
    if (COM.available())
    {
        uint8_t i = 0;
        memset(CMD, 0, sizeof CMD); // 清空数组

        while (SerialBT.available() > 0)
        {
            CMD[i] = SerialBT.read();
            i++;
            delay(5);
        }
        CMD_Analysis();
    }
}

void CMD_Analysis(void) // 指令分割
{

    // 分离字符串中的英文字符串和数字串

    memset(CMD_Head, 0, sizeof CMD_Head); // 清空数组
    CMD_Param = 0;                        // 储存参数头
    short int abs = 1;
    int i = 0;
    while (CMD[i] != '\0')
    {
        if ((CMD[i] >= 'A' && CMD[i] <= 'Z') || (CMD[i] >= 'a' && CMD[i] <= 'z'))
        {
            strncat((char *)CMD_Head, (char *)&CMD[i], 1); // 将英文字符追加到 alpha 中
        }
        else if (CMD[i] >= '0' && CMD[i] <= '9')
        {
            // 处理数字串
            int num = 0;
            while (CMD[i] >= '0' && CMD[i] <= '9')
            {
                num = num * 10 + (CMD[i] - '0');
                i++;
            }
            CMD_Param = num; // 将数字存储到 nums 数组中
        }
        else if (CMD[i] == '+')
        {
            abs = 1;
        }
        else if (CMD[i] == '-')
        {
            abs = -1;
        }

        i++;
    }
    // DEBUG_SERIAL.print(abs);
    CMD_Param *= abs;
    CMD_CRL();
}

void Security_detection(void) // 安全检查
{
    delay(1);
    ANGLE_QUE_FLAG++;
    if (ANGLE_QUE_FLAG >= 200)
    {
        float X_Curangle = Servo_X.queryRawAngle();
        delay(100);
        if (X_Curangle > 120)
        {
            Servo_X.wheelStop();
            // Servo_X.setTorque(false);
            delay(10);
            DEBUG_SERIAL.println("warnning,x-axis is out of max bounds." + String(X_Curangle, 1));
            delay(10);

            Servo_X.setAngle(110, 150);
            // Servo_X.wait();
            delay(200);
            Servo_X.setTorque(false);
            delay(10);
            DEBUG_SERIAL.println("out max bounds.");
            X_Curangle = 0;
        }
        else if ((X_Curangle < -120))
        {
            Servo_X.wheelStop();
            // Servo_X.setTorque(false);
            delay(10);
            DEBUG_SERIAL.println("warnning,x-axis is out of min bounds." + String(X_Curangle, 1));
            delay(10);
            Servo_X.setAngle(-110, 150);
            // Servo_X.wait();
            delay(200);
            Servo_X.setTorque(false);
            delay(10);
            // Servo_X.wait();
            DEBUG_SERIAL.println("out min bounds.");
            X_Curangle = 0;
        }
    }
}

void CMD_CRL(void)
{

    bool CMD_FLAG = 0;

    // 报告指令内容
    if (CMD_Head)
    {
        DEBUG_SERIAL.print("CMD:");
        DEBUG_SERIAL.println((char *)CMD_Head);
    }
    if (CMD_Param)
    {
        DEBUG_SERIAL.print("Param:");
        DEBUG_SERIAL.println(CMD_Param);
    }

    // DEBUG_SERIAL.println((char *)CMD_Head);
    // DEBUG_SERIAL.println(CMD_Param);

    if (!strcmp("backup", (char *)CMD_Head)) // 返回初始位置
    {
        Servo_Y.setRawAngle(RAW_ANGLE_Y, 1500);
        delay(10);
        Servo_X.setRawAngle(RAW_ANGLE_X, 1500);
        delay(1500);
    }
    else if (!strcmp("charge", (char *)CMD_Head)) // 充电开始
    {
        if (!(CLEAR_CHECK_BOOL || DISCHARGE_CHECK_BOOL)) // 充电安全检查，检查是否放电中
        {
            CHARGE_CHECK_BOOL = 1;
            DEBUG_SERIAL.println("Charging now...");
            digitalWrite(CHARGE_PORT, HIGH); // 安全检查通过，开始充电
        }
        else // 安全检查错误，配置放电引脚关闭
        {
            DEBUG_SERIAL.println("ERROR! NOW IS DISCHARGING!");
            // digitalWrite(DISCHARGE_PORT, LOW);
            // CHANGE_CHECK_BOOL = 1;
        }
    }
    else if (!strcmp("chargeover", (char *)CMD_Head)) // 充电完毕
    {
        DEBUG_SERIAL.println("Charged OK.");
        digitalWrite(CHARGE_PORT, LOW); // 配置关闭充电引脚
        CHARGE_CHECK_BOOL = 0;          // 标记允许放电
    }
    else if (!strcmp("discharge", (char *)CMD_Head)) // 放电开始
    {
        if (!(CLEAR_CHECK_BOOL || CHARGE_CHECK_BOOL)) // 充电安全检查，检查是否放电中
        {
            DISCHARGE_CHECK_BOOL = 1;
            DEBUG_SERIAL.println("Discharging now...");
            digitalWrite(DISCHARGE_PORT, HIGH); // 开始放电
            delay(DISCHARGE_SAFE_TIME);         // 至少放电时间 确保电路安全
        }
        else
        {
            DEBUG_SERIAL.println("ERROR! NOW IS CHARGING!");
            // digitalWrite(CHARGE_PORT, LOW); // 充电安全错误，配置充电引脚关闭
            // CHANGE_CHECK_BOOL = 0;
        }
    }
    else if (!strcmp("dischargeover", (char *)CMD_Head)) // 放电完毕
    {
        DEBUG_SERIAL.println("Discharge OK.");
        digitalWrite(DISCHARGE_PORT, LOW);
        DISCHARGE_CHECK_BOOL = 0;
    }
    else if (!strcmp("clear", (char *)CMD_Head)) // 放电完毕
    {
        if (!(DISCHARGE_CHECK_BOOL || CHARGE_CHECK_BOOL))
        {
            CLEAR_CHECK_BOOL = 1;
            DEBUG_SERIAL.println("Clearing remning power...");
            digitalWrite(CLEAR_PORT, HIGH);
        }
        else
        {
            DEBUG_SERIAL.println("Error,CHARGING OR DISCHARGING!");
        }
    }
    else if (!strcmp("clearover", (char *)CMD_Head)) // 放电完毕
    {

        CLEAR_CHECK_BOOL = 0;
        DEBUG_SERIAL.println("Cleared Success.");
        digitalWrite(CLEAR_PORT, LOW);
    }

    else if (!strcmp("setx", (char *)CMD_Head)) // 设置x轴角度
    {
        DEBUG_SERIAL.print("Set servo X angle to:");
        Servo_X.setAngle(CMD_Param + RAW_ANGLE_X);
        DEBUG_SERIAL.println(CMD_Param);
    }
    else if (!strcmp("sety", (char *)CMD_Head)) // 设置y轴角度
    {
        DEBUG_SERIAL.print("Set servo Y angle to:");
        Servo_Y.setAngle(CMD_Param * (-1) + RAW_ANGLE_Y); // 额。。。不想反舵机了。。。
        DEBUG_SERIAL.println(CMD_Param);
    }
    else if (!strcmp("turnleft", (char *)CMD_Head)) // 左转
    {
        // Servo_X.setTorque(true);
        // delay(20);
        Servo_X.wheelRunNCircle(0, 1); // 左转
        ANGLE_QUE_FLAG = 0;
        // Servo_Y.queryRawAngle();
        // Servo_X.setRawAngleByVelocity(120, 80, 100, 100, 0);
        // delay(2000);
        // Servo_X.setAngleRange(-120, 120);
    }
    else if (!strcmp("turnright", (char *)CMD_Head)) // 右转
    {
        // Servo_X.setRawAngleByVelocity(-120, 80, 100, 100 , 0);
        // delay(2000);
        // Servo_X.setTorque(true);
        // delay(20);
        Servo_X.wheelRunNCircle(1, 1); // 右转
        ANGLE_QUE_FLAG = 0;
        // Servo_X.setAngleRange(-120, 120);
    }
    else if (!strcmp("xstop", (char *)CMD_Head)) // x停止
    {
        Servo_X.wheelStop(); // 停止
        // Servo_X.setTorque(false);
        // delay(10);
        // Servo_X.setDamping(1000);
        delay(10);
        Servo_X.setTorque(true);
        delay(10);
    }

    else if (!strcmp("search", (char *)CMD_Head))
    {
        DEBUG_SERIAL.println("Auto Searching...");
        Servo_X.setSpeed(10);
        while (!(SerialBT.available() || DEBUG_SERIAL.available()))
        {

            delay(10);
            Servo_X.wheelRun(0);
            // Servo_X.setRawAngleByVelocity(-100, 20, 100, 100, 300);
            // DEBUG_SERIAL.println(Servo_X.isStop());
            DEBUG_SERIAL.println("Searching...");
            while (Servo_X.queryAngle() > -100)
            {
                if (SerialBT.available() || DEBUG_SERIAL.available())
                {
                    // BluTooTh_communication();
                    CMD_FLAG = true;
                    break;
                }
                delay(10);
            }
            Servo_X.wheelStop();
            delay(80);
            if (CMD_FLAG)
            {
                break;
            }
            // Servo_X.setRawAngle(RAW_ANGLE_X, 1500);
            Servo_X.wheelRun(1);
            while (Servo_X.queryAngle() < 100)
            {
                if (SerialBT.available() || DEBUG_SERIAL.available())
                {
                    // BluTooTh_communication();
                    CMD_FLAG = true;
                    break;
                }
                delay(10);
            }
            Servo_X.wheelStop();
            delay(80);
            if (CMD_FLAG)
            {
                break;
            }
            // DEBUG_SERIAL.println("Back Searching...");
            // Servo_X.wheelRun(1);
            // // Servo_X.setRawAngleByVelocity(0, 20, 0, 0, 300);
            // while (Servo_X.queryAngle() <= 100)
            // {
            //     if (SerialBT.available() || DEBUG_SERIAL.available())
            //     {
            //         CMD_FLAG = true;
            //         break;
            //     }
            //     delay(10);
            // }
            // Servo_X.wheelStop();
            // if (CMD_FLAG)
            // {
            //     break;
            // }
            // DEBUG_SERIAL.println("Still Searching...");
            //  Servo_X.setRawAngleByVelocity(100, 20, 0, 0, 300);
            // Servo_X.wheelRun(1);
            //  while (!Servo_X.isStop())
            //  {
            //      if (SerialBT.available() || DEBUG_SERIAL.available())
            //      {
            //          CMD_FLAG = true;
            //          break;
            //      }
            //      delay(10);
            //  }
            //  if (CMD_FLAG)
            //  {
            //      break;
            //  }
            // Servo_X.setRawAngleByVelocity(0, 20, 0, 0, 300);
            // while (!Servo_X.isStop())
            // {
            //     if (SerialBT.available() || DEBUG_SERIAL.available())
            //     {
            //         CMD_FLAG = true;
            //         break;
            //     }
            //     delay(10);
            // }
        }
        CMD_FLAG = false;
    }

    else if (!strcmp("turnup", (char *)CMD_Head)) // 抬头
    {
        Servo_Y.wheelRun(0); // 抬头
        ANGLE_QUE_FLAG = 0;
    }
    else if (!strcmp("turndown", (char *)CMD_Head)) // 低头
    {
        Servo_Y.wheelRun(1); // 低头
        ANGLE_QUE_FLAG = 0;
    }
    else if (!strcmp("ystop", (char *)CMD_Head)) // y低头
    {
        Servo_Y.wheelStop(); // y停止
        delay(10);
        Servo_Y.setTorque(true);
        delay(10);
    }

    else if (!strcmp("allpowerdown", (char *)CMD_Head)) // 全部释力
    {
        Servo_Y.setTorque(false);
        // Servo_Y.wait();
        delay(30);
        Servo_X.setTorque(false);
        delay(30);
        // Servo_X.wait();
    }
    else if (!strcmp("ypowerdown", (char *)CMD_Head)) // Y轴释力
    {
        Servo_Y.setTorque(false);
    }
    else if (!strcmp("xpowerdown", (char *)CMD_Head)) // X轴释力
    {
        Servo_X.setTorque(false);
    }
    else if (!strcmp("xping", (char *)CMD_Head)) // X轴PING
    {
        bool isOnline = Servo_X.ping();                                     // 舵机通讯检测
        String message = "servo #" + String(Servo_X.servoId, DEC) + " is "; // 日志输出
        if (isOnline)
        {
            message += "online";
        }
        else
        {
            message += "offline";
        }
        // 调试串口初始化
        DEBUG_SERIAL.println(message);
        // 等待1s
        delay(1000);
    }
    else if (!strcmp("yping", (char *)CMD_Head)) // Y轴PING
    {
        bool isOnline = Servo_Y.ping();                                     // 舵机通讯检测
        String message = "servo #" + String(Servo_Y.servoId, DEC) + " is "; // 日志输出
        if (isOnline)
        {
            message += "online";
        }
        else
        {
            message += "offline";
        }
        // 调试串口初始化
        DEBUG_SERIAL.println(message);
        // 等待1s
        delay(1000);
    }

    else if (!strcmp("auto", (char *)CMD_Head)) // Y轴PING
    {
        DEBUG_SERIAL.println("**************AUTO-LAUNCH***************");
        DEBUG_SERIAL.println("Prepping...");
        digitalWrite(CHARGE_PORT, LOW); // 关闭充电口
        delay(50);
        digitalWrite(DISCHARGE_PORT, LOW); // 关闭放电口
        delay(50);
        digitalWrite(CLEAR_PORT, HIGH); // 清除余电
        DEBUG_SERIAL.println("Clearing surplus power.");
        delay(1000);
        digitalWrite(CLEAR_PORT, LOW); // 清除完毕
        delay(300);
        DEBUG_SERIAL.println("OK,Charging now...");
        digitalWrite(CHARGE_PORT, HIGH); // 安全检查通过，开始充电
        delay(10000);
        digitalWrite(CHARGE_PORT, LOW); // 安全检查通过，开始充电
        DEBUG_SERIAL.println("Charge over.");
        delay(300);
        DEBUG_SERIAL.println("Ready to launch.");
        delay(1000);
        // DEBUG_SERIAL.println("3");
        // delay(1000);
        // DEBUG_SERIAL.println("2");
        // delay(1000);
        // DEBUG_SERIAL.println("1");
        // delay(1000);
        DEBUG_SERIAL.println("Launched.");
        digitalWrite(DISCHARGE_PORT, HIGH); // 安全检查通过，开始充电
        delay(3000);
        digitalWrite(DISCHARGE_PORT, LOW); // 安全检查通过，开始充电
        delay(300);
        DEBUG_SERIAL.println("All right,Clearing now.");
        digitalWrite(CLEAR_PORT, HIGH); // 清除余电
        delay(3000);
        digitalWrite(CLEAR_PORT, LOW); // 清除完毕
        DEBUG_SERIAL.println("Done.");
        delay(300);
    }

    else if (!strcmp("anglex", (char *)CMD_Head)) // X轴汇报角度
    {
        // Servo_X.setTorque(false);
        Servo_X.queryRawAngle();
        // 日志输出
        String message = "Status Code: " + String(Servo_X.protocol->responsePack.recv_status, DEC) + " servo #" + String(Servo_X.servoId, DEC) + " , Current Angle = " + String(Servo_X.curRawAngle, 1) + " deg";
        DEBUG_SERIAL.println(message);
        // 等待1s
        delay(100);
    }
    else if (!strcmp("angley", (char *)CMD_Head)) // Y轴汇报角度
    {
        // Servo_Y.setTorque(false);
        Servo_Y.queryRawAngle();
        // 日志输出
        String message = "Status Code: " + String(Servo_Y.protocol->responsePack.recv_status, DEC) + " servo #" + String(Servo_Y.servoId, DEC) + " , Current Angle = " + String(Servo_Y.curRawAngle, 1) + " deg";
        DEBUG_SERIAL.println(message);
        // 等待1s
        delay(100);
    }
    else if (!strcmp("angle", (char *)CMD_Head)) // 汇报角度
    {
        // Servo_Y.setTorque(false);
        // Servo_X.setTorque(false);
        Servo_Y.queryRawAngle();
        Servo_X.queryRawAngle();
        // 日志输出
        String message = "#" + String(Servo_Y.servoId, DEC) + ",Angle=" + String(Servo_Y.curRawAngle, 1) + "#" + String(Servo_X.servoId, DEC) + ",Angle=" + String(Servo_X.curRawAngle, 1);
        ;
        DEBUG_SERIAL.println(message);
        // 等待1s
        delay(100);
    }
    else if (!strcmp("askdis", (char *)CMD_Head)) // 汇报角度
    {
        COM.println("@GET_DIS\r");
        DEBUG_SERIAL.println("Had send @GET_DIS\r");
    }
    else if (!strcmp("thedis", (char *)CMD_Head)) // 汇报角度
    {
        DEBUG_SERIAL.println("Get distance:");
        DEBUG_SERIAL.println(CMD_Param);
    }
    else if (!strcmp("time", (char *)CMD_Head)) // 软复位
    {
        DEBUG_SERIAL.print("Uptime:");
        DEBUG_SERIAL.println(esp_timer_get_time());
    }
    else if (!strcmp("hi", (char *)CMD_Head)) // 握手
    {
        DEBUG_SERIAL.println("Hi!");
    }
    else if (!strcmp("restart", (char *)CMD_Head)) // 软复位
    {
        Servo_Y.setTorque(false);
        Servo_X.setTorque(false);
        esp_restart();
    }
    else
    {
        DEBUG_SERIAL.println("ERROR,Unknow Command!");
    }
}