#ifndef DR_EMPOWER_CAN_H
#define DR_EMPOWER_CAN_H

#include <string>
#include <vector>
#include <cstdint>
#include "serial/serial.h"  // 假设使用serial库

class MotorController {
public:
    // 构造函数，初始化串口
    MotorController(const std::string& port_name, int baudrate);

    // 设置电机角度、速度、模式等参数
    bool setAngle(int id, float angle, float speed, float param, int mode);

    // 获取电机状态信息
    bool replyState(int id_num);

private:
    // 数据解码函数：将字节数据转换为浮点和 int16_t
    std::vector<float> decodeData(const std::vector<uint8_t>& data, const std::string& format);

    std::vector<uint8_t> formatData(const std::vector<float>& data, const std::string& format);

    // 从串口接收数据并转换为 CAN 数据格式
    std::vector<uint8_t> receiveData(bool return_id);

    // 将 UART 数据转换为 CAN 数据格式
    std::vector<uint8_t> uartToCan(const std::vector<uint8_t>& data);

    // 将数据转换为 UART 格式的 CAN 数据帧并发送
    std::vector<uint8_t> canToUart(const std::vector<uint8_t>& data, int rtr);

    // 发送命令，将数据发送到串口
    void sendCommand(int id_num, uint8_t cmd, const std::vector<uint8_t>& data);

    // 串口写数据
    bool writeData(const std::vector<uint8_t>& data);

    // 串口错误处理
    void handleUartError();

    // 从串口读取数据
    // std::vector<uint8_t> readData(size_t num_bytes);
    std::vector<uint8_t> readData(size_t num);

private:
    std::string port;                 // 串口端口名
    int uart_baudrate;                // 波特率
    serial::Serial uart;              // 串口对象
    bool canfd_mode;                  // CAN FD 模式标志
    int read_flag;                    // 读取标志
    int reply_state_error = 0;        // 状态回复错误计数
    static const int MOTOR_NUM = 6;   // 假设的电机数量
    std::vector<std::vector<float>> motor_state;  // 电机状态数据
    bool enable_reply_state = true;   // 是否启用状态回复
    int READ_FLAG = 0;                // 全局读取标志
};

#endif // DR_EMPOWER_CAN_H
