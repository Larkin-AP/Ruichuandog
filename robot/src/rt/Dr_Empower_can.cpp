#include "rt/Dr_Empower_can.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <cstdint>
#include <thread>
#include <chrono>

// 构造函数初始化
MotorController::MotorController(const std::string& port_name, int baudrate) 
    : port(port_name), uart_baudrate(baudrate), canfd_mode(false), read_flag(0) {
    try {
        uart.setPort(port);
        uart.setBaudrate(uart_baudrate);
        uart.open();
        if (!uart.isOpen()) {
            throw std::runtime_error("Failed to open UART port");
        }
    } catch (const std::exception& e) {
        std::cerr << "Error opening UART port: " << e.what() << std::endl;
    }
}


// 将数据转换为字节数据
std::vector<uint8_t> MotorController::formatData(const std::vector<float>& data, const std::string& format) {
    std::vector<uint8_t> rdata;
    size_t data_index = 0;

    for (char f : format) {
        if (data_index >= data.size()) break;

        if (f == 'f') {  // 处理 float 类型，小端序存储
            float value = data[data_index];
            uint8_t bytes[4];
            std::memcpy(bytes, &value, sizeof(float));  // 将 float 转为字节
            
            // 按小端序填充（Python 默认使用小端序）
            rdata.push_back(bytes[0]);
            rdata.push_back(bytes[1]);
            rdata.push_back(bytes[2]);
            rdata.push_back(bytes[3]);
            data_index++;
        } else if (f == 'h') {  // 处理 int16 类型，小端序存储
            int16_t val = static_cast<int16_t>(data[data_index]);
            rdata.push_back(val & 0xFF);         // 低字节
            rdata.push_back((val >> 8) & 0xFF);  // 高字节
            data_index++;
        }
    }

    // 确保字节长度达到 8
    while (rdata.size() < 8) {
        rdata.push_back(0x00);
    }
    // std::cout << "Data: ";
    // for (auto byte : rdata) {
    //     std::cout << static_cast<unsigned>(byte) << " "; // 按原始数据逐字节输出
    // }
    // std::cout << std::endl;

    return rdata;
}


// 设置角度
bool MotorController::setAngle(int id, float angle, float speed, float param, int mode) {
    float factor = 0.01f;
    std::vector<uint8_t> data;
    uint8_t cmd;

    switch (mode) {
        case 0: {  // 轨迹跟踪模式
            int16_t s16_speed = static_cast<int16_t>(std::abs(speed) / factor);
            int16_t s16_width = static_cast<int16_t>(std::min(param / factor, 300.0f));
            data = formatData({angle, static_cast<float>(s16_speed), static_cast<float>(s16_width)}, "fhh");
            cmd = 0x19;
            break;
        }
        case 1: {  // 梯形轨迹模式
            if (speed > 0 && param > 0) {
                int16_t s16_speed = static_cast<int16_t>(std::abs(speed) / factor);
                int16_t s16_accel = static_cast<int16_t>(param / factor);
                data = formatData({angle, static_cast<float>(s16_speed), static_cast<float>(s16_accel)}, "fhh");
                cmd = 0x1A;
            } else {
                std::cerr << "speed or accel <= 0" << std::endl;
                return false;
            }
            break;
        }
        case 2: {  // 前馈控制模式
            int16_t s16_speed_ff = static_cast<int16_t>(speed / factor);
            int16_t s16_torque_ff = static_cast<int16_t>(param / factor);
            data = formatData({angle, static_cast<float>(s16_speed_ff), static_cast<float>(s16_torque_ff)}, "fhh");
            cmd = 0x1B;
            break;
        }
        default:
            std::cerr << "---error in set_angle---: Invalid mode" << std::endl;
            return false;
    }

    // 调用发送命令函数
    sendCommand(id, cmd, data);
    // bool reply_res = replyState(id);
    // if (reply_res) {
    //         std::cout << "Successfully received motor state." << std::endl;
    //         // 在这里可以处理 `motor_state` 数据
    //         } else {
    //         std::cerr << "Failed to receive motor state." << std::endl;
    //         }

    return true;
}

// CAN 到 UART 的数据转换
std::vector<uint8_t> MotorController::canToUart(const std::vector<uint8_t>& data, int rtr) {
    std::vector<uint8_t> udata(16, 0);
    udata[0] = 0xAA;  // 帧头
    udata[1] = 0x00;
    udata[3] = 0x08;  // 固定设置为 0x08

    // 设置第 3 个字节（如 py 中：udata[2]）
    if (rtr == 1) {
        udata[2] = 0x01;
    } else if (canfd_mode) {
        udata[2] = 0x06;  // 当 canfd_mode 启用时设置为 0x06
    } else {
        udata[2] = 0x00;
    }

    // 将数据填入 CAN 数据段
    if (data.size() == 11 && data[0] == 0x08) {
        for (int i = 0; i < 10; i++) {
            udata[6 + i] = data[i + 1];
        }
    }

    // 打印调试信息
    std::cout << "Data: ";
    for (auto byte : udata) {
        std::cout << static_cast<unsigned>(byte) << " ";
    }
    std::cout << std::endl;

    return udata;
}


// 发送命令
void MotorController::sendCommand(int id_num, uint8_t cmd, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> cdata(11, 0);
    cdata[0] = 0x08;  // 数据帧头
    uint16_t id_list = (id_num << 5) + cmd;
    cdata[1] = (id_list >> 8) & 0xFF;
    cdata[2] = id_list & 0xFF;

    for (size_t i = 0; i < data.size() && i < 8; ++i) {
        cdata[3 + i] = data[i];
    }

    // 调用 canToUart 封装为 UART 数据
    writeData(canToUart(cdata, 0));
}

// 串口写数据
bool MotorController::writeData(const std::vector<uint8_t>& data) {
    try {
        if (uart.isOpen()) {
            uart.write(data.data(), data.size());
            std::cout << "Data written to UART: ";
            for (const auto& byte : data) {
                std::cout << std::hex << static_cast<int>(byte) << " ";
            }
            std::cout << std::endl;
            return true;
        } else {
            std::cerr << "UART port is not open." << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "---error in write_data---: " << e.what() << std::endl;
        handleUartError();
        return false;
    }
}

// 数据解码函数：将字节数据转换为 float 和 int16_t
std::vector<float> MotorController::decodeData(const std::vector<uint8_t>& data, const std::string& format) {
    std::vector<float> decodedData;
    size_t byteIndex = 0;

    for (char f : format) {
        if (f == 'f' && byteIndex + 4 <= data.size()) {
            uint32_t intValue = (data[byteIndex] << 24) | (data[byteIndex + 1] << 16) |
                                (data[byteIndex + 2] << 8) | data[byteIndex + 3];
            float floatValue;
            std::memcpy(&floatValue, &intValue, sizeof(floatValue));
            decodedData.push_back(floatValue);
            byteIndex += 4;
        } else if (f == 'h' && byteIndex + 2 <= data.size()) {
            int16_t intValue = (data[byteIndex] << 8) | data[byteIndex + 1];
            decodedData.push_back(static_cast<float>(intValue));
            byteIndex += 2;
        } else {
            throw std::runtime_error("Invalid format or insufficient data in decodeData");
        }
    }
    return decodedData;
}

bool MotorController::replyState(int id_num) {
    READ_FLAG = 0;
    try {
        if (enable_reply_state && id_num <= MOTOR_NUM) {
            READ_FLAG = 0;
            std::vector<uint8_t> rdata = receiveData(true);
            if (READ_FLAG == 1) {
                if (id_num == 0) {
                    id_num = ((((rdata[1] << 8) + rdata[2]) & 0x07E0) >> 5) & 0xFF;
                }
                float factor = 0.01f;
                std::vector<uint8_t> cdata(rdata.begin() + 3, rdata.end());
                std::vector<float> data = decodeData(cdata, "fhh");

                motor_state[id_num - 1][0] = data[0];
                motor_state[id_num - 1][1] = data[1] * factor;
                motor_state[id_num - 1][2] = data[2] * factor;
                motor_state[id_num - 1][3] = (rdata[2] & 0x02) >> 1;  // traj_done 标志
                motor_state[id_num - 1][4] = (rdata[2] & 0x04) >> 2;  // axis_error 标志
            } else {
                READ_FLAG = -1;
                reply_state_error++;
                return false;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "---error replyState---: " << e.what() << std::endl;
        reply_state_error++;
        return false;
    }
    return true;
}

std::vector<uint8_t> MotorController::receiveData(bool return_id) {
    std::vector<uint8_t> udata = readData(16);
    if (READ_FLAG == 1) {
        std::vector<uint8_t> cdata = uartToCan(udata);
        return return_id ? cdata : std::vector<uint8_t>(cdata.begin() + 3, cdata.end());
    }
    return {};
}

std::vector<uint8_t> MotorController::uartToCan(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> cdata(11, 0x00);
    if (data.size() == 16 && data[3] == 0x08) {
        for (size_t i = 0; i < 10; ++i) {
            cdata[1 + i] = data[i + 6];
        }
        return cdata;
    } else {
        READ_FLAG = -1;
        return {};
    }
}

// 串口接收函数：从 UART 中读取指定字节数
std::vector<uint8_t> MotorController::readData(size_t num) {
    READ_FLAG = -1;  // 初始化读取标志为 -1
    std::vector<uint8_t> byte_list;
    int retry_limit = 10000;  // 允许重试的最大次数

    // 循环等待串口缓冲区有数据可读取
    while (uart.available() == 0 && retry_limit > 0) {
        retry_limit--;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 等待一段时间以减少 CPU 占用
    }

    // 从串口读取数据，直到获取到所有字节
    while (uart.available() > 0 && byte_list.size() < num) {
        byte_list.push_back(uart.read(1)[0]);
    }

    // 检查是否成功接收到指定数量的数据
    if (byte_list.size() == num) {
        READ_FLAG = 1;  // 成功读取标志
    } else {
        std::cerr << "Received data error in readData(): ";
        for (const auto& byte : byte_list) {
            std::cerr << static_cast<int>(byte) << " ";
        }
        std::cerr << std::endl;
        READ_FLAG = -1;  // 读取错误标志
    }

    return byte_list;
}


// 串口错误处理
void MotorController::handleUartError() {
    std::cerr << "Attempting to restart UART connection..." << std::endl;
    uart.close();
    try {
        uart.open();
        if (!uart.isOpen()) {
            throw std::runtime_error("Failed to reopen UART port");
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to restart UART connection: " << e.what() << std::endl;
    }
}
