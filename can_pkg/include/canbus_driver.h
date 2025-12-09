#ifndef CANBUS_DRIVER_H
#define CANBUS_DRIVER_H

#include "controlcan.h"
#include <mutex>
#include <atomic>
#include <thread>
#include <cstdint>
#include <cstdio>
#include <functional>

// 定义回调函数类型：参数是 (数据指针, 数据长度, 通道号)
using CanCallback = std::function<void(const VCI_CAN_OBJ*, int, int)>;

class CanbusDriver {
public:
    CanbusDriver();
    ~CanbusDriver();

    bool init();
    void stop();

    // 注册接收回调函数
    void set_callback(CanCallback cb);

    void send_can_data(uint8_t id, uint8_t *data,uint8_t len);

    // void get_can_data(uint8_t id, uint8_t *data,uint8_t len);

private:
    void receive_loop();

    std::mutex can_mutex_;
    std::atomic<bool> run_flag_;
    std::thread recv_thread_;
    CanCallback rx_cb_;
};

#endif // CANBUS_DRIVER_H