#include "canbus_driver.h"
#include <cstring>
#include <chrono>

CanbusDriver::CanbusDriver() : run_flag_(false) {}

CanbusDriver::~CanbusDriver() {
    stop();
}

void CanbusDriver::set_callback(CanCallback cb) {
    rx_cb_ = cb;
}

bool CanbusDriver::init() {
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != 1) {
        printf("Error: Open CAN device failed!\n");
        return false;
    }

    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;
    config.Timing0 = 0x00; // 1Mbaud 波特率
    config.Timing1 = 0x14;
    config.Mode = 0;

    for (int ch = 0; ch < 2; ch++) {
        if (VCI_InitCAN(VCI_USBCAN2, 0, ch, &config) != 1 ||
            VCI_StartCAN(VCI_USBCAN2, 0, ch) != 1) {
            printf("Error: Init/Start CAN%d failed!\n", ch + 1);
            return false;
        }
    }

    run_flag_ = true;
    recv_thread_ = std::thread(&CanbusDriver::receive_loop, this);
    printf("CAN driver started.\n");
    return true;
}

void CanbusDriver::stop() {
    if (run_flag_) {
        run_flag_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        VCI_ResetCAN(VCI_USBCAN2, 0, 0);
        VCI_ResetCAN(VCI_USBCAN2, 0, 1);
        VCI_CloseDevice(VCI_USBCAN2, 0);
        printf("CAN driver closed.\n");
    }
}

void CanbusDriver::send_can_data(uint8_t id, uint8_t *data,uint8_t len) {
    if (!run_flag_) return;
    std::lock_guard<std::mutex> lk(can_mutex_);
    VCI_CAN_OBJ send;
    memset(&send, 0, sizeof(send));
    send.ID = id;
    send.DataLen = len;
    memcpy(send.Data,data,len);
    VCI_Transmit(VCI_USBCAN2, 0, 0, &send, 1);
}


// void CanbusDriver::get_can_data(uint8_t id, uint8_t *data,uint8_t len) {

// }



void CanbusDriver::receive_loop() {
    VCI_CAN_OBJ rec[300];

    while (run_flag_) {
        for (int ch = 0; ch < 2; ch++) {
            int reclen = 0;
            {
                reclen = VCI_Receive(VCI_USBCAN2, 0, ch, rec, 300, 100); 
            }

            if (reclen > 0) {
                // 如果注册了回调，就将数据传给 ROS 节点
                if (rx_cb_) {
                    rx_cb_(rec, reclen, ch);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}