#include <ros/ros.h>
#include "controlcan.h"
#include "canbus_driver.h"
#include <atomic>
#include <vector>
#include <algorithm>


CanbusDriver driver;

// uint64_t send_cnt;
// uint64_t receive_cnt;

//feedback函数
void process_can_data(const VCI_CAN_OBJ* data, int len, int ch) 
{
    static auto last_print_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    
    // receive_cnt += len;
    //检查是否超过 500 毫秒
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time).count() >= 500) 
    {
        last_print_time = now;

        ROS_INFO("CH%d: %d frames", ch, len);

        // ROS_INFO("receive_cnt: %d send_cnt :%d", receive_cnt, send_cnt);
        for (int i = 0; i < len; ++i) {
            std::ostringstream ss;
            ss << std::hex << std::uppercase;

            ss << "[ID 0x" << data[i].ID << "] "
            << "Len " << std::dec << (int)data[i].DataLen << " : ";

            ss << std::hex << std::setfill('0');
            for (int k = 0; k < data[i].DataLen; ++k) {
                ss << std::setw(2) << (int)data[i].Data[k] << ' ';
            }

            ROS_INFO("%s", ss.str().c_str());
        }
    }

}




int main(int argc, char** argv)
{   
    ros::init(argc, argv, "can_ctrl");
    ros::NodeHandle nh;

    driver.set_callback(process_can_data);
    //初始化驱动
    if (!driver.init()) {
        ROS_ERROR("CAN Driver Init failed!");
        return -1;
    }
    ros::Rate loop_rate(500); 
    while (ros::ok())
    {
        uint8_t send_buf[8] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
        driver.send_can_data(1,send_buf,8);
        // send_cnt++;
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
