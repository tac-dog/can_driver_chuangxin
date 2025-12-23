### 创芯科技can driver

适用于 **x86 架构系统** 的创芯科技 CAN 设备驱动

#### Build

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/tac-dog/can_driver_chuangxin.git
catkin build
```

#### 添加USB权限

```
sudo vi /etc/udev/rules.d/99-myusb.rules

添加以下内容：
##
ACTION=="add",SUBSYSTEMS=="usb", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0053",
GROUP="users", MODE="0777"

保存后重新插拔can盒
```

#### Run

```
roslaunch can_pkg canbus.launch
```

#### Notes

```
可以进入环回模式（自发自收模式）测试，即config.mode = 2;
若系统架构不是x86如 ARM、Raspberry Pi、Jetson等），自行去官网替换对应的libcontrolcan.so文件
官网资料链接:https://www.zhcxgd.com/2.html
```

