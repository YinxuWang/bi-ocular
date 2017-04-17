#binocular_driver
###安装环境配置,安装opencv,libusb
```
sudo apt-get install build-essential #install gcc, g++
sudo apt-get install libboost-dev
sudo apt-get install cmake # 安装cmake
sudo apt-get install libusb-1.0-0-dev #libusb支持
sudo apt-get install libopencv-dev #opencv的支持
```

###idVendor和idProduc
插拔双目相机,在终端输入
```
dmesg
```
找到Product为DUAL-CAM GLOBAL M9V024的usb设备对应的idVendor和idProduct
```
13472.009858] usb 3-2: new high-speed USB device number 24 using xhci_hcd
[13472.146367] usb 3-2: New USB device found, idVendor=04b4, idProduct=1005
[13472.146370] usb 3-2: New USB device strings: Mfr=1, Product=2, SerialNumber=0
[13472.146372] usb 3-2: Product: DUAL-CAM GLOBAL MT9V024 
[13472.146373] usb 3-2: Manufacturer: CypTest
```


###编译
```
mkdir build
cd build
cmake ..
make -j4
```
###权限设置
如果不设置rules,那么用普通用户启动应用程序的时候,会出现Pemission Denied.解决方法为在`/etc/udev/rules.d`目录中创建rules.例如
cypress.rules,内容如下:
```
SUBSYSTEM=="usb_device", ATTRS{idVendor}=="1234", MODE="0666"
```
重启即可.

###IMU模块
Maker binocular集成了mpu6050 IMU模块,加速度计的量程为2g,陀螺仪的量程为2000deg/s.

#ROS Driver
For a Ros version driver, please go to this url:http://git.oschina.net/JaneLee/maker_binocular_ros
