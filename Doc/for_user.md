## 如何获得

---

1. **下载程序固件**：进入[Release](https://github.com/Liu-Curiousity/QDrive-Software/releases)页面，下载最新的固件文件(
   .bin、.hex或.elf文件)。
2. **准备驱动板**：从此工程准备QDrive驱动板，并组装完成。
3. **烧录固件**：使用STLink/JLink/DAPLink等编程器，将下载的固件烧录到QDrive驱动板上。可以使用STM32CubeProgrammer或其他支持的编程工具。

    - 烧录接口(hc0.8-4pin)位置如下图所示，注意线序(V:3.3V G:GND C:SWCLK D:SWDIO)：

   ![](.\Doc\Images\SWD接口.png "SWD接口")

## 如何使用

适用于4310电机的QDrive具有以下两个接口：CAN总线、Type-C接口的USB串口。

---

### 首次上电

1. **连接电源**：在XT30(2+2)接口上连接8~26V电源为QDrive驱动板供电。
2. **连接电脑**：使用Type-C数据线将QDrive驱动板连接到计算机。
3. **打开串口工具**：QDrive的USB串口集成一个轻量化shell，使用串口工具(如PuTTY、MobaXTerm等)打开串口终端，波特率任意。
4. **新机校准**：首次使用时，需进行新机校准。输入`calibrate`命令，按提示操作完成校准。

### 后续使用

1. 可使用USB串口控制QDrive，串口输入`help`命令查看可用命令列表。
2. 可使用CAN总线控制QDrive，使用CAN总线发送相应的指令包：[QDrive CAN通信协议](../Applications/Src/CommunicationProtocol.md)。