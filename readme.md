# QDrive - 高性能轻量化FOC实例

此为软件部分，硬件部分参见：[QDrive-硬件部分](https://github.com/Liu-Curiousity/QDrive-Hardware)

您可以复刻此项目的硬件部分，或购买QDrive成品驱动板：[QDrive成品驱动板](https://e.tb.cn/h.hvxbdqyMjSPHMGw?tk=HijG4lsnjo0)。

![ "PCB"](./Doc/Images/PCB.jpg)

[//]: # (![]&#40;./Doc/Images/PCB渲染图.png "PCB渲染图"&#41;)

[//]: # ()

[//]: # (![]&#40;./Doc/Images/电机渲染图.png "电机渲染图"&#41;)

## 简介

QDrive是一个基于STM32G431系列MCU的高性能FOC控制器，旨在提供一个轻量级、易于使用的FOC解决方案。支持多种电机类型，并且具有高效的电流控制和速度控制功能。

此工程仅适用于4310电机QDrive驱动板，如需使用其他电机，可以自行移植。

## 特性

- **高性能**：基于STM32G431系列MCU，提供强大的处理能力。
- **高精度**：采用18位编码器，可以实现0.005°的精细角度控制。
- **ms级响应**：经测试，可在40ms内使电机转动半圈。
- **丰富接口**：支持CAN总线通信、USB串口通信、支持shell交互。

## 使用方法

- [**对于开发者**](./Doc/for_developer.md)
- [**对于使用者**](./Doc/for_user.md)
