# QDrive 使用手册

## FOC类接口

- CtrlType: 控制类型
- speed(): 获取电机转速,单位rpm
- angle(): 获取电机角度,单位rad
- current(): 获取Q轴电流,单位A
- voltage(): 获取母线电压,单位V


- calibrate(): FOC基础校准,包括:校准偏置电流,测量相电阻,校准编码器方向,校准电角度
- anticogging_calibrate(): 校准反齿轮效应


- init(): 初始化FOC类
- enable(): 使能FOC控制
- disable(): 失能FOC控制
- start(): 启动FOC控制
- stop(): 停止FOC控制

- Ctrl(): 外部控制调用接口
- Ctrl_ISR(): FOC(位置环,速度环)控制中断服务函数
- loopCtrl(): FOC(电流环)控制中断服务函数
- updateVoltage(): 更新母线电压

- pole_pairs: 电机极对数
- CtrlFrequency: 控制频率,单位Hz
- CurrentCtrlFrequency: 电流环控制频率,单位Hz

- initialized: FOC类是否已初始化
- enabled: FOC控制是否已使能
- started: FOC控制是否已启动
- calibrated: FOC类是否已校准
- anticogging_enabled: FOC反齿轮效应校准是否已启用
- anticogging_calibrated: FOC反齿轮效应校准是否已完成

## QDrive 启动流程概述

1. 构造FOC类对象
    - 配置电机参数: pole_pairs
    - 配置控制频率: CtrlFrequency, CurrentCtrlFrequency
    - 配置滤波器: CurrentFilter,SpeedFilter
    - 配置硬件接口: BLDC驱动,编码器,储存器,电流传感器
    - 配置PID参数: CurrentPID, SpeedPID, PositionPID
2. 调用init()函数进行初始化
    - 完成init()初始化后,FOC其他函数才能正常使用
3. 调用enable()函数使能FOC控制
   - 完成enable()使能后,FOC控制器开始工作,驱动启动,电机状态开始更新(如果已校准)
4. 调用calibrate()函数进行FOC基础校准(若未校准)
    - 完成calibrate()校准后,FOC控制器可以正常工作,电机状态开始更新
5. 调用anticogging_calibrate()函数进行FOC基础校准(若未校准)
    - 完成anticogging_calibrate()校准后,可以启用齿槽转矩补偿功能
    - 需要先完成calibrate()基础校准
6. 调用start()函数启动FOC控制

### 依赖关系

- init(): 完成类构造
- enable(): 完成init()初始化
- start(): 完成enable()使能, 完成calibrate()校准
- calibrate(): 完成enable()使能, 未调用start()启动
- anticogging_calibrate(): 完成enable()使能, 未调用start()启动, 完成calibrate()校准
- 更新电机状态(电流,转速,角度): 完成enable()使能, 完成calibrate()校准