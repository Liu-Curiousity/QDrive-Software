/**
 * @brief   FOC驱动库
 * @details
 * @author  LiuHaoqi
 * @date    2024-7-10
 * @version V3.0.0
 * @note    此库为中间层库,与硬件完全解耦
 * @warning 无
 * @par     历史版本:
 *		    V1.0.0创建于2024-7-3
 *		    v2.0.0修改于2024-7-10,添加d轴电流PID控制
 *		    V3.0.0修改于2025-4-12,中间漏了好多版本
 *		    V4.0.0修改于2025-5-4,添加CurrentSensor类,后将续从current_sensor中获取电流
 *		    V4.1.0修改于2025-5-5,重命名PolePairs为pole_pairs,添加电流偏置校准和相电阻测量,并在电角度校准时自动调整硬拖电压
 *		    V4.1.1修改于2025-5-6,校准相电流偏置前等待30ms,修复测量相电阻时忘记应用电流偏置校准导致相电阻测量误差的问题
 *		    V4.1.2修改于2025-5-6,优化操作逻辑,开始校准前清除已校准标志
 *		    V4.1.3修改于2025-5-6,更改校准函数名
 *		    V5.0.0修改于2025-6-26,调整initialize,enable,start三层实现逻辑细节
 *		    V5.0.0调整SetPhaseVoltage()参数顺序
 *		    V5.1.0修改于2025-7-3,修复calibrate()函数致命问题,重新调整init,enable,start三层实现逻辑细节,为后续无感算法铺路,调整更新电压函数接口名称
 */


#include <algorithm>
#include <numbers>
#include <numeric>
#include "FOC.h"
#include "FOC_config.h"

using namespace std;

void FOC::init() {
    // 1.初始化BLDC驱动
    if (!bldc_driver.initialized)
        bldc_driver.init();
    // 2.初始化编码器
    if (!bldc_encoder.initialized)
        bldc_encoder.init();
    // 3.初始化电流传感器
    if (!current_sensor.initialized)
        current_sensor.init();
    // 4.初始化flash
    if (!storage.initialized)
        storage.init();
    // 5.从flash中读取校准数据
    load_storage_calibration();
    // 6.完成初始化
    initialized = true;
}

void FOC::load_storage_calibration() {
    StorageStatus storage_status;
    storage.read(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
    if ((storage_status & 0xC0) == STORAGE_BASE_CALIBRATE_OK) {
        // 如果基础校准数据正常,则读取
        storage.read(0x100, reinterpret_cast<uint8_t *>(&encoder_direction), sizeof(encoder_direction));
        storage.read(0x110, reinterpret_cast<uint8_t *>(&zero_electric_angle), sizeof(zero_electric_angle));
        storage.read(0x120, reinterpret_cast<uint8_t *>(&iu_offset), sizeof(iu_offset));
        storage.read(0x130, reinterpret_cast<uint8_t *>(&iv_offset), sizeof(iv_offset));
        storage.read(0x140, reinterpret_cast<uint8_t *>(&phase_resistance), sizeof(phase_resistance));
        storage.read(0x150, reinterpret_cast<uint8_t *>(&phase_inductance), sizeof(phase_inductance));
        calibrated = true;
    }
    if ((storage_status & 0x30) == STORAGE_ANTICOGGING_CALIBRATE_OK) {
        storage.read(0x800, reinterpret_cast<uint8_t *>(anticogging_map), sizeof(anticogging_map));
        anticogging_calibrated = true;
    }
    if ((storage_status & 0x0C) == STORAGE_PID_PARAMETER_OK) {
        storage.read(0x160, reinterpret_cast<uint8_t *>(&PID_Speed.kp), sizeof(PID_Speed.kp));
        storage.read(0x170, reinterpret_cast<uint8_t *>(&PID_Speed.ki), sizeof(PID_Speed.ki));
        storage.read(0x180, reinterpret_cast<uint8_t *>(&PID_Speed.kd), sizeof(PID_Speed.kd));
        storage.read(0x190, reinterpret_cast<uint8_t *>(&PID_Angle.kp), sizeof(PID_Angle.kp));
        storage.read(0x1A0, reinterpret_cast<uint8_t *>(&PID_Angle.ki), sizeof(PID_Angle.ki));
        storage.read(0x1B0, reinterpret_cast<uint8_t *>(&PID_Angle.kd), sizeof(PID_Angle.kd));
    }
    if ((storage_status & 0x03) == STORAGE_LIMIT_OK) {
        storage.read(0x1C0, reinterpret_cast<uint8_t *>(&PID_Angle.output_limit_p), sizeof(PID_Angle.output_limit_p));
        PID_Angle.output_limit_n = -PID_Angle.output_limit_p;
        storage.read(0x1D0, reinterpret_cast<uint8_t *>(&PID_Speed.output_limit_p), sizeof(PID_Speed.output_limit_p));
        PID_Speed.output_limit_n = -PID_Speed.output_limit_p;
    }
}

/**
 * @brief 储存校准数据
 * @param storage_type 储存数据类型
 */
void FOC::freeze_storage_calibration(const StorageStatus storage_type) {
    StorageStatus storage_status;
    storage.read(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
    switch (storage_type) {
        case STORAGE_BASE_CALIBRATE_OK:
            // 储存基础校准数据
            storage.write(0x100, reinterpret_cast<uint8_t *>(&encoder_direction), sizeof(encoder_direction));
            storage.write(0x110, reinterpret_cast<uint8_t *>(&zero_electric_angle), sizeof(zero_electric_angle));
            storage.write(0x120, reinterpret_cast<uint8_t *>(&iu_offset), sizeof(iu_offset));
            storage.write(0x130, reinterpret_cast<uint8_t *>(&iv_offset), sizeof(iv_offset));
            storage.write(0x140, reinterpret_cast<uint8_t *>(&phase_resistance), sizeof(phase_resistance));
            storage.write(0x150, reinterpret_cast<uint8_t *>(&phase_inductance), sizeof(phase_inductance));

            // 更新储存状态
            storage_status = static_cast<StorageStatus>((storage_status & 0x3F) | STORAGE_BASE_CALIBRATE_OK);
            storage.write(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
            break;
        case STORAGE_ANTICOGGING_CALIBRATE_OK:
            // 储存齿槽转矩补偿表
            storage.write(0x800, reinterpret_cast<uint8_t *>(anticogging_map), sizeof(anticogging_map));

            // 更新储存状态
            storage_status = static_cast<StorageStatus>((storage_status & 0xCF) | STORAGE_ANTICOGGING_CALIBRATE_OK);
            storage.write(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
            break;
        case STORAGE_PID_PARAMETER_OK:
            // 储存PID参数
            storage.write(0x160, reinterpret_cast<uint8_t *>(&PID_Speed.kp), sizeof(PID_Speed.kp));
            storage.write(0x170, reinterpret_cast<uint8_t *>(&PID_Speed.ki), sizeof(PID_Speed.ki));
            storage.write(0x180, reinterpret_cast<uint8_t *>(&PID_Speed.kd), sizeof(PID_Speed.kd));
            storage.write(0x190, reinterpret_cast<uint8_t *>(&PID_Angle.kp), sizeof(PID_Angle.kp));
            storage.write(0x1A0, reinterpret_cast<uint8_t *>(&PID_Angle.ki), sizeof(PID_Angle.ki));
            storage.write(0x1B0, reinterpret_cast<uint8_t *>(&PID_Angle.kd), sizeof(PID_Angle.kd));

            // 更新储存状态
            storage_status = static_cast<StorageStatus>((storage_status & 0xF3) | STORAGE_PID_PARAMETER_OK);
            storage.write(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
            break;
        case STORAGE_LIMIT_OK:
            storage.write(0x1C0, reinterpret_cast<uint8_t *>(&PID_Angle.output_limit_p),
                         sizeof(PID_Angle.output_limit_p));
            storage.write(0x1D0, reinterpret_cast<uint8_t *>(&PID_Speed.output_limit_p),
                         sizeof(PID_Speed.output_limit_p));

            // 更新储存状态
            storage_status = static_cast<StorageStatus>((storage_status & 0xFC) | STORAGE_LIMIT_OK);
            storage.write(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
            break;
        default: ;
    }
}

void FOC::enable() {
    if (!initialized) return; // 如果没有初始化,则不能使能
    //1.启动BLDC驱动
    if (!bldc_driver.enabled) {
        bldc_driver.enable();
        bldc_driver.set_duty(0, 0, 0);
    }
    //2.启动编码器
    if (!bldc_encoder.enabled)
        bldc_encoder.enable();
    // 3.启动电流传感器
    if (!current_sensor.enabled)
        current_sensor.enable();
    enabled = true;
}

void FOC::disable() {
    //1.关闭BLDC驱动
    if (bldc_driver.enabled) {
        bldc_driver.set_duty(0, 0, 0);
        bldc_driver.disable();
    }
    //2.关闭编码器
    if (bldc_driver.enabled)
        bldc_encoder.disable();
    enabled = false;
}

void FOC::start() {
    if (enabled && calibrated) started = true;
}

void FOC::stop() {
    started = false;
}

void FOC::calibrate() {
    if (!enabled) return; // 如果没有使能,则不能校准
    if (started) return;  // 如果已经启动,则不能校准
    calibrated = false;   // 标记为未校准

    /*1.校准电流*/
    SetPhaseVoltage(0, 0, 0); // 设置电压为0
    delay(30);
    iu_offset = iv_offset = 0;
    for (int i = 0; i < 200; ++i) {
        // 读取电流值,200次平均
        iu_offset += current_sensor.iu / 200;
        iv_offset += current_sensor.iv / 200;
        delay(1);
    }

    /*2.测量相电阻,确定后续校准使用电压*/
    bldc_driver.set_duty(0, 0, 0);
    float uu = 0;
    for (uu = 0; uu < 0.8f && current_sensor.iu - iu_offset < FOC_MAX_CURRENT * 0.9;) {
        uu += 0.001f;
        bldc_driver.set_duty(uu, 0, 0);
        delay(1);
    }
    phase_resistance = 0;
    for (int i = 0; i < 100; ++i) {
        phase_resistance += uu * Voltage / (current_sensor.iu - iu_offset) / 3 * 4 / 100;
        delay(1);
    }
    SetPhaseVoltage(0, 0, 0); // 设置电压为0
    const float voltage_align = clamp(uu * 4 / 3, -1.0f, 1.0f);

    /*3.校准编码器正方向,使其与q轴正方向相同*/
    SetPhaseVoltage(voltage_align * 0.6f, 0, 0);
    // 读取电机角度,100次平均
    delay(100);
    float begin_angle = 0;
    for (int i = 0; i < 100; ++i) {
        begin_angle += bldc_encoder.get_angle() / 100;
    }
    // 按q轴正方向硬拖2pi电角度
    for (int i = 0; i < 500; ++i) {
        const float angle = 2 * numbers::pi_v<float> * i / 500.0f;
        SetPhaseVoltage(voltage_align * 0.6f, 0, angle);
        delay(1);
    }
    // 读取电机角度,100次平均
    float end_angle = 0;
    for (int i = 0; i < 100; ++i) {
        end_angle += bldc_encoder.get_angle() / 100;
    }
    SetPhaseVoltage(0, 0, 0); // 停止电机
    if ((end_angle > begin_angle && end_angle < begin_angle + numbers::pi_v<float>) ||
        end_angle < begin_angle - numbers::pi_v<float>) {
        encoder_direction = true;
    } else {
        encoder_direction = false;
    }

    /*4.校准电角度零点*/
    float sum_offset_angle = 0;
    for (int i = 0; i < pole_pairs; ++i) {
        // 按q轴正方向硬拖2pi电角度
        for (int j = 0; j < 250; ++j) {
            const float angle = 2 * numbers::pi_v<float> * j / 250.0f;
            SetPhaseVoltage(voltage_align * 0.6f, 0, angle);
            delay(1);
        }
        SetPhaseVoltage(voltage_align, 0, 0);
        delay(300);
        for (int j = 0; j < 100; ++j) {
            if (encoder_direction)
                sum_offset_angle += (2 * numbers::pi_v<float> - bldc_encoder.get_angle()) / 100;
            else
                sum_offset_angle += bldc_encoder.get_angle() / 100;
            delay(1);
        }
    }
    SetPhaseVoltage(0, 0, 0);
    zero_electric_angle = (sum_offset_angle - numbers::pi_v<float> * (pole_pairs - 1)) / pole_pairs;

    /*5.校准完成*/
    freeze_storage_calibration(STORAGE_BASE_CALIBRATE_OK); // 保存基础校准数据
    calibrated = true;
}

void FOC::anticogging_calibrate() {
    if (!enabled) return;           // 如果没有使能,则不能校准
    if (!calibrated) return;        // 如果没有基础校准,则不能校准
    if (started) return;            // 如果已经启动,则不能校准
    anticogging_calibrated = false; // 标记为未校准

    Ctrl(CtrlType::CurrentCtrl, 0); // 释放电机
    anticogging_calibrating = true; // 开始校准,即开始闭环控制
    delay(5);
    // 读取电角度零点校准后的电机角度,并确定补偿表开始索引
    auto index = static_cast<uint16_t>(Angle * numbers::inv_pi_v<float> * 0.5f * map_len);
    Ctrl(CtrlType::AngleCtrl, numbers::pi_v<float> * 2 * index / map_len); // 先定位到前一个点,并延时做准备
    delay(20);
    float angle_ = 0, iq_ = 0;
    // 采集初期几个点不可信任,多采集30个点将其覆盖
    for (int i = 0; i < map_len + 30; ++i) {
        index = (index + 1) % map_len;
        Ctrl(CtrlType::AngleCtrl, numbers::pi_v<float> * 2 * index / map_len);
        float speed_ = 0.3;
        while (abs(angle_ - numbers::pi_v<float> * 2 * index / map_len) > numbers::pi_v<float> * 2 / map_len / 10 ||
               abs(speed_) > 0.08) {
            // 0度2pi度溢出补偿
            float tmp = Angle;
            if (numbers::pi_v<float> * 2 * index / map_len > 6.2 && tmp < 0.1) tmp += numbers::pi_v<float> * 2;
            if (numbers::pi_v<float> * 2 * index / map_len < 0.1 && tmp > 6.2) tmp -= numbers::pi_v<float> * 2;
            // 过个低通
            angle_ = angle_ * 0.8f + tmp * 0.2f;
            speed_ = speed_ * 0.97f + Speed * 0.03f;
            iq_ = iq_ * 0.80f + Iq * 0.20f;
            delay(1);
        }
        anticogging_map[index] = iq_;
    }
    Ctrl(CtrlType::CurrentCtrl, 0);  // 释放电机
    anticogging_calibrating = false; // 结束校准,即关闭闭环控制
    // 后处理:将补偿表平移到0点(应为正转校准时的Iq平均值大于0(需推动转子转动))
    const float anticogging_avg = accumulate(anticogging_map, anticogging_map + map_len, 0.0f) / map_len;
    for (auto& anticogging : anticogging_map) {
        anticogging -= anticogging_avg;
    }
    freeze_storage_calibration(STORAGE_ANTICOGGING_CALIBRATE_OK); // 保存基础校准数据
    anticogging_calibrated = true;
}

/**
 * @brief FOC电流变换
 * @param iu U相电流
 * @param iv V相电流
 * @param iw W相电流
 */
__attribute__((section(".ccmram_func")))
void FOC::UpdateCurrent(const float iu, const float iv, const float iw) {
    /**1.保存电流值**/
    Iu = iu;
    Iv = iv;
    Iw = iw;

    /**2.克拉克变换**/
    Ia = Iu;
    Ib = (Iu + 2 * Iv) * numbers::inv_sqrt3_v<float>;

    /**3.帕克变换**/
    const float cos_angle = cosf(ElectricalAngle);
    const float sin_angle = sinf(ElectricalAngle);
    Iq = CurrentQFilter(Ib * cos_angle - Ia * sin_angle);
    Id = CurrentDFilter(Ib * sin_angle + Ia * cos_angle);
}

/**
 * @brief FOC控制函数
 * @param ud 法向力矩,必须在0~1之间!
 * @param uq 切向力矩,必须在0~1之间!
 * @param electrical_angle 电机电角度,单位弧度
 */
__attribute__((section(".ccmram_func")))
void FOC::SetPhaseVoltage(float ud, float uq, const float electrical_angle) {
    // Uu,Uv,Uw不能设置到最大值1,为了防止电流采样时候MOS对电机有驱动,影响采样
    // 表现为某一相Ux=0时候(堵转测试),电流采样值偶尔出现尖峰,电机异常抽搐
    ud *= 0.99f;
    uq *= 0.99f;

    /**1.保存电压值**/
    Ud = ud;
    Uq = uq;

    /**2.帕克逆变换**/
    const float cos_angle = cosf(electrical_angle);
    const float sin_angle = sinf(electrical_angle);
    Ua = (-Uq * sin_angle + Ud * cos_angle) * 0.5f; // 除以2将Ua范围限制在[-0.5,0.5],使后续Uu,Uv,Uw范围在[0,1]
    Ub = (Uq * cos_angle + Ud * sin_angle) * 0.5f;  // 除以2将Ub范围限制在[-0.5,0.5],使后续Uu,Uv,Uw范围在[0,1]

    /**3.克拉克逆变换**/
    Uu = Ua + 0.5f; //加0.5使得Uu均值为0.5,在[0,1]之间变化
    Uv = -Ua * 0.5f + Ub * numbers::sqrt3_v<float> * 0.5f + 0.5f;

    // 原公式:
    // float Uw = -Ua * 0.5f - Ub * M_SQRT3_F * 0.5f + 0.5f;
    // 使用Uu,Uv计算得来,减少运算量(其实运算量大头在sin和cos):
    Uw = 1.5f - Uu - Uv;

    /**4.设置驱动器占空比**/
    bldc_driver.set_duty(Uu, Uv, Uw);
}

void FOC::updateVoltage(const float voltage) {
    if (voltage == 0) return;
    PID_CurrentQ.kp *= Voltage / voltage;
    PID_CurrentD.kp *= Voltage / voltage;
    PID_CurrentQ.ki *= Voltage / voltage;
    PID_CurrentD.ki *= Voltage / voltage;
    Voltage = voltage;
}

void FOC::setPID(const float pid_speed_kp, const float pid_speed_ki, const float pid_speed_kd,
                 const float pid_angle_kp, const float pid_angle_ki, const float pid_angle_kd) {
    if (!isnan(pid_speed_kp)) PID_Speed.kp = pid_speed_kp;
    if (!isnan(pid_speed_ki)) PID_Speed.ki = pid_speed_ki;
    if (!isnan(pid_speed_kd)) PID_Speed.kd = pid_speed_kd;
    if (!isnan(pid_angle_kp)) PID_Angle.kp = pid_angle_kp;
    if (!isnan(pid_angle_ki)) PID_Angle.ki = pid_angle_ki;
    if (!isnan(pid_angle_kd)) PID_Angle.kd = pid_angle_kd;
}

void FOC::setLimit(const float speed_limit, const float current_limit) {
    if (!isnan(speed_limit)) {
        PID_Angle.output_limit_p = speed_limit;
        PID_Angle.output_limit_n = -speed_limit;
    }
    if (!isnan(current_limit)) {
        PID_Speed.output_limit_p = current_limit;
        PID_Speed.output_limit_n = -current_limit;
    }
}

void FOC::storagePID() {
    freeze_storage_calibration(STORAGE_PID_PARAMETER_OK);
}

void FOC::Ctrl(const CtrlType ctrl_type, float value) {
    switch (ctrl_type) {
        case CtrlType::AngleCtrl:
            PID_Angle.SetTarget(value);
            break;
        case CtrlType::SpeedCtrl:
            value = clamp(value, PID_Angle.output_limit_n, PID_Angle.output_limit_p); // 限制最大速度
            PID_Speed.SetTarget(value);
            break;
        case CtrlType::CurrentCtrl:
            value = clamp(value, PID_Speed.output_limit_n, PID_Speed.output_limit_p); // 限制最大电流
            target_iq = value;
            break;
    }
    this->ctrl_type = ctrl_type;
}

__attribute__((section(".ccmram_func")))
void FOC::Ctrl_ISR() {
    if (!started && !anticogging_calibrating) return;

    /**1.速度闭环控制**/
    switch (ctrl_type) {
        case CtrlType::AngleCtrl:
            //使电机始终沿差值小于pi的方向转动
            if (Angle - PID_Angle.target > numbers::pi_v<float>)
                PID_Speed.SetTarget(PID_Angle.calc(Angle - 2 * numbers::pi_v<float>));
            else if (Angle - PID_Angle.target < -numbers::pi_v<float>)
                PID_Speed.SetTarget(PID_Angle.calc(Angle + 2 * numbers::pi_v<float>));
            else
                PID_Speed.SetTarget(PID_Angle.calc(Angle));
        case CtrlType::SpeedCtrl:
            target_iq = PID_Speed.calc(Speed);
        case CtrlType::CurrentCtrl:
            /*齿槽转矩补偿*/
            if (anticogging_enabled && anticogging_calibrated) {
                const uint16_t index =
                    static_cast<uint16_t>(Angle * map_len * 0.5f * numbers::inv_pi_v<float> + 0.5f) % map_len;
                PID_CurrentQ.SetTarget(target_iq + anticogging_map[index]);
            } else {
                PID_CurrentQ.SetTarget(target_iq);
            }
            break;
    }
}

/*CCMRAM加速运行*/
__attribute__((section(".ccmram_func")))
void FOC::loopCtrl() {
    static float temp;
    if (!enabled) return;    // 如果没有使能,则不能运行
    if (!calibrated) return; // 如果没有校准,则不能运行

    /**1.电流变换**/
    UpdateCurrent(current_sensor.iu - iu_offset,
                  current_sensor.iv - iv_offset,
                  current_sensor.iw + iu_offset + iv_offset);

    /**2.读取编码器角度**/
    if (encoder_direction)
        temp = bldc_encoder.get_angle() + zero_electric_angle;
    else
        temp = 2 * numbers::pi_v<float> - bldc_encoder.get_angle() + zero_electric_angle;
    Angle = temp > 2 * numbers::pi_v<float> ? temp - 2 * numbers::pi_v<float> :
            temp < 0 ? temp + 2 * numbers::pi_v<float> : temp;
    ElectricalAngle = Angle * pole_pairs;

    /**3.计算转速**/
    temp = Angle - PreviousAngle;
    if (PreviousAngle - Angle > numbers::pi_v<float>) temp += numbers::pi_v<float> * 2;
    else if (PreviousAngle - Angle < -numbers::pi_v<float>) temp -= numbers::pi_v<float> * 2;
    Speed = SpeedFilter(temp * 60 * CurrentCtrlFrequency * numbers::inv_pi_v<float> * 0.5f);
    PreviousAngle = Angle;

    static float ud = 0;
    static float uq = 0;
    if (started || anticogging_calibrating) {
        /**4.电流闭环控制**/
        ud = PID_CurrentD.calc(Id);
        uq = PID_CurrentQ.calc(Iq);
    } else {
        ud = uq = 0;
    }
    SetPhaseVoltage(ud, uq, ElectricalAngle);
}
