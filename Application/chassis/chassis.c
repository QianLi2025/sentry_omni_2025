#include "chassis.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "referee_task.h"
#include "general_def.h"
#include "user_lib.h"
#include "referee_UI.h"
#include "super_cap.h"

#include "bsp_dwt.h"
#include "arm_math.h"

#ifdef CHASSIS_BOARD
#include "ins_task.h"
#include "C_comm.h"
static CAN_Comm_Instance *chasiss_can_comm; // 用于底盘的CAN通信
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD

static Publisher_t *chassis_pub;  // 用于发布底盘的数据
static Subscriber_t *chassis_sub; // 用于订阅底盘的控制命令

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;                  // 底盘接收到的控制命令
__unused static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
// static referee_info_t *referee_data;                         // 用于获取裁判系统的数据
// static Referee_Interactive_info_t ui_data;                   // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCap_Instance *super_cap;                                 // 超级电容实例
static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
static PID_Instance chassis_follow_pid;  // 底盘跟随PID
// 功率限制算法的变量定义
// static float K_limit = 1.0f, P_limit = 0;                    // 功率限制系数
// static float chassis_power;                                  // 底盘功率
// static uint16_t chassis_power_buffer;                        // 底盘功率缓冲区
// static float chassis_speed_err;                              // 底盘速度误差
// static float scaling_lf, scaling_rf, scaling_lb, scaling_rb; // 电机输出缩放系数
// #define CHASSIS_MAX_POWER 240000.f                           // 底盘最大功率,15384 * 4，取了4个3508电机最大电流的一个保守值
// #define CHASSIS_MAX_SPEED 240000.f                           // 底盘最大速度,单位mm/s
#ifdef CHASSIS_MCNAMEE_WHEEL
#define CHASSIS_WHEEL_OFFSET 1.0f // 机器人底盘轮子修正偏移量
#elif defined(CHASSIS_OMNI_WHEEL)
#define CHASSIS_WHEEL_OFFSET 0.7071f // 机器人底盘轮子修正偏移量，根号2/2，即45度，用于修正全向轮的安装位置
#endif                               // CHASSIS_OMNI_WHEEL

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle   = &hcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 17, // 4.5
                .Ki            = 0.5, // 0
                .Kd            = 0, // 0
                .IntegralLimit = 12000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
            .current_PID = {
                .Kp            = 0.5, // 0.4
                .Ki            = 0.01,  // 0
                .Kd            = 0,
                .IntegralLimit = 16384,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16384,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id                             = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp                = 350,
        .Ki                = 2,
        .Kd                = 1.3,
        .MaxOut            = 15000,
        .DeadBand          = 0.1,
        .Improve           = PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.1,
    };
    PIDInit(&chassis_follow_pid, &chassis_follow_pid_conf);

    // referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

    SuperCap_Init_Config_s super_cap_config = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id      = 0x302,
            .rx_id      = 0x301,
        },
    };
    super_cap = SuperCapInit(&super_cap_config); // 超级电容初始化

#ifdef CHASSIS_BOARD
    Chassis_IMU_data                 = INS_Init(); // 底盘IMU初始化
    CAN_Comm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id      = 0x311,
            .rx_id      = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD


    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

}

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = (chassis_vx + chassis_vy) * CHASSIS_WHEEL_OFFSET - chassis_cmd_recv.wz; // 1
    vt_rf = (-chassis_vx + chassis_vy) * CHASSIS_WHEEL_OFFSET + chassis_cmd_recv.wz;  // 2
    vt_rb = (chassis_vx + chassis_vy) * CHASSIS_WHEEL_OFFSET + chassis_cmd_recv.wz; // 3
    vt_lb = (-chassis_vx + chassis_vy) * CHASSIS_WHEEL_OFFSET - chassis_cmd_recv.wz;  // 4
}

/**
 * @brief 电机速度限制
 *
 */


int w_watch;
/**
 * @brief
 *
 */
// ** /
static int v_max;
static void LimitChassisOutput()
{
    // // 功率限制待添加
    // uint16_t chassis_power_buffer = 0; // 底盘功率缓冲区
    // // uint16_t chassis_power        = 0;
    // uint16_t chassis_power_limit  = 0;
    // float P_limit                 = 1; // 功率限制系数

    // chassis_power_buffer = chassis_cmd_recv.chassis_power_buff;
    // chassis_power_limit  = chassis_cmd_recv.chassis_power_limit;

    // 功率限制待添加
    uint16_t chassis_power_buffer = chassis_cmd_recv.chassis_power_buff; // 底盘功率缓冲区
    float P_limit = 1; // 功率限制系数


    // 获取当前机器人状态下的底盘功率限制
    int W_limit =chassis_cmd_recv.chassis_power_limit;
  
    // 计算四个轮子目标速度的平均值
    float v_arv = (abs(vt_lf)+abs(vt_rf)+abs(vt_lb)+abs(vt_rb))/4;

    // 如果当前平均速度大于最大速度记录，则更新最大速度记录
    if(v_arv > v_max)
    {
        v_max = v_arv;
    }
    if(super_cap->state == SUP_CAP_STATE_CHARGING){   
    // 根据当前平均速度和最大速度调整功率限制
    W_limit = W_limit*(1 - v_arv/v_max);
    // 如果调整后的功率限制小于等于0，则设置为底盘功率限制的20%
    if(W_limit <= 0)
    {
        W_limit = chassis_cmd_recv.chassis_power_limit * 0.2;
    }
    }

    // 更新监控的功率限制值
    w_watch = W_limit;

    SuperCapSet(chassis_power_buffer, W_limit, 3); //3开启超电


    SuperCapSend(); // 发送超级电容数据
    // 完成功率限制后进行电机参考输入设定

    // 根据底盘电源缓冲区的值来设定功率限制
    if (chassis_power_buffer >= 30) {
        // 当底盘电源缓冲区的值大于等于30时，将功率限制设置为1（即最大功率）
        P_limit = 1;
    } else {
        // 当底盘电源缓冲区的值小于30时，将功率限制设置为缓冲区值除以30的结果
        // 这样可以确保在电源缓冲区较低时，功率限制较小，以防超功率扣血
        P_limit = chassis_power_buffer / 20.f;
    }

    DJIMotorSetRef(motor_lf, vt_lf * P_limit);
    DJIMotorSetRef(motor_rf, vt_rf * P_limit);
    DJIMotorSetRef(motor_lb, vt_lb * P_limit);
    DJIMotorSetRef(motor_rb, vt_rb * P_limit);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
    // max 48000
}

float offset_angle;
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
    SubGetMessage(chassis_sub, &chassis_cmd_recv);

       
    // if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    //     DJIMotorStop(motor_lf);
    //     DJIMotorStop(motor_rf);
    //     DJIMotorStop(motor_lb);
    //     DJIMotorStop(motor_rb);
    // } else { // 正常工作
    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_lb);
    DJIMotorEnable(motor_rb);
    //}

    // 根据控制模式设定旋转速度
    // 根据控制模式设定旋转速度
   //chassis_cmd_recv.chassis_mode =  CHASSIS_FOLLOW_GIMBAL_YAW
    // switch (chassis_cmd_recv.chassis_mode) 
    // {
    //     case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台PID模式
    //         chassis_cmd_recv.wz = PIDCalculate(&chassis_follow_pid, chassis_cmd_recv.offset_angle, 0);
    //         break;
    //     case CHASSIS_FIXED_ANGULAR_RATE: // 新增固定角速度模式
    //         chassis_cmd_recv.wz = 100.0f; // 示例固定值（单位：度/秒），根据实际需求修改
    //         break;
    //     case CHASSIS_DIRECT_GIMBAL_WZ: // 直接使用云台原始wz（新增）
    //     // 无需计算，直接保留来自云台板的原始wz值
    //         break;
    //     default:
    //         break;
    // }
switch (chassis_cmd_recv.chassis_mode)
{
    case CHASSIS_NAV:
        
    break;

    case CHASSIS_GIMBAL_FOLLOW :
        offset_angle = chassis_cmd_recv.offset_angle;
        if (offset_angle<1 && offset_angle>-1)
        {
            offset_angle = 0;
        }
        chassis_cmd_recv.wz = PIDCalculate(&chassis_follow_pid, offset_angle,0);
    break;

    default:
    break;
}


    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    // cos_theta = 0, sin_theta = 1;
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // chassis_feedback_data.shoot_heat   = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    // chassis_feedback_data.shoot_limit  = referee_data->GameRobotState.shooter_barrel_heat_limit;
    // chassis_feedback_data.bullet_speed = referee_data->ShootData.bullet_speed;
    // // 我方颜色id小于10是红色,大于10是蓝色,注意这里发送的是自己的颜色, 1:blue , 2:red
    // chassis_feedback_data.self_color = referee_data->GameRobotState.robot_id > 10 ? COLOR_BLUE : COLOR_RED;

    // ui_data.ui_mode          = chassis_cmd_recv.ui_mode;
    // ui_data.chassis_mode     = chassis_cmd_recv.chassis_mode;
    // ui_data.friction_mode    = chassis_cmd_recv.friction_mode;
    // ui_data.vision_mode      = chassis_cmd_recv.vision_mode;
    // ui_data.vision_lock_mode = chassis_cmd_recv.vision_lock_mode;
    // ui_data.level            = referee_data->GameRobotState.robot_level;
    // ui_data.lid_mode         = chassis_cmd_recv.lid_mode;
    // ui_data.super_cap_mode   = chassis_cmd_recv.super_cap_mode;
    // ui_data.loader_mode      = chassis_cmd_recv.loader_mode;
    // ui_data.vision_is_shoot  = chassis_cmd_recv.vision_is_shoot;
    // 推送反馈消息

    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);


}