/**
 * @file robot_cmd.c
 * @author your name (you@domain.com)
 * @brief 机器人核心控制任务
 * @attention 因为底盘接的是遥控器，但是云台需要进行视觉传输，因此在预编译时不应该屏蔽掉RobotCMDInit，
 *             否则会出现内存访问错误，应在该文件中修改预编译指令。
 *             由于底盘板和云台板的任务都包含有云台电机的任务，因此应该在此处进行双板之间的通信。
 * @version 0.1
 * @date 2024-01-15
 *
 * @copyright Copyright (c) 2024
 *
 */
// application layer for robot command
#include "robot_cmd.h"
#include "robot_def.h"

// module layer
#include "remote.h"
#include "miniPC_process.h"
#include "message_center.h"
#include "user_lib.h"
#include "miniPC_process.h"
#include "referee_protocol.h"
#include "referee_task.h"
#include "referee_UI.h"

#include "arm_math.h"
#include "DJI_motor.h"
#include "UARTComm.h"
#include "bsp_dwt.h"

#include "C_comm.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE     (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)     // pitch水平时电机的角度,0-360

// 对双板的兼容,条件编译
#ifdef GIMBAL_BOARD
#include "UARTComm.h"
static CAN_Comm_Instance *gimbal_can_comm;
static CMD_Gimbal_Send_Data_s gimbal_comm_send;
static CMD_Chassis_Send_Data_s *gimbal_comm_recv;
Vision_Recv_s vision_ctrl; // 视觉控制信息
static RC_ctrl_t *rc_data;         // 遥控器数据指针,初始化时返回
static attitude_t *gimba_IMU_data; // 云台IMU数据
static Subscriber_t *Vision_ctrl_sub;
static Publisher_t *robot_fetch_pub;

static Publisher_t *shoot_cmd_pub;           // 发射摩擦轮控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射摩擦轮反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射摩擦轮的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射摩擦轮获取的反馈信息

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;        // 传递给云台yaw的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data;   // 从云台yaw获取的反馈信息

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
#endif

#ifdef CHASSIS_BOARD
#include "UARTComm.h"
#include "referee_UI.h"
#include "Navi_process.h"
static UARTComm_Instance *chassis_uart_comm; // 双板通信
static CAN_Comm_Instance *chassis_can_comm;
static CMD_Chassis_Send_Data_s chassis_comm_send;

static referee_info_t *referee_data;                         // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data;                   // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
static CMD_Gimbal_Send_Data_s *chassis_comm_recv;
static Navigation_Recv_s *navigation_ctrl; // 视觉控制信息
static IMU_Send_to_Navi imu_send_to_navi;

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_XY rc;
#endif

#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif

static Robot_Upload_Data_s robot_fetch_data; //从裁判系统获取的机器人状态信息

static float chassis_speed_buff;
static void RemoteControlSet(void);  // 遥控器控制量设置
// static void MouseKeySet(void);       // 图传链路控制量设置
// static void RemoteMouseKeySet(void); // 通过遥控器的键鼠控制
static void EmergencyHandler(void) __attribute__((used));
static void CalcOffsetAngle(void); // 计算云台和底盘的偏转角度
static float yaw_target;
static Robot_Status_e robot_state; // 机器人整体工作状态

// static int watch_frispel,watch_frrsper;

#ifdef GIMBAL_BOARD
/**
 * @brief 云台板CMD应用初始化
 * 
 * 本函数负责初始化云台和射击模块所需的发布者和订阅者，
 * 以及遥控器和视觉控制模块。同时，配置和初始化UART通信，
 * 并设置初始的射击和云台控制命令及机器人状态。
 */
void GimbalCMDInit(void)
{
    // 注册云台俯仰角命令的发布者
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    // 注册云台俯仰角反馈的订阅者
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    // 注册射击命令的发布者
    shoot_cmd_pub  = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    // 注册射击反馈的订阅者
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    robot_fetch_pub = PubRegister("robot_fetch", sizeof(Robot_Upload_Data_s));

    Vision_ctrl_sub = SubRegister("vision_ctrl", sizeof(Vision_Recv_s));

    CAN_Comm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id      = 0x312,
            .rx_id      = 0x311,
        },
        .recv_data_len = sizeof(CMD_Chassis_Send_Data_s),
        .send_data_len = sizeof(CMD_Gimbal_Send_Data_s),
    };
    gimbal_can_comm = CANCommInit(&comm_conf);
 
    // 初始化遥控器,使用串口3
    rc_data = RemoteControlInit(&huart3); // 初始化遥控器,C板上使用USART3


    gimba_IMU_data = INS_Init();
    // // 配置UART通信初始化参数
    // UARTComm_Init_Config_s comm_conf = {
    //     .uart_handle = &huart1,
    //     .recv_data_len = sizeof(CMD_Chassis_Send_Data_s),
    //     .send_data_len = sizeof(CMD_Gimbal_Send_Data_s),
    // };

    // // 初始化UART通信
    // gimbal_uart_comm = UARTCommInit(&comm_conf);

    // 初始化射击命令为正常模式、600ms死亡时间、小弹速
    shoot_cmd_send.attack_mode  = NORMAL;
    shoot_cmd_send.dead_time    = 600;
    shoot_cmd_send.bullet_speed = SMALL_AMU_30;

    // 初始化云台控制命令为俯仰角和 yaw 角为0
    gimbal_cmd_send.pitch       = 0;
    gimbal_cmd_send.yaw         = 0;

    // 设置机器人的初始状态为准备就绪
    robot_state                 = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}
float delt_t;
void GimbalCMDGet(void) //获取反馈数据
{
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data); //获取云台反馈数据
    // SubGetMessage(Vision_ctrl_sub, &vision_ctrl);
    delt_t = DWT_GetTimeline_ms()-vision_ctrl.revc_time;

    gimbal_comm_recv = (CMD_Chassis_Send_Data_s *)CANCommGet(gimbal_can_comm);
    // chassis_fetch_data = gimbal_comm_recv->Chassis_fetch_data;
    // shoot_fetch_data = gimbal_comm_recv->Shoot_fetch_data; 
    // gimbal_fetch_data.yaw_motor_single_round_angle = gimbal_comm_recv->Gimbal_fetch_data.yaw_motor_single_round_angle;

    robot_fetch_data = gimbal_comm_recv->Robot_fetch_data;
    shoot_cmd_send.referee_shoot_speed = robot_fetch_data.bullet_speed;
    shoot_cmd_send.rest_heat = robot_fetch_data.shoot_limit - robot_fetch_data.shoot_heat;
}
static float tc,tl,td;
static float hibernate_time,dead_time;
void GimbalCMDSend(void)
{   
    PubPushMessage(gimbal_cmd_pub, (void*)&gimbal_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void*)&shoot_cmd_send);
    PubPushMessage(robot_fetch_pub, (void*)&robot_fetch_data);

    // gimbal_comm_send.Shoot_Ctr_Cmd.is_tracking = shoot_cmd_send.is_tracking;
    // gimbal_comm_send.Shoot_Ctr_Cmd.shoot_rate = shoot_cmd_send.shoot_rate;
    // gimbal_comm_send.Shoot_Ctr_Cmd.load_mode = shoot_cmd_send.load_mode;

    // gimbal_comm_send.Gimbal_Ctr_Cmd.gimbal_imu_data_yaw.YawTotalAngle = gimba_IMU_data->YawTotalAngle;
    // gimbal_comm_send.Gimbal_Ctr_Cmd.gimbal_imu_data_yaw.Gyro = gimba_IMU_data->Gyro[2];
    // gimbal_comm_send.Gimbal_Ctr_Cmd.gimbal_mode = gimbal_cmd_send.gimbal_mode;
    // gimbal_comm_send.Gimbal_Ctr_Cmd.yaw = gimbal_cmd_send.yaw;
    // gimbal_comm_send.Gimbal_Ctr_Cmd.chassis_rotate_wz = chassis_cmd_send.wz;

    // gimbal_comm_send.Gimbal_Ctr_Cmd = gimbal_cmd_send;
    gimbal_comm_send.Chassis_Ctr_Cmd.vx = chassis_cmd_send.vx;
    gimbal_comm_send.Chassis_Ctr_Cmd.vy = chassis_cmd_send.vy;
    gimbal_comm_send.Chassis_Ctr_Cmd.wz = chassis_cmd_send.wz;
    gimbal_comm_send.Chassis_Ctr_Cmd.super_cap_mode = chassis_cmd_send.super_cap_mode;
    gimbal_comm_send.Chassis_Ctr_Cmd.chassis_mode = chassis_cmd_send.chassis_mode;
    gimbal_comm_send.Chassis_Ctr_Cmd.offset_angle = chassis_cmd_send.offset_angle;

    // gimbal_comm_send.Chassis_Ctr_Cmd.rc_l1 = rc_data->rc.rocker_l1;
    // gimbal_comm_send.Chassis_Ctr_Cmd.rc_l_ = rc_data->rc.rocker_l_;
    // gimbal_comm_send.Chassis_Ctr_Cmd.rc_r1 = rc_data->rc.rocker_r1;
    // gimbal_comm_send.Chassis_Ctr_Cmd.rc_r_ = rc_data->rc.rocker_r_;

    gimbal_comm_send.Chassis_Ctr_Cmd.yaw = gimba_IMU_data->Yaw;
    gimbal_comm_send.Chassis_Ctr_Cmd.pitch = gimba_IMU_data->Pitch;
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    return;
    else
    {
    CANCommSend(gimbal_can_comm, (uint8_t *)&gimbal_comm_send);
    hibernate_time = DWT_GetTimeline_ms();     // 记录触发指令的时间
    dead_time      = 1; // 2ms发送一次   
    }
    tc = DWT_GetTimeline_ms();
    td = tc-tl;
    tl = tc;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void GimbalCMDTask(void)
{
    //获取各个模块数据
    GimbalCMDGet();
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    shoot_cmd_send.rest_heat = robot_fetch_data.shoot_limit - robot_fetch_data.shoot_heat - 20; // 计算剩余热量
    RemoteControlSet();
  

    // 发送控制信息
    // 推送消息,双板通信,视觉通信等
    // chassis_cmd_send.friction_mode = shoot_cmd_send.friction_mode;
   // chassis_cmd_send.vision_mode   = vision_ctrl->is_tracking ? LOCK : UNLOCK;
    // chassis_cmd_send.lid_mode      = shoot_cmd_send.lid_mode;
    //发送指令
    GimbalCMDSend();
    
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet(void)
{
    robot_state                     = ROBOT_READY;
    shoot_cmd_send.shoot_mode       = SHOOT_ON;
    chassis_cmd_send.chassis_mode   = CHASSIS_SLOW; // 底盘模式
    gimbal_cmd_send.gimbal_mode     = GIMBAL_GYRO_MODE;
    chassis_cmd_send.super_cap_mode = SUPER_CAP_ON;


    // 上巡航+导航
    //中遥控+自瞄
    //下导航
    
    //底盘参数
    //上：导航小陀螺 底盘小陀螺
    //中：遥控  底盘跟随
    //底：纯导航 底盘跟随
    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    // max 70.f,参数过大会达到电机的峰值速度，导致底盘漂移等问题，且毫无意义
    chassis_cmd_send.vx = -80.0f * (float)rc_data[TEMP].rc.rocker_l_; // _水平方向
    chassis_cmd_send.vy = -80.0f * (float)rc_data[TEMP].rc.rocker_l1; // 1竖直方向
    if(switch_is_down(rc_data[TEMP].rc.switch_right)){
        if(rc_data[TEMP].rc.dial>640||rc_data[TEMP].rc.dial<-640)
        chassis_cmd_send.wz = 0;
        else chassis_cmd_send.wz = -50.0f * (float)rc_data[TEMP].rc.dial/2;
    }else if(switch_is_up(rc_data[TEMP].rc.switch_right)){
        chassis_cmd_send.wz = -30*400;
    }
    else{
        chassis_cmd_send.wz = 0;
    }
    // chassis_cmd_send.wz = (float)rc_data[TEMP].rc.dial*360/660+chassis_cmd_send.wz;
    // chassis_cmd_send.chassis_angle = chassis_cmd_send.chassis_angle;         
    if(switch_is_up(rc_data[TEMP].rc.switch_right) || switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NAV;
    }
    else 
    {
        chassis_cmd_send.chassis_mode = CHASSIS_GIMBAL_FOLLOW;
    }

    
    //云台参数 //上巡航，中下遥控
    // 自瞄，左侧开关为中上是自瞄
    if ((switch_is_mid(rc_data[TEMP].rc.switch_left) || switch_is_up(rc_data[TEMP].rc.switch_left)) && vision_ctrl.is_tracking) // 左侧开关状态为[中] / [上],视觉模式
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_cmd_send.yaw   = (vision_ctrl.yaw == 0 ? gimbal_cmd_send.yaw : -vision_ctrl.yaw);
        gimbal_cmd_send.pitch = (vision_ctrl.pitch == 0 ? gimbal_cmd_send.pitch : vision_ctrl.pitch);
    }
    else if(switch_is_up(rc_data[TEMP].rc.switch_right)&&!vision_ctrl.is_tracking){
    
        gimbal_cmd_send.gimbal_mode     = GIMBAL_CRUISE_MODE;
        yaw_target += 0.003f * 90;
        gimbal_cmd_send.yaw =yaw_target;
    }
    else {
        gimbal_cmd_send.gimbal_mode     = GIMBAL_GYRO_MODE ;
        yaw_target += 0.003f * (float)rc_data[TEMP].rc.rocker_r_;
        gimbal_cmd_send.pitch += 0.004f * (float)rc_data[TEMP].rc.rocker_r1;
        gimbal_cmd_send.yaw =yaw_target;
    }

    // if(gimba_IMU_data->YawTotalAngle-gimbal_cmd_send.yaw>360 || gimba_IMU_data->YawTotalAngle-gimbal_cmd_send.yaw<-360){
    //     gimbal_cmd_send.yaw=gimba_IMU_data->YawTotalAngle;
    // }


    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    // if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
    //    gimbal_cmd_send.gimbal_mode     = GIMBAL_CRUISE_MODE;
    //     // 按照摇杆的输出大小进行角度增量,增益系数需调整
    //     gimbal_cmd_send.yaw -= 0.001f * (float)rc_data[TEMP].rc.rocker_r_;
    //     gimbal_cmd_send.pitch += 0.004f * (float)rc_data[TEMP].rc.rocker_r1;
    // }

    // // 云台参数,确定云台控制数据
    // if ((switch_is_mid(rc_data[TEMP].rc.switch_left) || switch_is_up(rc_data[TEMP].rc.switch_left)) && vision_ctrl->is_tracking) // 左侧开关状态为[中] / [上],视觉模式
    // {
    //      gimbal_cmd_send.gimbal_mode     = GIMBAL_GYRO_MODE;
    //     gimbal_cmd_send.yaw   = (vision_ctrl->yaw == 0 ? gimbal_cmd_send.yaw : vision_ctrl->yaw);
    //     gimbal_cmd_send.pitch = (vision_ctrl->pitch == 0 ? gimbal_cmd_send.pitch : vision_ctrl->pitch);
    // }

    // 云台软件限位
    if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
        gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
        gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;

    
    // // 发射参数
 

    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (switch_is_mid(rc_data[TEMP].rc.switch_left) || switch_is_up(rc_data[TEMP].rc.switch_left)) // 向上超过100,打开摩擦轮
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else{
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    }
    // 拨弹控制,遥控器固定为一种拨弹模式,
    
   
    if( shoot_cmd_send.friction_mode == FRICTION_ON ){


        if ((vision_ctrl.is_shooting == 1 && switch_is_up(rc_data[TEMP].rc.switch_left))&& shoot_cmd_send.rest_heat >= 20)
        {
            shoot_cmd_send.load_mode = LOAD_MEDIUM;
            shoot_cmd_send.shoot_rate = SHOOT_RATE;
        }   
   
    else{shoot_cmd_send.load_mode = LOAD_STOP;}
     }
    // if (vision_ctrl->is_shooting == 0)
    //     shoot_cmd_send.load_mode = LOAD_STOP;

    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,


}
#endif // DEBUG
#ifdef CHASSIS_BOARD
/**
 * @brief 初始化底盘板CMD应用
 * 
 * 该函数负责初始化与底盘、云台和射击控制相关的命令和反馈数据流的发布者和订阅者。
 * 通过调用PubRegister和SubRegister函数，为后续的数据通信建立通道。
 * 
 * 
 */
void ChassisCMDInit(void)
{   

    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    // // 注册云台俯仰角命令的发布者
    // gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    // // 注册云台俯仰角反馈的订阅者
    // gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    // // 注册射击命令的发布者
    // shoot_cmd_pub  = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    // // 注册射击反馈的订阅者
    // shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI    

    navigation_ctrl = NavigationInit(&huart1);
    // 配置UART通信初始化参数
    // UARTComm_Init_Config_s comm_conf = {
    //     .uart_handle = &huart1,
    //     .recv_data_len = sizeof(CMD_Gimbal_Send_Data_s),
    //     .send_data_len = sizeof(CMD_Chassis_Send_Data_s),
    // };

    // // 初始化UART通信
    // chassis_uart_comm = UARTCommInit(&comm_conf);

    CAN_Comm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0x311,
            .rx_id      = 0x312,
        },
        .recv_data_len = sizeof(CMD_Gimbal_Send_Data_s),
        .send_data_len = sizeof(CMD_Chassis_Send_Data_s),
    };
    chassis_can_comm = CANCommInit(&comm_conf);




    // 设置机器人的初始状态为准备就绪
    robot_state                 = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入  
}

void ChassisCMDGet(void)
{
    // chassis_comm_recv = (CMD_Gimbal_Send_Data_s *)UARTCommGet(chassis_uart_comm);
    chassis_comm_recv = (CMD_Gimbal_Send_Data_s *)CANCommGet(chassis_can_comm);
    chassis_cmd_send = chassis_comm_recv->Chassis_Ctr_Cmd;
    // gimbal_cmd_send = chassis_comm_recv->Gimbal_Ctr_Cmd;
    // shoot_cmd_send = chassis_comm_recv->Shoot_Ctr_Cmd;
    // gimbal_cmd_send.gimbal_imu_data_yaw.Gyro = gimbal_cmd_send.gimbal_imu_data_yaw.Gyro;//cc;
    chassis_cmd_send.chassis_power_buff = referee_data->PowerHeatData.buffer_energy;
    chassis_cmd_send.chassis_power_limit = referee_data->GameRobotState.chassis_power_limit;
    
    // shoot_cmd_send.shoot_rate = chassis_comm_recv->Shoot_Ctr_Cmd.shoot_rate;
    // shoot_cmd_send.load_mode = chassis_comm_recv->Shoot_Ctr_Cmd.load_mode;
    // shoot_cmd_send.is_tracking = chassis_comm_recv->Shoot_Ctr_Cmd.is_tracking;

    // gimbal_cmd_send.yaw = chassis_comm_recv->Gimbal_Ctr_Cmd.yaw;
    // gimbal_cmd_send.gimbal_imu_data_yaw.YawTotalAngle = chassis_comm_recv->Gimbal_Ctr_Cmd.gimbal_imu_data_yaw.YawTotalAngle;
    // gimbal_cmd_send.gimbal_imu_data_yaw.Gyro = chassis_comm_recv->Gimbal_Ctr_Cmd.gimbal_imu_data_yaw.Gyro;
    // gimbal_cmd_send.gimbal_mode = chassis_comm_recv->Gimbal_Ctr_Cmd.gimbal_mode;
    // gimbal_cmd_send.chassis_rotate_wz = chassis_comm_recv->Gimbal_Ctr_Cmd.chassis_rotate_wz;

    chassis_cmd_send.vx = chassis_comm_recv->Chassis_Ctr_Cmd.vx;
    chassis_cmd_send.vy = chassis_comm_recv->Chassis_Ctr_Cmd.vy;
    chassis_cmd_send.wz = chassis_comm_recv->Chassis_Ctr_Cmd.wz;
    chassis_cmd_send.chassis_mode = chassis_comm_recv->Chassis_Ctr_Cmd.chassis_mode;
    chassis_cmd_send.super_cap_mode = chassis_comm_recv->Chassis_Ctr_Cmd.super_cap_mode;
    chassis_cmd_send.offset_angle = chassis_comm_recv->Chassis_Ctr_Cmd.offset_angle;
    chassis_cmd_send.chassis_power_buff = referee_data->PowerHeatData.buffer_energy;
    chassis_cmd_send.chassis_power_limit = referee_data->GameRobotState.chassis_power_limit;

    // rc.rc_l1 = chassis_comm_recv->Chassis_Ctr_Cmd.rc_l1;
    // rc.rc_l_ = chassis_comm_recv->Chassis_Ctr_Cmd.rc_l_;
    // rc.rc_r1 = chassis_comm_recv->Chassis_Ctr_Cmd.rc_r1;
    // rc.rc_r_ = chassis_comm_recv->Chassis_Ctr_Cmd.rc_r_;

    imu_send_to_navi.yaw=  chassis_cmd_send.yaw;
    imu_send_to_navi.pitch = chassis_cmd_send.pitch;
    imu_send_to_navi.roll = 0;
    //TODO：发射，云台，Robot 状态反馈
}

void RobotStateGet(void)
{
 /*
    robot_fetch_data.shoot_heat   = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    robot_fetch_data.shoot_limit  = referee_data->GameRobotState.shooter_barrel_heat_limit;
    robot_fetch_data.bullet_speed = referee_data->ShootData.bullet_speed;
    // 我方颜色id小于10是红色,大于10是蓝色,注意这里发送的是自己的颜色, 1:blue , 2:red
    robot_fetch_data.self_color = referee_data->GameRobotState.robot_id > 10 ? COLOR_BLUE : COLOR_RED;

    // robot_fetch_data.self_color = referee_data;
    // robot_fetch_data.shoot_heat = referee_data->ShootData;
*/
    robot_fetch_data.shoot_heat   = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    robot_fetch_data.shoot_limit  = referee_data->GameRobotState.shooter_barrel_heat_limit;
    robot_fetch_data.bullet_speed = referee_data->ShootData.bullet_speed;
    // 我方颜色id小于10是红色,大于10是蓝色,注意这里发送的是自己的颜色, 1:blue , 2:red
    robot_fetch_data.self_color = referee_data->GameRobotState.robot_id > 10 ? COLOR_BLUE : COLOR_RED;

}

void ChassisCMDSend(void)
{/*
    // PubPushMessage(gimbal_cmd_pub, (void*)&gimbal_cmd_send);
    // PubPushMessage(shoot_cmd_pub, (void*)&shoot_cmd_send);
    PubPushMessage(chassis_cmd_pub, (void*)&chassis_cmd_send);

     SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);


    //chassis_comm_send.Chassis_fetch_data = &chassis_fetch_data;
    chassis_comm_send.Gimbal_fetch_data = gimbal_fetch_data;
    chassis_comm_send.Robot_fetch_data = robot_fetch_data; 
    //TODO:robot_fetch_data 待赋值

    //chassis_comm_send.Shoot_fetch_data = &shoot_fetch_data;
    // UARTCommSend(chassis_uart_comm,(uint8_t*)&chassis_comm_send);
    */
    PubPushMessage(chassis_cmd_pub, (void*)&chassis_cmd_send); 

    chassis_comm_send.Robot_fetch_data = robot_fetch_data;

    CANCommSend(chassis_can_comm,(uint8_t*)&chassis_comm_send);
}

void ChassisCMDTask(void)
{
    ChassisCMDGet();
    RobotStateGet();

    if(chassis_cmd_send.chassis_mode == CHASSIS_GIMBAL_FOLLOW)
    {
    
    }
    else if (chassis_cmd_send.chassis_mode == CHASSIS_NAV) //导航与遥控混合控制 ATTENTION：此处各个if的先后顺序极其重要！谨慎修改！
    {
        if((navigation_ctrl->wz>10||navigation_ctrl->wz<-10)){navigation_ctrl->wz=0;}
        if((navigation_ctrl->wz>-0.2&&navigation_ctrl->wz<0.2)){navigation_ctrl->wz=0;}
       if(navigation_ctrl->vx != 0 || navigation_ctrl->vy != 0||navigation_ctrl->wz !=0) //&& referee_data->GameState.game_progress == 4)  //导航控制
        {
        
            //TODO:game progress限制待添加
            chassis_cmd_send.vx = navigation_ctrl->vx * NAV_K;
            chassis_cmd_send.vy = navigation_ctrl->vy * NAV_K;
            chassis_cmd_send.wz = -navigation_ctrl->wz * NAV_K*0.5;
            chassis_cmd_send.offset_angle = NAV_OFFSET_ANGLE;


        }
        // else{
        //     // chassis_cmd_send.offset_angle = NAV_OFFSET_ANGLE;
        //     chassis_cmd_send.vx = chassis_cmd_send.vx;
        //     chassis_cmd_send.vy = chassis_cmd_send.vy;
        //     chassis_cmd_send.wz = chassis_cmd_send.wz;
        // }

        // if (navigation_ctrl->spiral_mode == SPIRAL_ON) //决策小陀螺控制
        // {
        //     chassis_cmd_send.wz = 18000;
        // }
        // else if (navigation_ctrl->spiral_mode == SPIRAL_OFF)
        // {
        //     chassis_cmd_send.wz = 0;
        // }
    }

    ChassisCMDSend();
    NavigationSend(referee_data,rc,imu_send_to_navi);
}
#endif




// static void RemoteMouseKeySet(void)
// {
//     // switch (video_data[TEMP].key_count[V_KEY_PRESS_WITH_CTRL][V_Key_X] % 2) {
//     //     case 0:
//     //         EmergencyHandler();
//     //         return; // 当没有按下激活键时,直接返回
//     //     default:
//     //         break; // 当按下激活键时,继续执行
//     // }
//     robot_state                     = ROBOT_READY;
//     shoot_cmd_send.shoot_mode       = SHOOT_ON;
//     chassis_cmd_send.chassis_mode   = CHASSIS_SLOW; // 底盘模式
//     gimbal_cmd_send.gimbal_mode     = GIMBAL_GYRO_MODE;
//     chassis_cmd_send.super_cap_mode = SUPER_CAP_ON;

//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2) {
//         case 0:
//             chassis_speed_buff              = 0.8f;
//             chassis_cmd_send.chassis_mode   = CHASSIS_SLOW;
//             chassis_cmd_send.super_cap_mode = SUPER_CAP_OFF;
//             break;
//         default:
//             chassis_speed_buff              = 2.5f;
//             chassis_cmd_send.chassis_mode   = CHASSIS_MEDIUM;
//             chassis_cmd_send.super_cap_mode = SUPER_CAP_ON;
//             break;
//     }

//     if (rc_data[TEMP].key[KEY_PRESS].x) {
//         chassis_speed_buff              = 1.f;
//         chassis_cmd_send.chassis_mode   = CHASSIS_FOLLOW_GIMBAL_YAW;
//         chassis_cmd_send.super_cap_mode = SUPER_CAP_ON;
//     }

//     // 若在底盘跟随云台模式下按住shift键，则强制改为小陀螺模式
//     if (rc_data[TEMP].key[KEY_PRESS].shift && chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
//         chassis_speed_buff              = 2.5f;
//         chassis_cmd_send.chassis_mode   = CHASSIS_MEDIUM;
//         chassis_cmd_send.super_cap_mode = SUPER_CAP_ON;
//     }

//     chassis_cmd_send.vx = -(rc_data[TEMP].key[KEY_PRESS].d - rc_data[TEMP].key[KEY_PRESS].a) * 50000 * chassis_speed_buff; // 系数待测
//     chassis_cmd_send.vy = -(rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s) * 50000 * chassis_speed_buff;
//     chassis_cmd_send.wz = rc_data[TEMP].key[KEY_PRESS].shift * 24000 * chassis_speed_buff;

//     gimbal_cmd_send.yaw -= (float)rc_data[TEMP].mouse.x / 660 * 2.5; // 系数待测
//     gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 2.5;

//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 2) {
//         case 0:
//             chassis_cmd_send.vision_lock_mode = ARMOR;
//             gimbal_cmd_send.vision_lock_mode  = ARMOR;
//             VisionSetEnergy(0);
//             break;
//         default:
//             chassis_cmd_send.vision_lock_mode = RUNNE;
//             gimbal_cmd_send.vision_lock_mode  = RUNNE;
//             VisionSetEnergy(1);
//             break;
//     }

//     if (rc_data[TEMP].mouse.z != 0) {
//         VisionSetReset(1);
//     } else {
//         VisionSetReset(0);
//     }
//     if (vision_ctrl->is_tracking) {
//         if (vision_ctrl->is_shooting) {
//             chassis_cmd_send.vision_mode = LOCK;
//             gimbal_cmd_send.vision_mode  = LOCK;
//         } else {
//             chassis_cmd_send.vision_mode = UNLOCK;
//             gimbal_cmd_send.vision_mode  = UNLOCK;
//         }
//         if (rc_data[TEMP].mouse.press_r) // 右键开启自瞄
//         {
//             gimbal_cmd_send.yaw   = (vision_ctrl->yaw == 0 ? gimbal_cmd_send.yaw : vision_ctrl->yaw);
//             gimbal_cmd_send.pitch = (vision_ctrl->pitch == 0 ? gimbal_cmd_send.pitch : vision_ctrl->pitch);
//         }
//     } else {
//         chassis_cmd_send.vision_mode = UNLOCK;
//         gimbal_cmd_send.vision_mode  = UNLOCK;
//     }

//     // 云台软件限位
//     if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
//         gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
//     else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
//         gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;

//     // V键刷新UI
//     if (rc_data[TEMP].key[KEY_PRESS].v) {
//         chassis_cmd_send.ui_mode = UI_REFRESH;
//     } else {
//         chassis_cmd_send.ui_mode = UI_KEEP;
//     }

//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 2) // Q键开关摩擦轮
//     {
//         case 0:
//             shoot_cmd_send.friction_mode = FRICTION_OFF;
//             break;
//         default:
//             shoot_cmd_send.friction_mode = FRICTION_ON;
//             break;
//     }

//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 4) // B键切换发弹模式
//     {
//         case 0:
//             shoot_cmd_send.load_mode     = LOAD_SLOW;
//             chassis_cmd_send.loader_mode = LOAD_SLOW; // 在此处处理是为了刷新UI
//             shoot_cmd_send.shoot_rate    = 4;
//             break;
//         case 1:
//             shoot_cmd_send.load_mode     = LOAD_MEDIUM;
//             chassis_cmd_send.loader_mode = LOAD_MEDIUM;
//             shoot_cmd_send.shoot_rate    = 8;
//             break;
//         case 2:
//             shoot_cmd_send.load_mode     = LOAD_FAST;
//             chassis_cmd_send.loader_mode = LOAD_FAST;
//             shoot_cmd_send.shoot_rate    = 12;
//             break;
//         default:
//             shoot_cmd_send.load_mode     = LOAD_1_BULLET;
//             chassis_cmd_send.loader_mode = LOAD_1_BULLET;
//             break;
//     }

//     if (!rc_data[TEMP].mouse.press_l ||
//         shoot_cmd_send.friction_mode == FRICTION_OFF ||
//         shoot_cmd_send.rest_heat <= 0) {
//         shoot_cmd_send.load_mode = LOAD_STOP;
//     }

//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) {
//         case 0:
//             chassis_cmd_send.vision_is_shoot = IS_SHOOTING_ON;
//             if (vision_ctrl->is_shooting == 0 && vision_ctrl->is_tracking == 1) {
//                 shoot_cmd_send.load_mode = LOAD_STOP;
//             }
//             break;
//         case 1:
//             chassis_cmd_send.vision_is_shoot = IS_SHOOTING_OFF;
//             break;
//     }
//     // 这行代码在演
//     // if (vision_ctrl->is_shooting == 0 && vision_ctrl->is_tracking == 1 &&
//     //     rc_data[TEMP].mouse.press_r) {
//     //     shoot_cmd_send.load_mode = LOAD_STOP;
//     // }

//     if (rc_data[TEMP].key[KEY_PRESS].f) // F键开启拨盘反转模式
//     {
//         shoot_cmd_send.load_mode     = LOAD_REVERSE;
//         chassis_cmd_send.loader_mode = LOAD_REVERSE;
//     }

//     switch (rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL][Key_E] % 2) // E键开关弹舱
//     {
//         case 0:
//             shoot_cmd_send.lid_mode = LID_CLOSE;
//             break;
//         default:
//             shoot_cmd_send.lid_mode = LID_OPEN;
//             break;
//     }
// }


#ifdef GIMBAL_BOARD
 
/**
 * @brief  紧急停止,包括遥控器右侧上侧拨杆打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler(void)
{
    // 急停
    robot_state                     = ROBOT_STOP;
    gimbal_cmd_send.gimbal_mode     = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode   = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.super_cap_mode = SUPER_CAP_ON;
    shoot_cmd_send.shoot_mode       = SHOOT_OFF;
    shoot_cmd_send.friction_mode    = FRICTION_OFF;
    shoot_cmd_send.load_mode        = LOAD_STOP;
    shoot_cmd_send.lid_mode         = LID_CLOSE;
}

#endif // DEBUG