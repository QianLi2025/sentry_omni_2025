/**
  * @file       robot_param_omni_infantry.h
  * @brief      这里是全向轮步兵机器人参数配置文件，包括物理参数、PID参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.1.0     2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *  V1.1.1     2025-1-15     CJH               1. 完成射击基本功能
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL  // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT    // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER            // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL  // 选择控制类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  //选择机械臂类型

typedef enum {
    // 底盘CAN1
    WHEEL1 = 0,
    WHEEL2 = 1,
    WHEEL3 = 2,
    WHEEL4 = 3,
    // 云台CAN2
    YAW = 4,
    PITCH = 5,
    TRIGGER = 6,
    FRIC1 = 0,
    FRIC2 = 1,
} DJIMotorIndex_e;  //DJI电机在接收数据数组中的索引

// 底盘的遥控器相关宏定义 ---------------------
#define CHASSIS_MODE_CHANNEL   0  // 选择底盘状态 开关通道号
#define CHASSIS_X_CHANNEL      1  // 前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL      0  // 左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL     0  // 旋转的遥控器通道号码
#define CHASSIS_ROLL_CHANNEL   4  // ROLL角的遥控器通道号码
#define CHASSIS_RC_DEADLINE    5  // 摇杆死区

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define WHEEL_RADIUS 0.106f  //(m)轮子半径
#define WHEEL_DIRECTION 1    //轮子方向
#define CAR_RADIUS 0.23f//(m)车中轴到轮子的距离
//upper_limit parameters ---------------------
#define MAX_SPEED_VECTOR_VX 1.5f
#define MAX_SPEED_VECTOR_VY 1.5f
#define MAX_SPEED_VECTOR_WZ 3.0f

//lower_limit parameters ---------------------
#define MIN_SPEED_VECTOR_VX -MAX_SPEED_VECTOR_VX
#define MIN_SPEED_VECTOR_VY -MAX_SPEED_VECTOR_VY
#define MIN_SPEED_VECTOR_WZ -MAX_SPEED_VECTOR_WZ

//PID parameters ---------------------
//驱动轮速度环PID参数
#define KP_CHASSIS_WHEEL_SPEED 193.0f
#define KI_CHASSIS_WHEEL_SPEED 0.0f
#define KD_CHASSIS_WHEEL_SPEED 0.4f
#define MAX_IOUT_CHASSIS_WHEEL_SPEED 10000.0f
#define MAX_OUT_CHASSIS_WHEEL_SPEED 30000.0f

//云台跟随角度环PID参数
#define KP_CHASSIS_GIMBAL_FOLLOW_ANGLE 20.0f
#define KI_CHASSIS_GIMBAL_FOLLOW_ANGLE 0.03f
#define KD_CHASSIS_GIMBAL_FOLLOW_ANGLE 0.3f
#define MAX_IOUT_CHASSIS_GIMBAL_FOLLOW_ANGLE 10000.0f
#define MAX_OUT_CHASSIS_GIMBAL_FOLLOW_ANGLE 300000.0f

#define MAX_ROLL            (0.3f)
#define MIN_ROLL            (-MAX_ROLL)

//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.6f
//rocker value (max 660) change to horizontal speed (m/s)
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.6f//0.5f
#define CHASSIS_WX_RC_SEN 0.5f

#define CHASSIS_WZ_SET_SCALE 0.1f
#define MOTOR_DISTANCE_TO_CENTER 0.2f

#define CHASSIA_SPIN_SPEED 1.5f        //小陀螺旋转速度设定
#define CHASSIA_STOP_SPEED 0.0f

//chassis forward or back max speed
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 250.0f
//chassis left or right max speed
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 250.5f
//底盘小陀螺速度
#define NORMAL_MAX_CHASSIS_SPEED_WX 250.0f
#define NORMAL_MIN_CHASSIS_SPEED_WX 0.0f
//当底盘数据错误时电流值给0
#define CHASSIA_CURR_ZERO 0.0f

/*-------------------- Gimbal --------------------*/
//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)1000

//mouse sensitivity ---------------------
#define MOUSE_SENSITIVITY (0.5f)
//remote controller sensitivity ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY (100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)
//motor parameters ---------------------
//电机id
#define GIMBAL_DIRECT_YAW_ID ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)2)

//电机can口
#define GIMBAL_DIRECT_YAW_CAN ((uint8_t)2)
#define GIMBAL_DIRECT_PITCH_CAN ((uint8_t)2) 

//电机种类        
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION (1) 
#define GIMBAL_DIRECT_PITCH_DIRECTION (1)

//减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_DIRECT_YAW_MODE (0)
#define GIMBAL_DIRECT_PITCH_MODE (0)

//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH (0.3f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.5f)

//电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID (2.3731f) //云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_MID (2.1246f) //云台初始化正对齐的时候使用的yaw轴正中心量

//PID parameters ---------------------
//YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE (9.00f)
#define KI_GIMBAL_YAW_ANGLE (0.003f)
#define KD_GIMBAL_YAW_ANGLE (0.75f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.05f)
#define MAX_OUT_GIMBAL_YAW_ANGLE (20.0f)
//VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY (5000.0f)
#define KI_GIMBAL_YAW_VELOCITY (1.0f)
#define KD_GIMBAL_YAW_VELOCITY (0.1f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (10000.0f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY (30000.0f)

//PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE (4.5f)
#define KI_GIMBAL_PITCH_ANGLE (0.0001f)
#define KD_GIMBAL_PITCH_ANGLE (3.0f)
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (1.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE (10.0f)
//VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY (1200.0f)
#define KI_GIMBAL_PITCH_VELOCITY (30.0f)
#define KD_GIMBAL_PITCH_VELOCITY (100.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (10000.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY (30000.0f)

/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
#define FRIC_RADIUS 0.03f  // (m)摩擦轮半径
#define BULLET_NUM 8       // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1          // 定义枪管个数（一个枪管2个摩擦轮）

/*MOTOR paramters --------------------*/

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M2006)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机ID
#define TRIGGER_MOTOR_ID 8
#define FRIC_MOTOR_R_ID 6
#define FRIC_MOTOR_L_ID 5

//电机can口
#define TRIGGER_MOTOR_CAN 1
#define FRIC_MOTOR_R_CAN 1
#define FRIC_MOTOR_L_CAN 1

//电机std_id
#define TRIGGER_STD_ID 0x1FF
#define FRIC_STD_ID 0x1FF

//单环拨弹速度
#define TRIGGER_SPEED               (200.0f)
//摩擦轮速度
#define FRIC_R_SPEED                  (666.0f) 
#define FRIC_L_SPEED                  (-666.0f) 
#define FRIC_SPEED_LIMIT            (640.0f) 

//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191

//电机rpm 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18

/*BLOCK&REVERSE parameters------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED         5.0f
#define BLOCK_TIME                  1000
#define REVERSE_TIME                1250
#define REVERSE_SPEED               (-20.0f)  

/*PID parameters ---------------------*/

//拨弹轮电机PID速度单环
#define TRIGGER_SPEED_PID_KP (50.0f)
#define TRIGGER_SPEED_PID_KI (0.5f)
#define TRIGGER_SPEED_PID_KD (0.1f)

#define TRIGGER_SPEED_PID_MAX_OUT (10000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (1000.0f)

// 单发模式 拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (20.0f)
#define TRIGGER_ANGEL_PID_KI (0.5f)
#define TRIGGER_ANGEL_PID_KD (0.0f)

#define TRIGGER_ANGEL_PID_MAX_OUT (10000.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (1000.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (666.0f)
#define FIRC_SPEED_PID_KI (0.6f)
#define FRIC_SPEED_PID_KD (0.02f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

#endif /* INCLUDED_ROBOT_PARAM_H */
