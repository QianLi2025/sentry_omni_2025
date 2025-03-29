#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "LK_motor.h"
#include "message_center.h"
#include "general_def.h"
#include <stdio.h>
#include <math.h>
static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotor_Instance *yaw_motor;
static LKMotor_Instance  *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

// 调试数据
static float yaw_target, yaw_current, pitch_target, pitch_current, yaw_motor_angle, pitch_motor_angle;
static float pitch_cd_ms;
static float last_pitch=0;
static float pitch_target;      
static float pitch_current;     
static float pitch_motor_angle;
void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 70,
                .Ki                = 0,
                .Kd                = 2,
                .Improve           = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter ,
                .IntegralLimit     = 5000,
                .CoefB             = 0.3,
                .CoefA             = 0.2,
                .MaxOut            = 10000,
                .Derivative_LPF_RC = 0.025,
            },
            .speed_PID = {
                .Kp            = 98,
                .Ki            = 160,
                .Kd            = 0,
                .CoefB         = 0.3,
                .CoefA         = 0.2,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate,
                .IntegralLimit = 25000,
                .MaxOut        = 25000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 13,
                .Ki                = 0.1,
                .Kd                = 0.25,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
                .IntegralLimit     = 10,
                .CoefB             = 0.1,
                .CoefA             = 0.1,
                .MaxOut            = 60,
                .Derivative_LPF_RC = 0.03,
                .Output_LPF_RC     = 0.05,
            },
            .speed_PID = {
                .Kp            = 10,
                .Ki            = 0.1,
                .Kd            = 0,
                .CoefB         = 0.6,
                .CoefA         = 0.3,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
                .IntegralLimit = 500,
                .MaxOut        = 200000,
                .Output_LPF_RC = 0.001f,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
           // .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = LK4010,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor   = DJIMotorInit(&yaw_config);
    pitch_motor = LKMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    //调试用变量
    pitch_target      = gimbal_cmd_recv.pitch;
    pitch_current     = gimba_IMU_data->Pitch;
    pitch_motor_angle = pitch_motor->measure.total_angle;
    yaw_target      = gimbal_cmd_recv.yaw;
    yaw_current     = gimba_IMU_data->Yaw;
    yaw_motor_angle = yaw_motor->measure.total_angle;
    
switch (gimbal_cmd_recv.gimbal_mode) {
        
        // 停止
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            LKMotorStop(pitch_motor);
            break;
        // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE: // 后续只保留此模式
        //yaw
            DJIMotorEnable(yaw_motor);
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            DJIMotorSetRef(yaw_motor, -gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            //pitch
             LKMotorEnable(pitch_motor);
             // yaw和pitch会在robot_cmd中处理好多圈和单圈
             last_pitch =gimbal_cmd_recv.pitch;
            LKMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
            break;
        // 巡航模式
   case GIMBAL_CRUISE_MODE:
        DJIMotorEnable(yaw_motor);
        DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
        DJIMotorSetRef(yaw_motor, 200);
        LKMotorEnable(pitch_motor);
        pitch_cd_ms= DWT_GetTimeline_ms()/150.0f;
        pitch_cd_ms = 18.0f*sinf(pitch_cd_ms);
            if(last_pitch-pitch_cd_ms<20){
                last_pitch =pitch_cd_ms;
                LKMotorSetRef(pitch_motor, pitch_cd_ms);
            }
        default:
            break;
}
    // 设置反馈数据,主要是imu和yaw的ecd
        gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}