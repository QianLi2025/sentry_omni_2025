
#include "robot_def.h"
#include "dji_motor.h"
#include "LK_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "New_gimbal.h"
#include <stdio.h>
#include <math.h>
static attitude_t *gimba_IMU_data; // 云台IMU数据

static LKMotor_Instance  *pitch_motor;
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static float pitch_cd_ms;
static int pitch_timer =0;
static float last_pitch=0;
static float pitch_target;      
static float pitch_current;     
static float pitch_motor_angle;

static DJIMotor_Instance *yaw_motor;

static float delt_time_yaw,current,last=0;
void YawInit (){
   
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 60,
                .Ki                = 0,
                .Kd                = 0.5,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .IntegralLimit     = 10,
                .CoefB             = 0.3,
                .CoefA             = 0.2,
                .MaxOut            = 5000,//4000
                .Derivative_LPF_RC = 0.25,
            },
            .speed_PID = {
                .Kp            = 33,
                .Ki            = 0.01,
                .Kd            = 0,
                .CoefB         = 0.3,
                .CoefA         = 0.2,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate,
                .IntegralLimit = 5000,
                .MaxOut        = 30000,//10000
            },
             .other_angle_feedback_ptr = &gimbal_cmd_recv.gimbal_imu_data_yaw.YawTotalAngle,
            // // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
             .other_speed_feedback_ptr = &gimbal_cmd_recv.gimbal_imu_data_yaw.Gyro,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020};
        yaw_motor   = DJIMotorInit(&yaw_config);

        gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
        gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
        
        // osDelay(20);

        // float target_yaw = 0;
        // target_yaw = yaw_motor->measure.total_angle;
        // DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
        // DJIMotorSetRef(yaw_motor, target_yaw);

}
void YawTask(){
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    //TODO：YAW运行在底盘中，需要从CMD获取云台陀螺仪

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode) {
        
        // 停止
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            break;
        // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE: // 后续只保留此模式
            DJIMotorEnable(yaw_motor);
         yaw_motor->motor_controller.speed_PID.Kp =33;
            yaw_motor->motor_controller.speed_PID.Ki =0;
            yaw_motor->motor_controller.angle_PID.Kp =60;
            yaw_motor->motor_controller.angle_PID.Kd =0.5;
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);

            //  if(gimbal_cmd_recv.gimbal_imu_data_yaw.YawTotalAngle+gimbal_cmd_recv.yaw>90){
            //     gimbal_cmd_recv.yaw=-gimbal_cmd_recv.gimbal_imu_data_yaw.YawTotalAngle;
            // }
            DJIMotorSetRef(yaw_motor, -gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            break;
        // 巡航模式
        case GIMBAL_CRUISE_MODE:
        DJIMotorEnable(yaw_motor);

        DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
            yaw_motor->motor_controller.speed_PID.Kp =50;
            yaw_motor->motor_controller.speed_PID.Ki =2;           
            DJIMotorSetRef(yaw_motor, 200);
        default:
            break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...
    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
    current = DWT_GetTimeline_ms();
    delt_time_yaw = current - last;
    last = current;
}

# if defined(GIMBAL_BOARD)
void PitchInit(){
    gimba_IMU_data = INS_Init();
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 13,//0.5,
                .Ki                =0.1,//0.07,
                .Kd                = 0.25,//0.001,
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
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch, //上正下负      
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            //.other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),//上正下负
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
    pitch_motor = LKMotorInit(&pitch_config);
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    LKMotorEnable(pitch_motor);
}

void PitchTask(){
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    pitch_target      = gimbal_cmd_recv.pitch;
    pitch_current     = gimba_IMU_data->Pitch;
    pitch_current     = gimba_IMU_data->Pitch;
    pitch_motor_angle = pitch_motor->measure.total_angle;
    // gimbal_cmd_recv.gimbal_mode=GIMBAL_GYRO_MODE;
    switch (gimbal_cmd_recv.gimbal_mode) {
        // 停止
        case GIMBAL_ZERO_FORCE:
            LKMotorStop(pitch_motor);
            break;
        // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE: // 后续只保留此模式
            LKMotorEnable(pitch_motor);
             // yaw和pitch会在robot_cmd中处理好多圈和单圈
             last_pitch =gimbal_cmd_recv.pitch;
            LKMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
            
            break;
        // 巡航模式
        case GIMBAL_CRUISE_MODE:
            LKMotorEnable(pitch_motor);

            pitch_cd_ms= DWT_GetTimeline_ms()/300.0f;
            pitch_cd_ms = 18.0f*sinf(pitch_cd_ms);
            if(last_pitch-pitch_cd_ms<20){
                last_pitch =pitch_cd_ms;
                LKMotorSetRef(pitch_motor, pitch_cd_ms);
            }
            //注：陀螺仪对应G[0]Pitch,G[1]Roll,G[2]YAW 上正下负
            //Pitch 最高25 最低-20
        default:
            break;
}

    //
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
#endif
