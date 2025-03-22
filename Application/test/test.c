#include "test.h"
// 测试代码
// 请在这里添加测试代码
// 请不要在其他地方添加测试代码
// 请不要在这里添加非测试代码
#include "dmmotor.h"
#include "DJI_motor.h"
#include "remote.h"
#include "Navi_process.h"

#define K_speedcalc 6000

static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy,chassis_wz;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

static uint8_t is_init;

static RC_ctrl_t *remote;
static Navigation_Recv_s *navigation_ctrl; // 视觉控制信息

void testChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 2, // 4.5
                .Ki            = 0.1, // 0
                .Kd            = 0, // 0
                .IntegralLimit = 6000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
            .current_PID = {
                .Kp            = 0.6, // 0.4
                .Ki            = 0.05,  // 0
                .Kd            = 0,
                .IntegralLimit = 6000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 15000,
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
    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);
}

void ChassisCalc(float vx,float vy,float wz)
{
    vt_lf = vx+vy+wz;
    vt_rf = -vx+vy+wz;
    vt_lb = vx-vy+wz;
    vt_rb = -vx-vy-wz;

    DJIMotorSetRef(motor_lf, vt_lf * K_speedcalc);
    DJIMotorSetRef(motor_rf, vt_rf * K_speedcalc);
    DJIMotorSetRef(motor_lb, vt_lb * K_speedcalc);
    DJIMotorSetRef(motor_rb, vt_rb * K_speedcalc);
}

void TESTInit(void)
{
    testChassisInit();
    remote = RemoteControlInit(&huart3);
    navigation_ctrl = NavigationInit(&huart1);

}

void TESTTask(void)
{
//    if(switch_is_down(remote[TEMP].rc.switch_left))
//    {
//     chassis_vx = 0;
//     chassis_vy = 0;
//     chassis_wz = remote->rc.dial * 0.5f;
//    }
//    else if (switch_is_mid(remote[TEMP].rc.switch_left))
//    {
//     chassis_vx = remote->rc.rocker_l1;
//     chassis_vy = remote->rc.rocker_l_;
//     chassis_wz = remote->rc.dial;    
//    }
//    else if (switch_is_up(remote[TEMP].rc.switch_left))
//    {
    chassis_vx = navigation_ctrl->vx;
    chassis_vy = navigation_ctrl->vy;
    chassis_wz = navigation_ctrl->wz;
    // chassis_vx = ;
//    }    
   
   ChassisCalc(chassis_vx,chassis_vy,chassis_wz);
   NavigationSend(); 
}