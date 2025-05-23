#include "robot.h"
#include "roboTask.h"
#include "robot_def.h"
#include "test.h"
#include "chassis.h"
#include "gimbal.h"
#include "bsp_init.h"
#include "gimball.h"
#include "shoot.h"
// #include "New_shoot.h"
// #include "New_gimbal.h"

#include "robot_cmd.h"
// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"

//#include "New_chassis.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
// #include "gimbal.h"

#include "shoot.h"

#endif
/**
 * @brief 机器人初始化
 *
 */
void RobotInit(void)
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    // BSP初始化
    BSPInit();
    // 应用层初始化
#if defined(GIMBAL_BOARD)
    GimbalCMDInit();
    //PitchInit();
    // FrictionInit();
    GimbalInit();
    ShootInit();
    //GimballInit();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisCMDInit();
     //YawInit();
    // LoaderInit();
     ChassisInit();
#endif
    // 测试代码
    // TESTInit();
    

    // rtos创建任务
    OSTaskInit();
    // 初始化完成,开启中断
    __enable_irq();
}

/**
 * @brief 机器人任务入口
 *
 */
void RobotTask()
{
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    GimbalCMDTask();
    GimbalTask();
    //FrictionTask();
    ShootTask();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisCMDTask();
    ChassisTask();
    //YawTask();
    // LoaderTask();    
#endif
// TESTTask();
}

/*  下面为测试代码,可忽略    */
