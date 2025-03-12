#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
void PitchInit();
void YawInit();
/**
 * @brief 云台任务
 *
 */
void PitchTask();
void YawTask();

#endif // GIMBAL_H