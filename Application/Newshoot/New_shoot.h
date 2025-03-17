#ifndef SHOOT_H
#define SHOOT_H

/**
 * @brief 摩擦轮初始化,会被RobotInit()调用
 *
 */
void FrictionInit();

/**
 * @brief 弹盘初始化,会被RobotInit()调用
 *
 */
void LoaderInit();


/**
 * @brief 发射任务
 *
 */
void FrictionTask();
void LoaderTask();
#endif // SHOOT_H