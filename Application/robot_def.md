# robot_def
<p align='right'>qianli@gmail.com</p>
这是关于哨兵机器人的定义说明

> todo:能否简化大量的预编译条件？当前的预编译条件让人眼花

## 机器人接线定义
 **云台板** : 
 * can1: 云台pitch （LK-MG4010E-i10 id：4） 
 * can2: 摩擦轮 (DJI-M3508 id: 1,2)
 * usart1(丝印：UART2)：板间通讯（下c板）
 * usart6(丝印：UART1)：图传
 * usart3(丝印：DBUS):  DR16 

 **底盘板** : 
 * can1: 云台yaw（DJI-GM6020 id：1）; 拨盘（DJI-2006 id：1）
 * can2: 底盘轮（DJI-M3508 id：1,2,3,4）超电
 * usart1(丝印：UART2)：板间通讯（上c板）
 * usart6(丝印：UART1)：电源管理


## 机器人任务逻辑
> **底盘C板** 
> * 底盘任务
> * 云台Yaw轴任务
> * 拨盘电机任务
> * 导航与决策通讯（USB）


> **云台C板** 
> * 摩擦轮任务
> * 云台Pitch轴任务
> * 自瞄通讯（USB）
> * CMD任务

* 底盘C板应向云台发送的数据有：裁判系统数据、导航决策通讯数据、

* 云台C板应向底盘发送的数据有：底盘控制指令、云台Yaw控制指令、云台陀螺仪数据、拨盘控制指令，视觉yaw控制指令