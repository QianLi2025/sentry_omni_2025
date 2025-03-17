/**
 * @file miniPC_process.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   用于处理miniPC的数据，包括解析和发送
 * @version 0.1
 * @date 2024-01-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MINIPC_PROCESS_H
#define MINIPC_PROCESS_H

#include "stdint.h"
#include "bsp_usart.h"

#define NAVIGATION_RECV_HEADER 0xA5u // 视觉接收数据帧头
#define NAVIGATION_SEND_HEADER 0x5Au // 视觉发送数据帧头
#define NAVIGATION_SEND_TAIL   0xAAu // 视觉发送数据帧尾

#define NAVIGATION_RECV_SIZE   38u // 当前为固定值,12字节 header 1 +date 35 +checksum 2
#define NAVIGATION_SEND_SIZE   63u // header 1 + date 59 + checksum 2 + ending 1

// #pragma pack(1) // 1字节对齐

/* 是否追踪 */
typedef enum {
    VISION_NO_SHOOTING = 0u,
    VISION_SHOOTING    = 1u,
} VISION_SHOOTING_e;

/* 是否重置追踪 */
typedef enum {
    VISION_RESET_TRACKER_NO  = 0u,
    VISION_RESET_TRACKER_YES = 1u,
} VISION_RESET_TRACKER_e;

/* 目标ID */
typedef enum {
    VISION_OUTPOST = 0u,
    VISION_GUARD   = 6u,
    VISION_BASE    = 7u,
} VISION_ID_e;

/* 装甲板数量 */
typedef enum {
    VISION_ARMORS_NUM_BALANCE = 2u,
    VISION_ARMORS_NUM_OUTPOST = 3u,
    VISION_ARMORS_NUM_NORMAL  = 4u,
} VISION_ARMORS_NUM_e;

/* 敌方装甲板颜色 */
typedef enum {
    VISION_DETECT_COLOR_RED  = 0u,
    VISION_DETECT_COLOR_BLUE = 1u,
} VISION_DETECT_COLOR_e;

typedef enum {
    COLOR_NONE = 0,
    COLOR_BLUE = 1,
    COLOR_RED  = 2,

} Self_Color_e;

/* 视觉通信初始化接收结构体 */
typedef struct
{
    uint8_t header; // 头帧校验位
} Navigation_Recv_Init_Config_s;

/* 视觉通信初始化发送结构体 */
typedef struct
{
    uint8_t header;       // 头帧校验位
    // uint8_t detect_color; // 0-red 1-blue
    uint8_t tail;         // 尾帧校验位
} Navigation_Send_Init_Config_s;

/* 视觉实例初始化配置结构体 */
typedef struct
{
    Navigation_Recv_Init_Config_s recv_config; // 接收数据结构体
    Navigation_Send_Init_Config_s send_config; // 发送数据结构体
    USART_Init_Config_s usart_config;      // 串口实例结构体
} Navigation_Init_Config_s;

/* minipc -> stm32 (接收结构体) */
#pragma pack(1) // 1字节对齐
typedef struct
{
    uint8_t header;
    uint8_t naving;
    uint8_t poing;
    float yaw_target;
    float nav_x;
    float nav_y;
    uint32_t sentry_decision;

    uint8_t R_tracking;
    uint8_t R_shoot;
    float yaw_From_R;
    float R_yaw;
    float R_pitch;
    uint8_t target_shijue; //击打目标，从白头发给黑头 
    uint8_t intention;
    uint16_t start_position_x; 
    uint16_t start_position_y; 
    int8_t delta_x[49]; 
    int8_t delta_y[49];
    uint8_t Flag_turn;
    uint8_t Flag_headforward;
    uint16_t checksum;
} Navigation_Recv_s;

/* stm32 -> minipc (发送结构体) */


typedef struct
{
    uint8_t header;
    uint8_t Flag_progress;
    uint8_t color;
    uint16_t projectile_allowance_17mm;
    uint16_t remaining_gold_coin;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_num; 
    uint16_t red_7_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_7_HP;
    uint16_t blue_outpost_HP;  
    uint16_t blue_base_HP;
    uint16_t checksum; 
    float R_yaw;	
    float R_pitch; 
    double yaw12;
    float tar_pos_x;
    float tar_pos_y;
    uint8_t cmd_key;
    uint8_t ending;
    float bullet_speed;
    uint8_t Flag_start;
    uint8_t Flag_off_war;
    float R_yaw_speed;
    uint8_t is_rune;
    uint8_t is_reset;
    float R_roll;  
    uint8_t tail;
} Navigation_Send_s;
#pragma pack() // 取消1字节对齐
/* 视觉通信模块实例 */
typedef struct
{
    Navigation_Recv_s *recv_data; // 接收数据结构体指针
    Navigation_Send_s *send_data; // 发送数据结构体指针
    USART_Instance *usart;        // 串口实例指针
} Navigation_Instance;
// #pragma pack() // 取消1字节对齐

/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Navigation_Recv_s *NavigationRecvRegister(Navigation_Recv_Init_Config_s *recv_config);

/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Navigation_Send_s *NavigationSendRegister(Navigation_Send_Init_Config_s *send_config);

/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Navigation_Recv_s *NavigationInit(UART_HandleTypeDef *video_usart_handle);

/**
 * @brief 发送函数
 *
 *
 */
void NavigationSend();

// /**
//  * @brief
//  *
//  * @param yaw
//  * @param pitch
//  * @param roll
//  * @param bullet_speed
//  */
// void VisionSetAltitude(float yaw, float pitch, float roll, float bullet_speed, float yaw_speed);

// void VisionSetEnergy(uint8_t is_energy_mode);

// /**
//  * @brief 设置颜色
//  *
//  * @param detect_color 5-红色，6-蓝色
//  */
// void VisionSetDetectColor(Self_Color_e self_color);

// void VisionSetReset(uint8_t is_reset);

#endif // MINIPC_PROCESS_H