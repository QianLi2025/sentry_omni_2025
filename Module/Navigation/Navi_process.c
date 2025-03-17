#include "Navi_process.h"
#include "string.h"
#include "robot_def.h"
#include "crc_ref.h"
#include "daemon.h"
#include <math.h>
#include "referee_protocol.h"

static Navigation_Instance *navigation_instance; //用于和导航通信的串口实例
static uint8_t *nav_recv_buff __attribute__((unused));
static Daemon_Instance *navigation_daemon_instance;
// 全局变量区
extern uint16_t CRC_INIT;

// 标准化角度到0到360度范围内
// static float StandardizeAngle(float angle)
// {
//     float mod_angle = fmod(angle, 360.f);
//     if (mod_angle < 0) {
//         mod_angle += 360.0;
//     }
//     return mod_angle;
// }

// 计算并返回新的 total_angle，使其接近目标角度
// static float AdjustNearestAngle(float total_angle, float target_angle)
// {
//     // 标准化当前角度
//     float standard_current_angle = StandardizeAngle(total_angle);
//     // 计算角度差
//     float delta_angle = target_angle - standard_current_angle;
//     // 调整角度差到 -180 到 180 范围内
//     if (delta_angle > 180.f) {
//         delta_angle -= 360.f;
//     } else if (delta_angle < -180.f) {
//         delta_angle += 360.f;
//     }
//     // 计算新的目标 totalangle
//     return total_angle + delta_angle;
// }

/**
 * @brief 处理视觉传入的数据
 *
 * @param recv
 * @param rx_buff
 */
static void NaviRecvProcess(Navigation_Recv_s *recv, uint8_t *rx_buff)
{
    
    recv->naving = rx_buff[1];
    recv->poing = rx_buff[2];
    memcpy(&recv->nav_x, &rx_buff[3], 4);
    memcpy(&recv->nav_y, &rx_buff[7], 4);
    memcpy(&recv->sentry_decision, &rx_buff[11], 4);
//  JudgeSend(&recv->sentry_decision,Datacmd_Decision);
    memcpy(&recv->yaw_target, &rx_buff[15], 4);
    
    recv->R_tracking = rx_buff[19];
    recv->R_shoot = rx_buff[20];
    memcpy(&recv->yaw_From_R, &rx_buff[21], 4); //485传来的yaw的目标值，需要发送到上板
    memcpy(&recv->R_yaw, &rx_buff[25], 4);
    memcpy(&recv->R_pitch, &rx_buff[29], 4);
    recv->target_shijue = rx_buff[33];
    
    memcpy(&recv->Flag_turn, &rx_buff[34], 1);
    memcpy(&recv->Flag_headforward, &rx_buff[35], 1);
   // /* 使用memcpy接收浮点型小数 */
    // recv->is_tracking = rx_buff[1];

    // recv->is_shooting = rx_buff[2];
    // memcpy(&recv->yaw, &rx_buff[3], 4);
    // memcpy(&recv->pitch, &rx_buff[7], 4);
    // memcpy(&recv->distance, &rx_buff[11], 4);

    // /* 视觉数据处理 */
    // float yaw_total  = vision_instance->send_data->yaw; // 保存当前总角度
    // float yaw_target = recv->yaw;
    // recv->yaw        = AdjustNearestAngle(yaw_total, yaw_target);
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void NavigationOfflineCallback(void *id)
{
    // memset(vision_instance->recv_data + 1, 0, sizeof(Vision_Recv_s) - 1);
#ifdef VISION_USE_UART
    USARTServiceInit(vision_instance->usart);
#endif // !VISION_USE_UART
}

// /**
//  * @brief
//  *
//  * @param yaw
//  * @param pitch
//  * @param roll
//  * @param bullet_speed
//  */
// void VisionSetAltitude(float yaw, float pitch, float roll, float bullet_speed, float yaw_speed)
// {
//     vision_instance->send_data->yaw       = yaw;
//     vision_instance->send_data->pitch     = pitch;
//     vision_instance->send_data->roll      = roll;
//     vision_instance->send_data->yaw_speed = yaw_speed;
//     if (bullet_speed > 0) {
//         vision_instance->send_data->bullet_speed = bullet_speed;
//     } else {
//         vision_instance->send_data->bullet_speed = 25;
//     }
// }

// /**
//  * @brief 设置是否击打能量机关
//  *
//  * @param is_energy_mode 0-默认瞄准装甲板，1-瞄准能量机关
//  */
// void VisionSetEnergy(uint8_t is_energy_mode)
// {
//     vision_instance->send_data->is_energy_mode = is_energy_mode;
// }

// /**
//  * @brief 设置颜色
//  *
//  * @param detect_color 5-红色，6-蓝色
//  */
// void VisionSetDetectColor(Self_Color_e self_color)
// {
//     uint8_t detect_color = 0;
//     if (self_color == COLOR_BLUE) {
//         detect_color = 5; // 我方是蓝色，敌方是红色
//     }
//     if (self_color == COLOR_RED) {
//         detect_color = 6; // 我方是红色，敌方是蓝色
//     }
//     vision_instance->send_data->detect_color = detect_color;
// }

// void VisionSetReset(uint8_t is_reset)
// {
//     vision_instance->send_data->is_reset = is_reset;
// }
void NavigationSetRefreeDate()
{

}

void NavigationSetGameDate()
{

}
/**
 * @brief 发送数据处理函数
 *
 * @param send 待发送数据
 * @param tx_buff 发送缓冲区
 *
 */
static void NaviSendProcess(Navigation_Send_s *send, uint8_t *tx_buff)
{
    int tarforwrd = 0;
    // /* 发送帧头，目标颜色，是否重置等数据 */
    // tx_buff[0] = send->header;
    // tx_buff[1] = send->is_energy_mode;
    // tx_buff[2] = send->detect_color;
    // tx_buff[3] = send->is_reset;
    // // tx_buff[3] = send->reset_tracker;
    // // tx_buff[4] = 1;

    // /* 使用memcpy发送浮点型小数 */
    // memcpy(&tx_buff[4], &send->roll, 4);
    // memcpy(&tx_buff[8], &send->yaw, 4);
    // memcpy(&tx_buff[12], &send->pitch, 4);
    // memcpy(&tx_buff[16], &send->bullet_speed, 4);
    // memcpy(&tx_buff[20], &send->yaw_speed, 4);
	tx_buff[0] = send->header;
	tx_buff[1] = send->Flag_progress;
	tx_buff[2] = send->color;
	memcpy(&tx_buff[3], &send->projectile_allowance_17mm, 2);
	memcpy(&tx_buff[5], &send->remaining_gold_coin, 2);
	memcpy(&tx_buff[7], &send->supply_robot_id, 1);
	memcpy(&tx_buff[8], &send->supply_projectile_num, 1);
	memcpy(&tx_buff[9], &send->red_7_HP, 2);
	memcpy(&tx_buff[11], &send->red_outpost_HP, 2);
	memcpy(&tx_buff[13], &send->red_base_HP, 2);
	memcpy(&tx_buff[15], &send->blue_7_HP, 2);
	memcpy(&tx_buff[17], &send->blue_outpost_HP, 2);
	memcpy(&tx_buff[19], &send->blue_base_HP, 2);
	memcpy(&tx_buff[21], &send->yaw12, 8);
	memcpy(&tx_buff[29], &send->R_yaw, 4);
	memcpy(&tx_buff[33], &send->R_pitch, 4);
	memcpy(&tx_buff[37], &send->tar_pos_x, 4);
	memcpy(&tx_buff[41], &send->tar_pos_y, 4);
	memcpy(&tx_buff[45], &send->cmd_key, 1);
	memcpy(&tx_buff[46], &send->bullet_speed, 4);
	memcpy(&tx_buff[50], &send->Flag_start, 1);
	memcpy(&tx_buff[51], &send->Flag_off_war, 1);
	memcpy(&tx_buff[52], &send->R_yaw_speed, 4);
	memcpy(&tx_buff[56], &tarforwrd,4); //前进角度
//	memcpy(&tx_buff[60], &Tx_nav.is_rune,1);
//	memcpy(&tx_buff[61], &Tx_nav.is_reset,1);
//	memcpy(&tx_buff[62], &Tx_nav.R_roll,4);

    /*发送校验位*/
	send->checksum = Get_CRC16_Check_Sum(tx_buff,  NAVIGATION_SEND_SIZE - 3u, CRC_INIT); //size 60+3
	memcpy(&tx_buff[NAVIGATION_SEND_SIZE - 3u], &send->checksum, 2);
	tx_buff[NAVIGATION_SEND_SIZE - 1u] = send->ending;
	
}

/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * 
 * 
 * @param recv_config
 * @return Vision_Recv_s*
 */
Navigation_Recv_s *NavigationRecvRegister(Navigation_Recv_Init_Config_s *recv_config)
{
    Navigation_Recv_s *recv_data = (Navigation_Recv_s *)malloc(sizeof(Navigation_Recv_s));
    memset(recv_data, 0, sizeof(Navigation_Recv_s));

    recv_data->header = recv_config->header;

    return recv_data;
}

/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Navigation_Send_s *NavigationSendRegister(Navigation_Send_Init_Config_s *send_config)
{
    Navigation_Send_s *send_data = (Navigation_Send_s *)malloc(sizeof(Navigation_Send_s));
    memset(send_data, 0, sizeof(Navigation_Send_s));

    send_data->header       = send_config->header;
    // send_data->detect_color = send_config->detect_color;
    send_data->tail         = send_config->tail;
    return send_data;
}

#ifdef VISION_USE_UART

#endif

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
/**
 * @brief 回调函数，确认帧头后用于解析视觉数据
 *
 */
static void NaviDecodeVision(uint16_t var)
{

        
    UNUSED(var); // 仅为了消除警告
    if (nav_recv_buff[0] == navigation_instance->recv_data->header) {
        // 读取视觉数据
        /* 接收校验位 */
        memcpy(&navigation_instance->recv_data->checksum, &nav_recv_buff[NAVIGATION_RECV_SIZE - 2], 2);
        if (navigation_instance->recv_data->checksum == Get_CRC16_Check_Sum(nav_recv_buff, NAVIGATION_RECV_SIZE - 2, CRC_INIT)) {
            DaemonReload(navigation_daemon_instance);
            NaviRecvProcess(navigation_instance->recv_data, nav_recv_buff);
        } else {
            memset(navigation_instance->recv_data, 0, sizeof(Navigation_Recv_s));
        }
    }
}
/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Navigation_Recv_s *NavigationInit(UART_HandleTypeDef *navi_usart_handle)
{
    UNUSED(navi_usart_handle); // 仅为了消除警告
    navigation_instance = (Navigation_Instance *)malloc(sizeof(Navigation_Instance));
    memset(navigation_instance, 0, sizeof(Navigation_Instance));
    Navigation_Recv_Init_Config_s recv_config = {
        .header = NAVIGATION_RECV_HEADER,
    };

    USB_Init_Config_s conf     = {.rx_cbk = NaviDecodeVision};
    nav_recv_buff              = USBInit(conf);
    recv_config.header         = NAVIGATION_RECV_HEADER;
    navigation_instance->recv_data = NavigationRecvRegister(&recv_config);

    Navigation_Send_Init_Config_s send_config = {
        .header       = NAVIGATION_SEND_HEADER,
        // .detect_color = VISION_DETECT_COLOR_RED,
        .tail         = NAVIGATION_SEND_TAIL,
    };
    navigation_instance->send_data = NavigationSendRegister(&send_config);
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback     = NavigationOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id     = NULL,
        .reload_count = 5, // 50ms
    };
    navigation_daemon_instance = DaemonRegister(&daemon_conf);
    return navigation_instance->recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void NavigationSend()
{
    static uint8_t send_buff[NAVIGATION_SEND_SIZE];
    NaviSendProcess(navigation_instance->send_data, send_buff);
    USBTransmit(send_buff, NAVIGATION_SEND_SIZE);
}

#endif