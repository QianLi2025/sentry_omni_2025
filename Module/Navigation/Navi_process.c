#include "Navi_process.h"
#include "string.h"
#include "robot_def.h"
#include "crc_ref.h"
#include "daemon.h"
#include <math.h>
#include "referee_protocol.h"
#include "bsp_dwt.h"
/*=================================移植begin=====================================*/
#include "usb_typdef.h"
#include "custom_typedef.h"
#include "CRC8_CRC16.h"

#define USB_TASK_CONTROL_TIME 1  // ms

#define USB_OFFLINE_THRESHOLD 100  // ms
#define USB_CONNECT_CNT 10

// clang-format off

#define SEND_DURATION_Debug       5   // ms
#define SEND_DURATION_Imu         5   // ms
#define SEND_DURATION_RobotStateInfo   10  // ms
#define SEND_DURATION_Event       10  // ms
#define SEND_DURATION_Pid         10  // ms
#define SEND_DURATION_AllRobotHp  10  // ms
#define SEND_DURATION_GameStatus  10  // ms
#define SEND_DURATION_RobotMotion 10  // ms
#define SEND_DURATION_GroundRobotPosition   10// ms
#define SEND_DURATION_RfidStatus   10// ms
#define SEND_DURATION_RobotStatus  10// ms
#define SEND_DURATION_JointState   10// ms
#define SEND_DURATION_Buff         10// ms

// clang-format on

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte


// #define CheckDurationAndSend(send_name)                                                  \
//     do {                                                                                 \
//         if ((HAL_GetTick() - LAST_SEND_TIME.##send_name) >= SEND_DURATION_##send_name) { \
//             LAST_SEND_TIME.##send_name = HAL_GetTick();                                  \
//             UsbSend##send_name##Data();                                                  \
//         }                                                                                \
//     } while (0)

// // Variable Declarations
// static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

// static const Imu_t * IMU;
// static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// 判断USB连接状态用到的一些变量
static bool USB_OFFLINE = true;
static float RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// // 数据发送结构体
// // clang-format off
// static SendDataDebug_s       SEND_DATA_DEBUG;
// static SendDataImu_s         SEND_DATA_IMU;
// static SendDataRobotStateInfo_s   SEND_DATA_ROBOT_STATE_INFO;
// static SendDataEvent_s       SEND_DATA_EVENT;
// static SendDataPidDebug_s    SEND_DATA_PID;
// static SendDataAllRobotHp_s  SEND_DATA_ALL_ROBOT_HP;
// static SendDataGameStatus_s  SEND_DATA_GAME_STATUS;
// static SendDataRobotMotion_s SEND_ROBOT_MOTION_DATA;
// static SendDataGroundRobotPosition_s SEND_GROUND_ROBOT_POSITION_DATA;
// static SendDataRfidStatus_s  SEND_RFID_STATUS_DATA;
// static SendDataRobotStatus_s SEND_ROBOT_STATUS_DATA;
// static SendDataJointState_s  SEND_JOINT_STATE_DATA;
// static SendDataBuff_s        SEND_BUFF_DATA;

// clang-format on

// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataPidDebug_s RECEIVE_PID_DEBUG_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

// 发送数据间隔时间
typedef struct
{
    uint32_t Debug;
    uint32_t Imu;
    uint32_t RobotStateInfo;
    uint32_t Event;
    uint32_t Pid;
    uint32_t AllRobotHp;
    uint32_t GameStatus;
    uint32_t RobotMotion;
    uint32_t GroundRobotPosition;
    uint32_t RfidStatus;
    uint32_t RobotStatus;
    uint32_t JointState;
    uint32_t Buff;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

/*================================================移植end====================================*/
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
    // static uint32_t len = USB_RECEIVE_LEN;
    uint8_t * rx_data_start_address = rx_buff;  // 接收数据包时存放于缓存区的起始位置
    uint8_t * rx_data_end_address;  // 接收数据包时存放于缓存区的结束位置
    uint8_t * sof_address = rx_buff;

    // 计算数据包的结束位置
    rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
    // 读取数据
    // USB_Receive(rx_data_start_address, &len);  // Read data into the buffer

    while (sof_address <= rx_data_end_address) {  // 解析缓冲区中的所有数据包
        // 寻找帧头位置
        while (*(sof_address) != RECEIVE_SOF && (sof_address <= rx_data_end_address)) {
            sof_address++;
        }
        // 判断是否超出接收数据范围
        if (sof_address > rx_data_end_address) {
            break;  //退出循环
        }
        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            // 检查整包CRC16校验 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, 4 + data_len + 2);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(ReceiveDataRobotCmd_s));
                    } break;
                    case PID_DEBUG_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_PID_DEBUG_DATA, sof_address, sizeof(ReceiveDataPidDebug_s));
                    } break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_VIRTUAL_RC_DATA, sof_address, sizeof(ReceiveDataVirtualRc_s));
                    } break;
                    default:
                        break;
                }
                if (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP) {
                    LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4]));
                    RECEIVE_TIME = DWT_GetTimeline_ms();
                }
            }
            sof_address += (data_len + HEADER_SIZE + 2);
        } else {  //CRC8校验失败，移动到下一个字节
            sof_address++;
        }
    }
    // 更新下一次接收数据的起始位置
    if (sof_address > rx_data_start_address + USB_RECEIVE_LEN) {
        // 缓冲区中没有剩余数据，下次接收数据的起始位置为缓冲区的起始位置
        rx_data_start_address = nav_recv_buff[0];
    } else {
        uint16_t remaining_data_len = USB_RECEIVE_LEN - (sof_address - rx_data_start_address);
        // 缓冲区中有剩余数据，下次接收数据的起始位置为缓冲区中剩余数据的起始位置
        rx_data_start_address = nav_recv_buff[0] + remaining_data_len;
        // 将剩余数据移到缓冲区的起始位置
        memcpy(nav_recv_buff[0], sof_address, remaining_data_len);
    }

    recv->vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    recv->vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    recv->wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;
}
// /**
//  * @brief 处理视觉传入的数据
//  *
//  * @param recv
//  * @param rx_buff
//  */
// static void NaviRecvProcess(Navigation_Recv_s *recv, uint8_t *rx_buff)
// {
    
//     recv->naving = rx_buff[1];
//     recv->poing = rx_buff[2];
//     memcpy(&recv->nav_x, &rx_buff[3], 4);
//     memcpy(&recv->nav_y, &rx_buff[7], 4);
//     memcpy(&recv->sentry_decision, &rx_buff[11], 4);
// //  JudgeSend(&recv->sentry_decision,Datacmd_Decision);
//     memcpy(&recv->yaw_target, &rx_buff[15], 4);
    
//     recv->R_tracking = rx_buff[19];
//     recv->R_shoot = rx_buff[20];
//     memcpy(&recv->yaw_From_R, &rx_buff[21], 4); //485传来的yaw的目标值，需要发送到上板
//     memcpy(&recv->R_yaw, &rx_buff[25], 4);
//     memcpy(&recv->R_pitch, &rx_buff[29], 4);
//     recv->target_shijue = rx_buff[33];
    
//     memcpy(&recv->Flag_turn, &rx_buff[34], 1);
//     memcpy(&recv->Flag_headforward, &rx_buff[35], 1);
//    // /* 使用memcpy接收浮点型小数 */
//     // recv->is_tracking = rx_buff[1];

//     // recv->is_shooting = rx_buff[2];
//     // memcpy(&recv->yaw, &rx_buff[3], 4);
//     // memcpy(&recv->pitch, &rx_buff[7], 4);
//     // memcpy(&recv->distance, &rx_buff[11], 4);

//     // /* 视觉数据处理 */
//     // float yaw_total  = vision_instance->send_data->yaw; // 保存当前总角度
//     // float yaw_target = recv->yaw;
//     // recv->yaw        = AdjustNearestAngle(yaw_total, yaw_target);
// }

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

        
    // UNUSED(var); // 仅为了消除警告
    // if (nav_recv_buff[0] == navigation_instance->recv_data->header) {
    //     // 读取视觉数据
    //     /* 接收校验位 */
    //     memcpy(&navigation_instance->recv_data->checksum, &nav_recv_buff[NAVIGATION_RECV_SIZE - 2], 2);
    //     if (navigation_instance->recv_data->checksum == Get_CRC16_Check_Sum(nav_recv_buff, NAVIGATION_RECV_SIZE - 2, CRC_INIT)) {
            DaemonReload(navigation_daemon_instance);
            NaviRecvProcess(navigation_instance->recv_data, nav_recv_buff);
        // } else {
        //     memset(navigation_instance->recv_data, 0, sizeof(Navigation_Recv_s));
        // }
 //   }
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