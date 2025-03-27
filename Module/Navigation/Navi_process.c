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
#include "robot_param_omni_infantry.h"
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

// 判断USB连接状态用到的一些变量
static bool USB_OFFLINE = true;
static float RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;



// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataPidDebug_s RECEIVE_PID_DEBUG_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

/*================================================移植end====================================*/
static Navigation_Instance *navigation_instance; //用于和导航通信的串口实例
static uint8_t *nav_recv_buff __attribute__((unused));
static Daemon_Instance *navigation_daemon_instance;
// 全局变量区
extern uint16_t CRC_INIT;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/
static void UsbSendData(void);
static void UsbInit(void);
void USB_Transmit(uint8_t *buffer, uint16_t len);

/*=================================移植发送begin==================================*/


#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - LAST_SEND_TIME.send_name) >= SEND_DURATION_##send_name) { \
            LAST_SEND_TIME.send_name = HAL_GetTick();                                  \
            UsbSend##send_name##Data();                                                  \
        }                                                                                \
    } while (0)

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// 发送数据间隔时间
typedef struct
{
    uint32_t Debug;
    uint32_t RobotStateInfo;
    uint32_t Imu;
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
// // 数据发送结构体
// // clang-format off
static SendDataDebug_s       SEND_DATA_DEBUG;
static SendDataImu_s         SEND_DATA_IMU;
static SendDataRobotStateInfo_s   SEND_DATA_ROBOT_STATE_INFO;
static SendDataEvent_s       SEND_DATA_EVENT;
static SendDataPidDebug_s    SEND_DATA_PID;
static SendDataAllRobotHp_s  SEND_DATA_ALL_ROBOT_HP;
static SendDataGameStatus_s  SEND_DATA_GAME_STATUS;
static SendDataRobotMotion_s SEND_ROBOT_MOTION_DATA;
static SendDataGroundRobotPosition_s SEND_GROUND_ROBOT_POSITION_DATA;
static SendDataRfidStatus_s  SEND_RFID_STATUS_DATA;
static SendDataRobotStatus_s SEND_ROBOT_STATUS_DATA;
static SendDataJointState_s  SEND_JOINT_STATE_DATA;
static SendDataBuff_s        SEND_BUFF_DATA;

// clang-format on

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

static void UsbSendDebugData(void);
static void UsbSendImuData(void);
static void UsbSendRobotStateInfoData(void);
static void UsbSendEventData(void);
// static void UsbSendPIdDebugData(void);
static void UsbSendAllRobotHpData(void);
static void UsbSendGameStatusData(void);
static void UsbSendRobotMotionData(void);
static void UsbSendGroundRobotPositionData(void);
static void UsbSendRfidStatusData(void);
static void UsbSendRobotStatusData(void);
static void UsbSendJointStateData(void);
static void UsbSendBuffData(void);






/*=============================移植接收begin============================================*/
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
    recv->spiral_mode = RECEIVE_ROBOT_CMD_DATA.data.chassis.spiral_mode;
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


void NavigationSetRefreeDate()
{

}

void NavigationSetGameDate()
{

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
    UsbInit();
    return navigation_instance->recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void NavigationSend(referee_info_t *referee_data)
{
    // static uint8_t send_buff[NAVIGATION_SEND_SIZE];
    // NaviSendProcess(navigation_instance->send_data, send_buff);
    //  USBTransmit(send_buff, NAVIGATION_SEND_SIZE);

    uint32_t event_data = referee_data->EventData.event_type;
    SEND_DATA_EVENT.data.non_overlapping_supply_zone = (event_data >> 0) & 0x01;
    SEND_DATA_EVENT.data.overlapping_supply_zone = (event_data >> 1) & 0x01;
    SEND_DATA_EVENT.data.supply_zone = (event_data >> 2) & 0x01;
    SEND_DATA_EVENT.data.small_energy = (event_data >> 3) & 0x01;
    SEND_DATA_EVENT.data.big_energy = (event_data >> 4) & 0x01;
    SEND_DATA_EVENT.data.central_highland = (event_data >> 5) & 0x03;
    SEND_DATA_EVENT.data.trapezoidal_highland = (event_data >> 7) & 0x03;
    SEND_DATA_EVENT.data.center_gain_zone = (event_data >> 21) & 0x03;
    
    SEND_DATA_GAME_STATUS.data.game_progress = referee_data->GameState.game_progress;
    SEND_DATA_GAME_STATUS.data.stage_remain_time = referee_data->GameState.stage_remain_time;

    SEND_ROBOT_STATUS_DATA.data.robot_id = referee_data->GameRobotState.robot_id;
    SEND_ROBOT_STATUS_DATA.data.robot_level = referee_data->GameRobotState.robot_level;
    SEND_ROBOT_STATUS_DATA.data.current_hp = referee_data->GameRobotState.current_HP;
    SEND_ROBOT_STATUS_DATA.data.maximum_hp = referee_data->GameRobotState.maximum_HP;
    SEND_ROBOT_STATUS_DATA.data.shooter_barrel_cooling_value = referee_data->GameRobotState.shooter_barrel_cooling_value;
    SEND_ROBOT_STATUS_DATA.data.shooter_barrel_heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
    SEND_ROBOT_STATUS_DATA.data.shooter_17mm_1_barrel_heat = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    SEND_ROBOT_STATUS_DATA.data.robot_pos_x = referee_data->GameRobotPos.x;
    SEND_ROBOT_STATUS_DATA.data.robot_pos_y = referee_data->GameRobotPos.y;
    SEND_ROBOT_STATUS_DATA.data.robot_pos_angle = referee_data->GameRobotPos.angle;
    SEND_ROBOT_STATUS_DATA.data.armor_id = referee_data->RobotHurt.armor_id;
    SEND_ROBOT_STATUS_DATA.data.hp_deduction_reason = referee_data->RobotHurt.hurt_type;
    SEND_ROBOT_STATUS_DATA.data.projectile_allowance_17mm = referee_data->ProjectileAllowance.projectile_allowance_17mm;
    SEND_ROBOT_STATUS_DATA.data.remaining_gold_coin = referee_data->ProjectileAllowance.remaining_gold_coin;


    UsbSendData();
}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    // 发送Debug数据
    CheckDurationAndSend(Debug);
    // 发送Imu数据
    CheckDurationAndSend(Imu);
    // 发送RobotStateInfo数据
    CheckDurationAndSend(RobotStateInfo);
    // 发送Event数据
    CheckDurationAndSend(Event);
    // 发送PidDebug数据
    // CheckDurationAndSend(Pid);
    // 发送AllRobotHp数据
    CheckDurationAndSend(AllRobotHp); 
    // 发送GameStatus数据
    CheckDurationAndSend(GameStatus);
    // 发送RobotMotion数据
    CheckDurationAndSend(RobotMotion);
    // 发送GroundRobotPosition数据
    CheckDurationAndSend(GroundRobotPosition);
    // 发送RfidStatus数据
    CheckDurationAndSend(RfidStatus);
    // 发送RobotStatus数据
    CheckDurationAndSend(RobotStatus);
    // 发送JointState数据
    CheckDurationAndSend(JointState);
    // 发送Buff数据
    CheckDurationAndSend(Buff);
}

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    // 订阅数据
    // IMU = Subscribe(IMU_NAME);                             // 获取IMU数据指针
    // FDB_SPEED_VECTOR = Subscribe(CHASSIS_FDB_SPEED_NAME);  // 获取底盘速度矢量指针

    // 数据置零
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_PID_DEBUG_DATA, 0, sizeof(ReceiveDataPidDebug_s));
    memset(&RECEIVE_VIRTUAL_RC_DATA, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));

    /*******************************************************************************/
    /* Serial                                                                     */
    /*******************************************************************************/
    
    // 1.初始化调试数据包
    // 帧头部分
    SEND_DATA_DEBUG.frame_header.sof = SEND_SOF;
    SEND_DATA_DEBUG.frame_header.len = (uint8_t)(sizeof(SendDataDebug_s) - 6);
    SEND_DATA_DEBUG.frame_header.id = DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_DEBUG.frame_header), sizeof(SEND_DATA_DEBUG.frame_header));
    // 数据部分
    for (uint8_t i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        SEND_DATA_DEBUG.packages[i].type = 1;
        SEND_DATA_DEBUG.packages[i].name[0] = '\0';
    }
    
    // 2.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));
    /*******************************************************************************/
    /* Referee                                                                     */
    /*******************************************************************************/
    
    // 3.初始化机器人信息数据包
    // 帧头部分
    SEND_DATA_ROBOT_STATE_INFO.frame_header.sof = SEND_SOF;
    SEND_DATA_ROBOT_STATE_INFO.frame_header.len = (uint8_t)(sizeof(SendDataRobotStateInfo_s) - 6);
    SEND_DATA_ROBOT_STATE_INFO.frame_header.id = ROBOT_STATE_INFO_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ROBOT_STATE_INFO.frame_header), sizeof(SEND_DATA_ROBOT_STATE_INFO.frame_header));
    // 数据部分
    SEND_DATA_ROBOT_STATE_INFO.data.type.chassis = CHASSIS_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.gimbal = GIMBAL_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.shoot = SHOOT_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.arm = MECHANICAL_ARM_TYPE;
    
    // 4.初始化事件数据包
    SEND_DATA_EVENT.frame_header.sof = SEND_SOF;
    SEND_DATA_EVENT.frame_header.len = (uint8_t)(sizeof(SendDataEvent_s) - 6);
    SEND_DATA_EVENT.frame_header.id = EVENT_DATA_SEND_ID;
    append_CRC8_check_sum
        ((uint8_t *)(&SEND_DATA_EVENT.frame_header), sizeof(SEND_DATA_EVENT.frame_header));

    // 5.初始化pid调参数据
    SEND_DATA_PID.frame_header.sof = SEND_SOF;
    SEND_DATA_PID.frame_header.len = (uint8_t)(sizeof(SendDataPidDebug_s) - 6);
    SEND_DATA_PID.frame_header.id = PID_DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_PID.frame_header), sizeof(SEND_DATA_PID.frame_header));

    // 6.初始化所有机器人血量数据
    SEND_DATA_ALL_ROBOT_HP.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOT_HP.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotHp_s) - 6);
    SEND_DATA_ALL_ROBOT_HP.frame_header.id = ALL_ROBOT_HP_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALL_ROBOT_HP.frame_header),
        sizeof(SEND_DATA_ALL_ROBOT_HP.frame_header));

    // 7.初始化比赛状态数据
    SEND_DATA_GAME_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_GAME_STATUS.frame_header.len = (uint8_t)(sizeof(SendDataGameStatus_s) - 6);
    SEND_DATA_GAME_STATUS.frame_header.id = GAME_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_GAME_STATUS.frame_header),
        sizeof(SEND_DATA_GAME_STATUS.frame_header));
    
    // 8.初始化机器人运动数据
    SEND_ROBOT_MOTION_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_MOTION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotMotion_s) - 6);
    SEND_ROBOT_MOTION_DATA.frame_header.id = ROBOT_MOTION_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_MOTION_DATA.frame_header),
        sizeof(SEND_ROBOT_MOTION_DATA.frame_header));
    
    // 9.初始化地面机器人位置数据
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.sof = SEND_SOF;
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.len =(uint8_t)(sizeof(SendDataGroundRobotPosition_s) - 6);
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.id = GROUND_ROBOT_POSITION_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_GROUND_ROBOT_POSITION_DATA.frame_header),
        sizeof(SEND_GROUND_ROBOT_POSITION_DATA.frame_header));

    // 10.初始化RFID状态数据
    SEND_RFID_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_RFID_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRfidStatus_s) - 6);
    SEND_RFID_STATUS_DATA.frame_header.id = RFID_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_RFID_STATUS_DATA.frame_header),
        sizeof(SEND_RFID_STATUS_DATA.frame_header));

    // 11.初始化机器人状态数据
    SEND_ROBOT_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotStatus_s) - 6);
    SEND_ROBOT_STATUS_DATA.frame_header.id = ROBOT_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_ROBOT_STATUS_DATA.frame_header),
        sizeof(SEND_ROBOT_STATUS_DATA.frame_header));
    
    // 12.初始化云台状态数据
    SEND_JOINT_STATE_DATA.frame_header.sof = SEND_SOF;
    SEND_JOINT_STATE_DATA.frame_header.len = (uint8_t)(sizeof(SendDataJointState_s) - 6);
    SEND_JOINT_STATE_DATA.frame_header.id = JOINT_STATE_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_JOINT_STATE_DATA.frame_header),
        sizeof(SEND_JOINT_STATE_DATA.frame_header));

    // 13.初始化机器人增益和底盘能量数据
    SEND_BUFF_DATA.frame_header.sof = SEND_SOF;
    SEND_BUFF_DATA.frame_header.len = (uint8_t)(sizeof(SendDataBuff_s) - 6);
    SEND_BUFF_DATA.frame_header.id = BUFF_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_BUFF_DATA.frame_header),
        sizeof(SEND_BUFF_DATA.frame_header));
}   


/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/
void USB_Transmit(uint8_t *buffer, uint16_t len)
{
    USBTransmit(buffer, len);    
}

/**
 * @brief 发送DEBUG数据
 * @param duration 发送周期
 */
static void UsbSendDebugData(void)
{
    SEND_DATA_DEBUG.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
    USB_Transmit((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
}

/**
 * @brief 发送IMU数据
 * @param duration 发送周期
 */
static void UsbSendImuData(void)
{
    if (IMU == NULL) {
        return;
    }

    SEND_DATA_IMU.time_stamp = HAL_GetTick();

    SEND_DATA_IMU.data.yaw = IMU->yaw;
    SEND_DATA_IMU.data.pitch = IMU->pitch;
    SEND_DATA_IMU.data.roll = IMU->roll;

    SEND_DATA_IMU.data.yaw_vel = IMU->yaw_vel;
    SEND_DATA_IMU.data.pitch_vel = IMU->pitch_vel;
    SEND_DATA_IMU.data.roll_vel = IMU->roll_vel;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

/**
 * @brief 发送机器人信息数据
 * @param duration 发送周期
 */
static void UsbSendRobotStateInfoData(void)
{
    SEND_DATA_ROBOT_STATE_INFO.time_stamp = HAL_GetTick();

    SEND_DATA_ROBOT_STATE_INFO.data.referee.id = 0;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
}


/**
 * @brief 发送事件数据
 * @param duration 发送周期
 */
static void UsbSendEventData(void)
{
    SEND_DATA_EVENT.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
    USB_Transmit((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
}

/**
 * @brief 发送PidDubug数据
 * @param duration 发送周期
 */
// static void UsbSendPidDebugData(void)
// {
//     SEND_DATA_PID.time_stamp = HAL_GetTick();
//     append_CRC16_check_sum((uint8_t *)&SEND_DATA_PID, sizeof(SendDataPidDebug_s));
// }

/**
 * @brief 发送全场机器人hp信息数据
 * @param duration 发送周期
 */
static void UsbSendAllRobotHpData(void)
{
    SEND_DATA_ALL_ROBOT_HP.time_stamp = HAL_GetTick();

    SEND_DATA_ALL_ROBOT_HP.data.red_1_robot_hp = 1;
    SEND_DATA_ALL_ROBOT_HP.data.red_2_robot_hp = 2;
    SEND_DATA_ALL_ROBOT_HP.data.red_3_robot_hp = 3;
    SEND_DATA_ALL_ROBOT_HP.data.red_4_robot_hp = 4;
    SEND_DATA_ALL_ROBOT_HP.data.red_7_robot_hp = 7;
    SEND_DATA_ALL_ROBOT_HP.data.red_outpost_hp = 8;
    SEND_DATA_ALL_ROBOT_HP.data.red_base_hp = 9;
    SEND_DATA_ALL_ROBOT_HP.data.blue_1_robot_hp = 1;
    SEND_DATA_ALL_ROBOT_HP.data.blue_2_robot_hp = 2;
    SEND_DATA_ALL_ROBOT_HP.data.blue_3_robot_hp = 3;
    SEND_DATA_ALL_ROBOT_HP.data.blue_4_robot_hp = 4;
    SEND_DATA_ALL_ROBOT_HP.data.blue_7_robot_hp = 7;
    SEND_DATA_ALL_ROBOT_HP.data.blue_outpost_hp = 8;
    SEND_DATA_ALL_ROBOT_HP.data.blue_base_hp = 9;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
}

/**
 * @brief 发送比赛状态数据
 * @param duration 发送周期
 */
static void UsbSendGameStatusData(void)
{
    SEND_DATA_GAME_STATUS.time_stamp = HAL_GetTick();

    SEND_DATA_GAME_STATUS.data.game_progress = 1;
    SEND_DATA_GAME_STATUS.data.stage_remain_time = 100;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
    USB_Transmit((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
}

/**
 * @brief 发送机器人运动数据
 * @param duration 发送周期
 */
static void UsbSendRobotMotionData(void)
{
    if (FDB_SPEED_VECTOR == NULL) {
        return;
    }

    SEND_ROBOT_MOTION_DATA.time_stamp = HAL_GetTick();

    SEND_ROBOT_MOTION_DATA.data.speed_vector.vx = FDB_SPEED_VECTOR->vx;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vy = FDB_SPEED_VECTOR->vy;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.wz = FDB_SPEED_VECTOR->wz;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
}

/**
 * @brief 发送地面机器人位置数据
 * @param duration 发送周期
 */
static void UsbSendGroundRobotPositionData(void)
{
    SEND_GROUND_ROBOT_POSITION_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
    USB_Transmit((uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
}

/**
 * @brief 发送RFID状态数据
 * @param duration 发送周期
 */
static void UsbSendRfidStatusData(void)
{
    SEND_RFID_STATUS_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
    USB_Transmit((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
}

/**
 * @brief 发送机器人状态数据
 * @param duration 发送周期
 */
static void UsbSendRobotStatusData(void)
{
    SEND_ROBOT_STATUS_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
}

/**
 * @brief 发送云台状态数据
 * @param duration 发送周期
 */
static void UsbSendJointStateData(void)
{
    SEND_JOINT_STATE_DATA.time_stamp = HAL_GetTick();
    SEND_JOINT_STATE_DATA.data.pitch = 0;//CmdGimbalJointState(AX_PITCH);
    SEND_JOINT_STATE_DATA.data.yaw = 0;// CmdGimbalJointState(AX_YAW);
    append_CRC16_check_sum((uint8_t *)&SEND_JOINT_STATE_DATA, sizeof(SendDataJointState_s));
    USB_Transmit((uint8_t *)&SEND_JOINT_STATE_DATA, sizeof(SendDataJointState_s));
}

/**
 * @brief 发送机器人增益和底盘能量数据
 * @param duration 发送周期
 */
static void UsbSendBuffData(void)
{
    SEND_BUFF_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
    USB_Transmit((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
    
}

/*=================================移植发送end==================================*/

#endif

