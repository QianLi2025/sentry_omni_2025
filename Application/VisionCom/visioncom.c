#include "visioncom.h"
#include "miniPC_process.h"
#include "ins_task.h"
#include "message_center.h"
#include "robot_def.h"


static Vision_Recv_s *vision_ct; // 视觉控制信息
static attitude_t *ins;
static Robot_Upload_Data_s robot_fetch; //从裁判系统获取的机器人状态信息

static Subscriber_t *robot_feed_sub; // 底盘反馈信息订阅者s
static Publisher_t *vision_ctrl_pub;

extern Vision_Recv_s vision_ctrl;
void VISIONCOM_Init(void){
    ins = INS_Init();

    // 初始化视觉控制
    vision_ct  = VisionInit(&huart1);
    robot_feed_sub = SubRegister("robot_fetch", sizeof(Robot_Upload_Data_s));
    vision_ctrl_pub = PubRegister("vision_ctrl", sizeof(Vision_Recv_s));
}

void VISIONCOM_Task(void){

    // 设置视觉发送数据,还需增加加速度和角速度数据
    SubGetMessage(robot_feed_sub, &robot_fetch);
    static float yaw, pitch, roll, bullet_speed, yaw_speed;

    yaw          = ins->YawTotalAngle;
    pitch        = ins->Pitch;
    roll         = ins->Roll;
    bullet_speed = robot_fetch.bullet_speed;
    yaw_speed    = ins->Gyro[2];

    VisionSetDetectColor(robot_fetch.self_color);
    // VisionSetDetectColor(5);
    VisionSetAltitude(yaw, pitch, roll, bullet_speed, yaw_speed);
    VisionSend();
    PubPushMessage(vision_ctrl_pub, vision_ct);
    vision_ctrl = *vision_ct;

}