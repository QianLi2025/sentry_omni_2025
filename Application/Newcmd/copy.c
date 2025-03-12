void RobotCMDInit(void)
{
    // 初始化遥控器,使用串口3
    rc_data = RemoteControlInit(&huart3); // 初始化遥控器,C板上使用USART3
#ifdef VIDEO_LINK
    video_data = VideoTransmitterControlInit(&huart6); // 初始化图传链路
#endif
    vision_ctrl     = VisionInit(&huart1); // 初始化视觉控制
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef GIMBAL_BOARD
    UARTComm_Init_Config_s uart_conf ={
        .uart_handle = &huart1,
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
    }
    cmd_uart_comm = UARTCommInit(&uart_conf);
#endif // GIMBAL_BOARD

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD

    shoot_cmd_send.attack_mode  = NORMAL;
    shoot_cmd_send.dead_time    = 600;
    shoot_cmd_send.bullet_speed = SMALL_AMU_30;
    gimbal_cmd_send.pitch       = 0;
    gimbal_cmd_send.yaw         = 0;
    robot_state                 = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
    // 获取各个模块的数据
#ifdef ONE_BOARD
    // 获取底盘反馈信息
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
#endif
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)UARTCommGet(cmd_uart_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();

    shoot_cmd_send.rest_heat = chassis_fetch_data.shoot_limit - chassis_fetch_data.shoot_heat - 20; // 计算剩余热量
    if (!rc_data[TEMP].rc.switch_right ||
        switch_is_down(rc_data[TEMP].rc.switch_right)) // 当收不到遥控器信号时，使用图传链路
    {
        MouseKeySet();
    } else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 当收到遥控器信号时,且右拨杆为中，使用遥控器
    {
        RemoteControlSet();
    } else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
        // EmergencyHandler();
        RemoteMouseKeySet();
    }

    // 设置视觉发送数据,还需增加加速度和角速度数据
    static float yaw, pitch, roll, bullet_speed, yaw_speed;
    yaw          = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    pitch        = gimbal_fetch_data.gimbal_imu_data.Roll;
    roll         = gimbal_fetch_data.gimbal_imu_data.Pitch;
    bullet_speed = chassis_fetch_data.bullet_speed;
    yaw_speed    = gimbal_fetch_data.gimbal_imu_data.Gyro[2];

    VisionSetDetectColor(chassis_fetch_data.self_color);
    VisionSetAltitude(yaw, pitch, roll, bullet_speed, yaw_speed);

    // 发送控制信
    // 推送消息,双板通信,视觉通信等
    chassis_cmd_send.friction_mode = shoot_cmd_send.friction_mode;
    chassis_cmd_send.vision_mode   = vision_ctrl->is_tracking ? LOCK : UNLOCK;
    chassis_cmd_send.lid_mode      = shoot_cmd_send.lid_mode;

    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    UARTCommSend(cmd_uart_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    VisionSend();
}
