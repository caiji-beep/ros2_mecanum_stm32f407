  git config --global user.email "you@example.com"
  git config --global user.name "Your Name"

备份
更新1（截止2025.11.22 20:34）：
1.移植RTOS
2.添加状态机管理
更新2（2025112301）
1.添加死区补偿及优化
2.软件看门狗（Nav command watchdog）
3.一些bug
更新3（2025112302）
1.硬件看门狗
更新4（2025112303）
1.添加蜂鸣器提醒
2.一些bug
3.添加icm20948驱动（待移植进系统）

更新5（2025120301）
1.移植了can通信（与stm32f103）
2.将icm20948驱动移植入系统（硬件IIC）
3.解决oled屏幕卡死bug
4.初步解决导航时can发送的imu数据不更新
5.待解决问题
  5.1 按键翻页显示问题
  5.2 与上位机can通信
  5.3 ICM20948改成软件IIC（视情况而定） 

更新6（2025120401）
1.按键翻页显示问题
2.与上位机can通信