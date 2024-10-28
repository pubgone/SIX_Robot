#ifndef _SPORTCALCULATE_H_
#define _SPORTCALCULATE_H_

#include <Arduino.h>
#include <mymath.h>
#include <FashionStar_UartServo.h>
#include <FashionStar_UartServoProtocol.h>

class Rob_Pos
{
private:
    Position3 Wps[6];//机器人末端相对于起始端的位置
    Position3 Wps_default[6];//默认情况下机器人末端相对于起始端的位置
    Position3 Wps_Leg[6];//各个机械腿起始端相对于机器人中心的坐标
    Position3 Rob_position;//机器人中心坐标
    Theta theta;
public:
    void Init();
};
#endif                                                                                                                                          // DEBUG