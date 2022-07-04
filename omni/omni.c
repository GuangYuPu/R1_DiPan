#include "omni.h"
#include <math.h>

Wheel wheel[4] = {0};
Wheel wheel_3[3] = {0};
double r[4] = {0};
double r_3[3] = {0};

void Kine_Init(double distance_x, double distance_y, double o_offset_x, double o_offset_y)
{
	// 计算各舵轮相对控制中心的位置
	wheel[0].o_pos_x = -(distance_x / 2 - o_offset_x);
	wheel[0].o_pos_y = distance_y / 2 - o_offset_y;
	wheel[1].o_pos_x = distance_x / 2 - o_offset_x;
	wheel[1].o_pos_y = distance_y / 2 - o_offset_y;
	wheel[2].o_pos_x = -(distance_x / 2 - o_offset_x);
	wheel[2].o_pos_y = -(distance_y / 2 - o_offset_y);
	wheel[3].o_pos_x = distance_x / 2 - o_offset_x;
	wheel[3].o_pos_y = -(distance_y / 2 - o_offset_y);

    for (int i = 0; i < 4; i++)
    {
        r[i] = sqrt(pow(wheel[i].o_pos_x,2)+pow(wheel[i].o_pos_y,2));
    }
    for (int j = 0; j < 3; j++)
    {
        r[j] = 0.2;
    }
}

void Kine_SetSpeed_40(Wheel *wheel,double robot_vx, double robot_vy, double robot_rot)
{
    wheel->speed = r[0]*robot_rot + sqrt(2)*robot_vy/2 - sqrt(2)*robot_vx/2;
}

void Kine_SetSpeed_41(Wheel *wheel,double robot_vx, double robot_vy, double robot_rot)
{
    wheel->speed = r[1]*robot_rot - sqrt(2)*robot_vy/2 - sqrt(2)*robot_vx/2;
}

void Kine_SetSpeed_42(Wheel *wheel,double robot_vx, double robot_vy, double robot_rot)
{
    wheel->speed = r[2]*robot_rot - sqrt(2)*robot_vy/2 + sqrt(2)*robot_vx/2;
}

void Kine_SetSpeed_43(Wheel *wheel,double robot_vx, double robot_vy, double robot_rot)
{
    wheel->speed = r[3]*robot_rot + sqrt(2)*robot_vy/2 + sqrt(2)*robot_vx/2;
}


Kine_SetSpeed_30(Wheel *wheel_3,double robot_vx, double robot_vy, double robot_rot)
{
    wheel_3->speed = r_3[0]*robot_rot - robot_vx;
}

Kine_SetSpeed_31(Wheel *wheel_3,double robot_vx, double robot_vy, double robot_rot)
{
    wheel_3->speed = r_3[1]*robot_rot - sqrt(3)*robot_vy/2 + robot_vx/2;
}

Kine_SetSpeed_32(Wheel *wheel_3,double robot_vx, double robot_vy, double robot_rot)
{
    wheel_3->speed = r_3[2]*robot_rot + sqrt(3)*robot_vy/2 + robot_vx/2;
}

void Kine_SetSpeed(double robot_vx, double robot_vy, double robot_rot)
{
    Kine_SetSpeed_40(&wheel[0],robot_vx,robot_vy,robot_rot);
    Kine_SetSpeed_41(&wheel[1],robot_vx,robot_vy,robot_rot);
    Kine_SetSpeed_42(&wheel[2],robot_vx,robot_vy,robot_rot);
    Kine_SetSpeed_43(&wheel[3],robot_vx,robot_vy,robot_rot);
}

void Kine_SetSpeed_3(double robot_vx, double robot_vy, double robot_rot)
{
    Kine_SetSpeed_30(&wheel_3[0],robot_vx,robot_vy,robot_rot);
    Kine_SetSpeed_31(&wheel_3[1],robot_vx,robot_vy,robot_rot);
    Kine_SetSpeed_32(&wheel_3[2],robot_vx,robot_vy,robot_rot);
}