#ifndef _OMNI_H__
#define _OMNI_H__

typedef struct WHEEL
{
	float o_pos_x; // 舵轮相对控制中心的 x 位置 (m)
	float o_pos_y; // 舵轮相对控制中心的 y 位置 (m)
	float speed;
} Wheel;

void Kine_Init(float distance_x, float distance_y, float o_offset_x, float o_offset_y);

void Kine_SetSpeed(float robot_vx, float robot_vy, float robot_rot);

void Kine_SetSpeed_3(float robot_vx, float robot_vy, float robot_rot);

extern Wheel wheel[4];
extern Wheel wheel_3[3];
extern float r[4];

#endif