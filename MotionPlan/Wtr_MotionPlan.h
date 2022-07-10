#ifndef WTR_MOTIONPLAN__
#define WTR_MOTIONPLAN__

#include "main.h"

typedef struct{
    float LT_x;
    float LT_y;
}LT_t;

typedef struct{
    float XY_x;
    float XY_y;
}XY_t;

typedef struct{
    float P;
    float V;
    float A;
}State_t;

void MotionPlan_Init(float ltx,float lty);
void WTR_MotionPlan_Update(float *vx,float *vy,uint32_t t,float ref_x,float ref_y);

extern LT_t lt_t_now;
extern LT_t lt_t_start;

extern XY_t xy_t_ref;
extern XY_t xy_t_now;
extern XY_t xy_t_start;

extern State_t statex_t_ref;
extern State_t statex_t_now;
extern State_t statey_t_ref;
extern State_t statey_t_now;

extern float ltylast,ltxlast;

#endif