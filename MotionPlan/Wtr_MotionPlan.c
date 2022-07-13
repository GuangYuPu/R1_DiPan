#include "Wtr_MotionPlan.h"
#include <math.h>
#include "ADS1256.h"

LT_t lt_t_now; //当前的激光读数
LT_t lt_t_start; //起点的激光读数

XY_t xy_t_ref; //终点的坐标
XY_t xy_t_now;  //当前的坐标
XY_t xy_t_start; //起点的坐标

State_t statex_t_ref; //当前时刻x方向的预期状态
State_t statex_t_now; //当前时刻x方向的实际状态
State_t statey_t_ref; //当前时刻y方向的预期状态
State_t statey_t_now; //当前时刻y方向的实际状态

float ltylast,ltxlast;

float a0 = 6.8;
float t0 = 0.5;
float Kp = 1;
float Kv = 0.1;

void trans_LT2XY(XY_t *xy_t,float LTx,float LTy,float ltylast)
{
    xy_t->XY_x = LTx + 0.4f;
    xy_t->XY_y = LTy + 0.4;
    if(
       (xy_t->XY_x>0.25&&xy_t->XY_x<0.75)
     ||(xy_t->XY_x>2.25&&xy_t->XY_x<2.75)
     ||(xy_t->XY_x>4.25&&xy_t->XY_x<4.75)
     ||(xy_t->XY_x>6.25&&xy_t->XY_x<6.75)
     )
     { 
        if((ltylast-LTy)>0.1) xy_t->XY_y += 0.15;
        if((ltylast-LTy)<-0.1) xy_t->XY_y -= 0.15;
     }
}

void MotionPlan_Init(float ltx,float lty)
{
    ltxlast = ltx + 0.4;
    ltylast = lty + 0.4;
}


float Caculate_tx(float pos_dis)
{
    float tx;
    tx = (pos_dis - 2*a0*t0*t0*t0)/(a0*t0*t0);
    return tx;
}

float Caculate_t0(float pos_dis)
{
    float t0;
    t0 = pow((pos_dis*0.5/a0),1.0/3);
    return t0;
}

void MotionPlan_state_update(State_t *state_t,uint32_t t_ms,float tx)
{
    float t = ((float)(t_ms))/1000.f;
    if((t>0)&&(t<t0))
    {
        state_t->A = a0*t;
        state_t->V = (1.0/2)*a0*t*t;
        state_t->P = (1.0/6)*a0*t*t*t;
    }
    else if((t>t0)&&(t<2*t0))
    {
        state_t->A = -a0*(t-2*t0);
        state_t->V = -(a0*(t*t - 4*t*t0 + 2*t0*t0))/2;
        state_t->P = -a0*t0*t0*t + a0*t0*t0*t0/3.0 + a0*t0*t*t - a0*t*t*t/6.0;
    }
    else if((t>2*t0)&&(t<(2*t0+tx)))
    {
        state_t->A = 0;
        state_t->V = a0*t0*t0;
        state_t->P = a0*t0*t0*t - a0*t0*t0*t0;
    }
    else if((t>2*t0+tx)&&(t<(3*t0+tx)))
    {
        state_t->A = -a0*(t-2*t0-tx);
        state_t->V = -(1.0/2)*a0*t*t + (2*a0*t0+a0*tx)*t - (a0*t0*t0 + 0.5*a0*tx*tx + 2*a0*t0*tx);
        state_t->P = -(1.0/6)*a0*t*t*t + (t0 + 0.5*tx)*a0*t*t	 - (t0*t0+0.5*tx*tx+2*t0*tx)*a0*t + a0*((1.0/6)*tx*tx*tx+(1.0/3)*t0*t0*t0+2*tx*t0*t0+t0*tx*tx);
    }
    else if((t>3*t0+tx)&&(t<(4*t0+tx)))
    {
        state_t->A = a0*(t-4*t0-tx);
        state_t->V = (1.0/2)*a0*t*t - a0*(4*t0 + tx)*t + a0*(8*t0*t0+4*t0*tx+0.5*tx*tx);
        //state_t->P = (1.0/6)*a0*t*t*t - a0*(0.5*tx*t*t + 2*t0*t*t) + a0*t*(8*t0*t0+4*t0*tx+0.5*tx*tx) - a0*((26.0/3)*t0*t0*t0+5*t0*t0*tx-(1.0/6)*pow(tx,3)+2*tx*tx*tx);    }
		state_t->P = -(a0*(- t*t*t + 12*t*t*t0 + 3*t*t*tx - 48*t*t0*t0 - 24*t*t0*tx - 3*t*tx*tx + 52*t0*t0*t0 + 42*t0*t0*tx + 12*t0*tx*tx + tx*tx*tx))/6;

		}
}

void MotionPlan_state_update_t0(State_t *state_t,uint32_t t_ms,float t0)
{
    float t = ((float)(t_ms))/1000.f;
    float tx = 0;
    if((t>0)&&(t<t0))
    {
        state_t->A = a0*t;
        state_t->V = (1.0/2)*a0*t*t;
        state_t->P = (1.0/6)*a0*t*t*t;
    }
    else if((t>t0)&&(t<2*t0))
    {
        state_t->A = -a0*(t-2*t0);
        state_t->V = -(a0*(t*t - 4*t*t0 + 2*t0*t0))/2;
        state_t->P = -a0*t0*t0*t + a0*t0*t0*t0/3.0 + a0*t0*t*t - a0*t*t*t/6.0;
    }
    else if((t>2*t0)&&(t<(2*t0+tx)))
    {
        state_t->A = 0;
        state_t->V = a0*t0*t0;
        state_t->P = a0*t0*t0*t - a0*t0*t0*t0;
    }
    else if((t>2*t0+tx)&&(t<(3*t0+tx)))
    {
        state_t->A = -a0*(t-2*t0-tx);
        state_t->V = -(1.0/2)*a0*t*t + (2*a0*t0+a0*tx)*t - (a0*t0*t0 + 0.5*a0*tx*tx + 2*a0*t0*tx);
        state_t->P = -(1.0/6)*a0*t*t*t + (t0 + 0.5*tx)*a0*t*t	 - (t0*t0+0.5*tx*tx+2*t0*tx)*a0*t + a0*((1.0/6)*tx*tx*tx+(1.0/3)*t0*t0*t0+2*tx*t0*t0+t0*tx*tx);
    }
    else if((t>3*t0+tx)&&(t<(4*t0+tx)))
    {
        state_t->A = a0*(t-4*t0-tx);
        state_t->V = (1.0/2)*a0*t*t - a0*(4*t0 + tx)*t + a0*(8*t0*t0+4*t0*tx+0.5*tx*tx);
        //state_t->P = (1.0/6)*a0*t*t*t - a0*(0.5*tx*t*t + 2*t0*t*t) + a0*t*(8*t0*t0+4*t0*tx+0.5*tx*tx) - a0*((26.0/3)*t0*t0*t0+5*t0*t0*tx-(1.0/6)*pow(tx,3)+2*tx*tx*tx);    }
		state_t->P = -(a0*(- t*t*t + 12*t*t*t0 + 3*t*t*tx - 48*t*t0*t0 - 24*t*t0*tx - 3*t*tx*tx + 52*t0*t0*t0 + 42*t0*t0*tx + 12*t0*tx*tx + tx*tx*tx))/6;

		}
}

void MotionPlan_Caculate(float *v,float ref_v,float ref_p,float now_v,float now_p)
{
    *v =Kp*(ref_p - now_p) + Kv*(ref_v - now_v);
}

void MotionPlan_Servo(float ref_x,float ref_y)
{
    xy_t_ref.XY_x = ref_x;
    xy_t_ref.XY_y = ref_y;
}

void WTR_MotionPlan_Update(float *vx,float *vy,uint32_t t_ms,float ref_x,float ref_y)
{
    float t = ((float)(t_ms))/1000.f;
    MotionPlan_Servo(ref_x,ref_y);
    
    ltxlast = statex_t_now.P;
    ltylast = statey_t_now.P;

    lt_t_now.LT_x = (float)ADS1256_diff_data[0];
    lt_t_now.LT_y = (float)ADS1256_diff_data[1];

    if(t_ms == 0)
    {
        lt_t_start.LT_x = lt_t_now.LT_x;
        lt_t_start.LT_y = lt_t_now.LT_y;
        trans_LT2XY(&xy_t_start,lt_t_start.LT_x,lt_t_start.LT_y,ltylast);
    }

    trans_LT2XY(&xy_t_now,lt_t_now.LT_x,lt_t_now.LT_y,ltylast);
    
    statex_t_now.P = xy_t_now.XY_x;
    statey_t_now.P = xy_t_now.XY_y;
    statex_t_now.V = (xy_t_now.XY_x - ltxlast)*1000;
    statey_t_now.V = (xy_t_now.XY_y - ltylast)*1000;

    float posdis_x = xy_t_ref.XY_x - xy_t_start.XY_x;
    float posdis_y = xy_t_ref.XY_y - xy_t_start.XY_y;

    float tx_x = Caculate_tx(posdis_x);
    float tx_y = Caculate_tx(posdis_y);

    if(tx_x > 0)
    {
        MotionPlan_state_update(&statex_t_ref,t_ms,tx_x);
    }
    else
    {
        float t0x = Caculate_t0(posdis_x);
        MotionPlan_state_update_t0(&statex_t_ref,t_ms,t0x);
    }

    if(tx_y > 0)
    {
        MotionPlan_state_update(&statey_t_ref,t_ms,tx_y);
    }
    else
    {
        float t0y = Caculate_t0(posdis_y);
        MotionPlan_state_update_t0(&statey_t_ref,t_ms,t0y);
    }

    MotionPlan_Caculate(vx,statex_t_ref.V,statex_t_ref.P,statex_t_now.V,statex_t_now.P);
    MotionPlan_Caculate(vy,statey_t_ref.V,statey_t_ref.P,statey_t_now.V,statey_t_now.P);
}