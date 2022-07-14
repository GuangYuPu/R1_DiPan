#ifndef _NRF_COM_H__
#define _NRF_COM_H__

#include "main.h"

#define HEADER_HIGH 0xff
#define HEADER_MIDDLE_1 0xAA
#define HEADER_MIDDLE_2 0x55
#define HEADER_LOW 0X99

#define HEADER_LENGTH 4
#define HEADER_LENGTH_TRANS 4
#define PAY_LOAD_LENGTH 24
#define PAY_LOAD_LENGTH_TRANS 27
#define BAG_LENGTH (HEADER_LENGTH + PAY_LOAD_LENGTH)
#define BAG_LENGTH_TRANS (HEADER_LENGTH + PAY_LOAD_LENGTH_TRANS)

typedef union{
    uint8_t raw[BAG_LENGTH];
    struct 
    {
        uint8_t header[HEADER_LENGTH];
        union
        {
            uint8_t pay_load[PAY_LOAD_LENGTH];
            struct 
            {
                int Leftx;
                int Lefty;
                int Rightx;
                int Righty;
							  uint8_t button_A;
							  uint8_t button_B;
							  uint8_t button_C;
							  uint8_t button_D;
							  uint8_t button_E;
							  uint8_t button_F;
							  uint8_t button_G;
							  uint8_t button_H;
            }__attribute__((packed));
        };
    }__attribute__((packed));
}__attribute__((packed)) nrfBag_t;

typedef union{
    uint8_t raw[BAG_LENGTH_TRANS];
    struct 
    {
        uint8_t header[HEADER_LENGTH_TRANS];
        union
        {
            uint8_t pay_load[PAY_LOAD_LENGTH_TRANS];
            struct 
            {
                int Leftx;
                int Lefty;
                int Rightx;
                int Righty;
							  uint8_t button_A;
							  uint8_t button_B;
							  uint8_t button_C;
							  uint8_t button_D;
							  uint8_t button_E;
							  uint8_t button_F;
							  uint8_t button_G;
							  uint8_t button_H;
							  uint8_t zone;
							  uint8_t qu_qiu;
							  uint8_t she_qiu;
            }__attribute__((packed));
        };
    }__attribute__((packed));
}__attribute__((packed)) nrfBag_trans_t;

extern nrfBag_trans_t nrfDataBag;
extern uint8_t nrfReceive[BAG_LENGTH*2];
extern int Leftx, Lefty, Rightx, Righty; 
extern uint8_t button_A,button_B,button_C,button_D,button_E,button_F,button_G,button_H,button_G_last,button_H_last;

void nrf_Transmit_init();
void send();
void nrf_receive_init();
void nrf_decode();

#endif