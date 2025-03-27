#ifndef __LIBNOW_H__
#define __LIBNOW_H__


typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t buttons;
} message_move;

typedef struct{
    float p;
    float i;
    float d;
} message_cal;

typedef enum{
    MOVE = 0,
    CALIBRATE
} MessageType;

typedef enum{
    DST_ROBOT = 0,
    DST_MANDO
} LibNowDst;

void libnow_init();

void libnow_addPeer(LibNowDst dst);

void libnow_sendMessage(LibNowDst dst,message_move *msg);
void libnow_sendMessage(LibNowDst dst,message_cal *msg);

#endif