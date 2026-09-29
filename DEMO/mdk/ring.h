// ring.h
#ifndef __RING_H
#define __RING_H

#include "zf_common_headfile.h"

// 状态定义
typedef enum {
    RING_IDLE,
    RING_DETECTED,
    RING_ENTER_GUIDE,
    RING_INSIDE,
    RING_EXIT_DETECT,
    RING_EXITING,
    RING_DONE
} ring_state_t;

// 全局变量声明

// 函数声明
void ring_handle(void);
void ring_init(void);

#endif