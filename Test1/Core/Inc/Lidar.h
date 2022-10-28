#ifndef LIDAR_H
#define LIDAR_H
#include "main.h"
#include <stdbool.h>
#include "SWTimer.h"
#include "math.h"
#define PACK_SIZE (5u)
#define CIRCLE_SIZE_DEG_F (360.0f)
#define CIRCLE_SIZE_DEG_I (360u)
#define MIN_ANGLE (0.0f)
#define MAX_ERR (10u)
#define LIDAR_OK (1u)
#define LIDAR_ERR (0u)
#define COMMAND_SIZE (2u)
#define HALF_CIRCLE (180.0f)
#define PI (3.141592654f)
#define BUFF_SIZE ((5*360))
#define ANGLE_SCALE_COEFFICIENT (64.0f)
#define DISTANCE_SCALE_COEFFICIENT (4.0f)
#define COM_DELAY (100u)
#define FIRST_BUFF_SZ (8u)
#define WAIT_MS (1000u)
#define MAX_SEND_NR (5u)
#define MAX_TICKS (504000000u)
#define ANGLE_LOW (1u)
#define ANGLE_HIGH (2u)
#define DIST_LOW (3u)
#define DIST_HIGH (4u)
#define MAX_QUALITY (15u)
void LidarInit(uint8_t* LidarRxBuff, bool *Received, UART_HandleTypeDef* Huart1, uint8_t* LidarState);
uint8_t LidarGetData(const uint8_t* LidarRxBuff, const bool LidarStarted, float* AngleBuff, float* DistanceBuff, const uint8_t LidarState);
#endif
