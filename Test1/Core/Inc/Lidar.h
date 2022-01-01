/*
 * Lidar.h
 *
 *  Created on: Dec 31, 2021
 *      Author: stefan
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_


#define BLOCK_SIZE 132


//#### LIDAR COMMANDS ####
const uint8_t rplidar_reset[] = {0xA5, 0x40};
const uint8_t rplidar_startscan[] = {0xA5, 0x20};


#endif
