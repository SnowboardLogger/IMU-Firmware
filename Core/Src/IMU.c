/*
 * IMU.c
 *
 *  Created on: Mar 5, 2023
 *      Author: wmyke
 */

// Planning to use Mode 1 (pg 35) w/ I2C
// Pinout Information (pg 7/8)

/* Underlying text is utilized for I2C Operation (pg 18) */
/* Assume SDO/SA0 is connected to Vdd (LSB is 1)
#define SAD_READ 0xD7
#define SAD_WRITE 0xD6
*/
// Assume SDO/SA0 is connected to Gnd (LSB is 0)
#define SAD_READ 0xD5
#define SAD_WRITE 0xD4
/* End of I2C Operation Zone */

/* Underlying text is utilized for SPI Operation (pg 20) */
/* End of SPI Operation Zone */

/* Control Registers */
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define CTRL4_C 0x13
#define CTRL5_C 0x14
#define CTRL6_C 0x15
#define CTRL7_G 0x16
#define CTRL8_XL 0x17
#define CTRL9_XL 0x18
#define CTRL10_C 0x19

// Temperature Sensor Registers
#define OUT_TEMP_L 0x20
#define OUT_TEMP_H 0x21

// G stands for angular rate
#define OUTX_L_G 0x22
#define OUTX_H_H 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

// A stands for linear acceleration
#define OUTX_L_A 0x28
#define OUTX_H_A 0x29
#define OUTY_L_A 0x2A
#define OUTY_H_A 0x2B
#define OUTZ_L_A 0x2C
#define OUTZ_H_A 0x2D

// Accelerometer *-Axis User Offset Correction
#define X_OFS_USR 0x73
#define Y_OFS_USR 0x74
#define Z_OFS_USR 0x75


