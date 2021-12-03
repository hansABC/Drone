/**
 * @file mpu9250_calibrate.h
 * @author Hans Burmester (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __MPU9250_CALIBRATE_H
#define __MPU9250_CALIBRATE_H

void wait(void);

//GIROSCÓPIO
void calibrate_gyro(void);

//ACELERÓMETRO
void calibrate_accel_axis(int axis, int dir);
void run_next_capture(int axis, int dir);
void calibrate_accel(void);

//MAGNETÓMETRO
void calibrate_mag(void);


#endif // __MPU9250_CALIBRATE_H