/**
 * @file Madgwick.h
 * @author Hans Burmester (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-09-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef Madgwick_h
#define Madgwick_h

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSinit(float sampleFreqDef, float betaDef);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickGetEulerAnglesDegrees(float *heading, float *pitch, float *roll);

#endif