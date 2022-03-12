/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "Arduino.h"
#include "IMU.h"

IMU::IMU(USFSMAX* usfsmax, uint8_t sensornum)
{
  _usfsmax   = usfsmax;
  _sensornum = sensornum;
}

/**
* @fn: computeIMU()
*
* @brief: Calculates state estimate using USFSMAX-generated data
* 
* @params: 
* @returns: 
*/
void IMU::computeIMU()
{
  float yaw[2];
  static float buff_roll[2] = {0.0f, 0.0f}, buff_pitch[2] = {0.0f, 0.0f}, buff_heading[2] = {0.0f, 0.0f};
  _usfsmax->getQUAT_Lin();
  Acq_time += micros() - Begin;

  // MAXUSFS Quaternion is ENU
  buff_heading[_sensornum] = atan2f(2.0f*(qt[_sensornum][1]*qt[_sensornum][2] - qt[_sensornum][0]*qt[_sensornum][3]), qt[_sensornum][0]*qt[_sensornum][0] -
                                    qt[_sensornum][1]*qt[_sensornum][1] + qt[_sensornum][2]*qt[_sensornum][2] - qt[_sensornum][3]*qt[_sensornum][3]);
  buff_pitch[_sensornum]   = asinf(2.0f*(qt[_sensornum][2]*qt[_sensornum][3] + qt[_sensornum][0]*qt[_sensornum][1]));
  buff_roll[_sensornum]    = atan2f(2.0f*(qt[_sensornum][0]*qt[_sensornum][2] - qt[_sensornum][1]*qt[_sensornum][3]), qt[_sensornum][0]*qt[_sensornum][0] -
                                    qt[_sensornum][1]*qt[_sensornum][1] - qt[_sensornum][2]*qt[_sensornum][2] + qt[_sensornum][3]*qt[_sensornum][3]);
  buff_heading[_sensornum] *= 57.2957795f;
  buff_pitch[_sensornum]   *= 57.2957795f;
  buff_roll[_sensornum]    *= 57.2957795f;
  angle[_sensornum][0]     = buff_roll[_sensornum];
  angle[_sensornum][1]     = buff_pitch[_sensornum];
  yaw[_sensornum]          = buff_heading[_sensornum];
  heading[_sensornum]      = yaw[_sensornum];                                                                                                                    // Mag declination added in USFSMAX
  if(heading[_sensornum] < 0.0f) heading[_sensornum] += 360.0f;                                                                                                  // Convert heading to 0 - 360deg range
}
