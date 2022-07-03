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

#ifndef IMU_h
#define IMU_h

#include "def.h"
#include "USFSMAX.h"

extern uint32_t                         Start_time;
extern float                            TimeStamp;
extern uint32_t                         Begin;
extern uint32_t                         Acq_time;
extern float                            qt[2][4];
extern float                            heading[2];
extern float                            angle[2][2];
extern float                            accData[2][3];
extern float                            gyroData[2][3];
extern float                            magData[2][3];
extern int16_t                          accLIN[2][3];                                                                                           
extern float                            acc_LIN[2][3];                                                                                                   // 6DOF linear acceleration
extern float                            Mx[2];
extern float                            My[2];
extern float                            g_per_count;
extern float                            beta;
extern float                            zeta;
extern float                            twoKp;
extern float                            twoKi;
extern float                            integralFBx;
extern float                            integralFBy;
extern float                            integralFBz;

class IMU
{
  public:
                                        IMU(USFSMAX*, uint8_t);
    void                                computeIMU();
    void                                compute_Alternate_IMU();
  private:
    USFSMAX*                            _usfsmax;
    uint8_t                             _sensornum;
};

#endif // IMU_h
