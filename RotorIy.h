// Copyright (C) 2018 kaz Kojima
//
// This file is part of RotorIy program.  This program is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program;
// see the file COPYING.

class RotorIy
{
 public:
  RotorIy (float dt, float gain, float epsilon);
  virtual ~RotorIy ();
  // Update with IMU value and return the current estimation.
  mv UpdateIMU (float ax, float ay, float az, float gx, float gy, float gz);
 private:
  // Estimated rotor.
  mv m_S;
  // Integrated gyro bivector which shows its drift.
  mv m_I;
  float m_dt, m_gain, m_epsilon;
  // Is 1.0(~0.1*GRAVITY_MSS) ok?
  const float m_norm_threshold = 1.0;
};
