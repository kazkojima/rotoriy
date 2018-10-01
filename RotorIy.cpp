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

#include "c3ga.h"

using namespace c3ga;

#include "RotorIy.h"

RotorIy::RotorIy (float dt, float gain, float epsilon):
  m_dt(dt), m_gain(gain), m_epsilon(epsilon)
{
  m_S = 1.0;
  m_I = 0.0;
}

RotorIy::~RotorIy ()
{
}

mv
RotorIy::UpdateIMU (float ax, float ay, float az, float gx, float gy, float gz)
{
  // Gyro. bivector and accel. directional vector.
  mv X = m_dt*(gx*(e2^e3) + gy*(e3^e1) + gz*(e1^e2));
  mv y = ax*e1 + ay*e2 + az*e3;

  mv v = applyVersor (m_S, -e3);
  mv dS;
  float nm = norm (y+v);

  // Don't fuse if y+v is too short.
  if (m_gain > 0 && nm > m_norm_threshold)
    {
      mv P = ((1.0/nm)*(y+v))*v;
      mv Y = -2.0*log (P);
      m_I = (1 - m_epsilon)*m_I + m_epsilon*X;
      X -= m_I;
      dS = exp (-0.5*(m_gain*m_dt*Y + X));
    }
  else
    {
      // Fall back to the dead reckoning.
      m_I = (1 - m_epsilon)*m_I + m_epsilon*X;
      X -= m_I;
      dS = exp (-0.5*X);
    }

  m_S = dS*m_S;

  return m_S;
}
