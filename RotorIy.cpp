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
  _dt(dt), _gain(gain), _epsilon(epsilon)
{
  _S = 1.0;
  _I = 0.0;
}

RotorIy::~RotorIy ()
{
}

mv
RotorIy::UpdateIMU (float ax, float ay, float az, float gx, float gy, float gz)
{
  // Gyro. bivector and accel. directional vector.
  mv X = _dt*(gx*(e2^e3) + gy*(e3^e1) + gz*(e1^e2));
  mv y = ax*e1 + ay*e2 + az*e3;

  mv v = applyVersor (_S, e3);
  mv dS;
  float nm = norm (y+v);

  // Don't fuse if y+v is too short.
  if (norm(y+v) > _norm_threshold)
    {
      mv P = ((1.0/nm)*(y+v))*v;
      mv Y = -2.0*log (P);
      _I = (1 - _epsilon)*_I + _epsilon*(_gain*_dt*Y + X);
      X -= _I;
      dS = exp (-0.5*(_gain*_dt*Y + X));
    }
  else
    {
      // Fall back to the dead reckoning.
      _I = (1 - _epsilon)*_I + _epsilon*X;
      X -= _I;
      dS = exp (-0.5*X);
    }

  _S = dS*_S;

  return _S;
}
