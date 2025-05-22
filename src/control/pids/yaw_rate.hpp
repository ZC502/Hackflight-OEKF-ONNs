/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <num.hpp>

class YawRateController {

    public:

        /**
          * Input is angular rate demand (deg/sec) and actual angular
          * rate from gyro; ouputput is arbitrary units scaled for motors.
          */
         static float run(
                 const bool airborne,
                 const float dt,
                 const float dpsi,
                 const float yaw)
        {
            static float _integral;

            const auto error = yaw - dpsi;

            _integral = airborne ? 
                Num::fconstrain(_integral + error * dt, ILIMIT) : 0;

            return airborne ? 
                Num::fconstrain(KP * error + KI * _integral, OUTPUT_LIMIT) : 0;
        }

    private:

         static constexpr float KP = 120;
         static constexpr float KI = 16.7;
         static constexpr float ILIMIT = 166.7;
         static constexpr float OUTPUT_LIMIT = INT16_MAX;

         float _integral;
};
