#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include "Sensor.h"  // Base class (assume exists; if not, simple struct with update())
#include "../utils/noise.h"  // Your noise gen/params

class GPSSensor : public Sensor {
public:
    GPSSensor();  // Constructor with noise params

    void update(float dt, const float* true_pos, float* meas_pos);  // Your noise update

    // Outputs (public for access)
    float pos_[3];  // m (x,y,z)

private:
    // No bias for GPS (focus white noise + jumps); add if needed
};

#endif
