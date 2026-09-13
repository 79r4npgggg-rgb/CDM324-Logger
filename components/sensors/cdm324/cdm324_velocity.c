#include "cdm324.h"

#include <math.h>

int32_t cdm324_velocity_mmps_from_freq_hz(int32_t freq_hz, uint32_t wheel_diameter_mm)
{
    if (freq_hz <= 0 || wheel_diameter_mm == 0) {
        return 0;
    }

    const double circumference_mm = (double)wheel_diameter_mm * 3.141592653589793;
    const double velocity_mm_per_sec = (double)freq_hz * circumference_mm;
    return (int32_t)lround(velocity_mm_per_sec);
}
