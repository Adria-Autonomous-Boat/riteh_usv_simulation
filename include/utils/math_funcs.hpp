#pragma once

#include <math.h>

#define M_PI_PRECISE	3.141592653589793238462643383279502884
#define M_PI_FLOAT  3.14159265f
#define MATH_PI		3.141592653589793238462643383280
#define FLT_EPSILON	__FLT_EPSILON__


namespace math
{
    
    constexpr float constrain(float val, float min_val, float max_val)
    {
        return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
    }

    template<typename _Tp>
    constexpr _Tp constrain(_Tp val, _Tp min_val, _Tp max_val)
    {
        return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
    }

    constexpr float wrap_pi(float x)
    {
        float low = float(-M_PI_PRECISE);
        float high = float(M_PI_PRECISE);

        // already in range
        if (low <= x && x < high) {
            return x;
        }

        const auto range = high - low;
        const auto inv_range = float(1) / range; // should evaluate at compile time, multiplies below at runtime
        const auto num_wraps = floor((x - low) * inv_range);
        return x - range * num_wraps;
    }

    template<typename T>
    constexpr T radians(T degrees)
    {
        return degrees * (static_cast<T>(MATH_PI) / static_cast<T>(180));
    }

    /*
    * Constant, linear, constant function with the two corner points as parameters
    * y_high          -------
    *                /
    *               /
    *              /
    * y_low -------
    *         x_low   x_high
    */
    template<typename T>
    const T interpolate(const T &value, const T &x_low, const T &x_high, const T &y_low, const T &y_high)
    {
        if (value <= x_low) {
            return y_low;

        } else if (value > x_high) {
            return y_high;

        } else {
            /* linear function between the two points */
            T a = (y_high - y_low) / (x_high - x_low);
            T b = y_low - (a * x_low);
            return (a * value) + b;
        }
    }

    /**
     * Type-safe sign/signum function
     *
     * @param[in] val Number to take the sign from
     * @return -1 if val < 0, 0 if val == 0, 1 if val > 0
     */
    template<typename T>
    int sign(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    template<typename _Tp>
    constexpr _Tp min(_Tp a, _Tp b)
    {
        return (a < b) ? a : b;
    }

    template<typename _Tp>
    constexpr _Tp max(_Tp a, _Tp b)
    {
        return (a > b) ? a : b;
    }

    // Type-safe abs
    template<typename _Tp>
    _Tp abs_t(_Tp val)
    {
        return ((val > (_Tp)0) ? val : -val);
    }

}  /* namespace math */

namespace trajectory
{

    /* Compute the maximum possible speed on the track given the desired speed,
    * remaining distance, the maximum acceleration and the maximum jerk.
    * We assume a constant acceleration profile with a delay of 2*accel/jerk
    * (time to reach the desired acceleration from opposite max acceleration)
    * Equation to solve: vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
    *
    * @param jerk maximum jerk
    * @param accel maximum acceleration
    * @param braking_distance distance to the desired point
    * @param final_speed the still-remaining speed of the vehicle when it reaches the braking_distance
    *
    * @return maximum speed
    */
    inline float computeMaxSpeedFromDistance(const float jerk, const float accel, const float braking_distance,
            const float final_speed)
    {
        auto sqr = [](float f) {return f * f;};
        float b =  4.0f * sqr(accel) / jerk;
        float c = - 2.0f * accel * braking_distance - sqr(final_speed);
        float max_speed = 0.5f * (-b + sqrtf(sqr(b) - 4.0f * c));

        // don't slow down more than the end speed, even if the conservative accel ramp time requests it
        return fmaxf(max_speed, final_speed);
    }
    
}  /* namespace trajectory */
