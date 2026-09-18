#pragma once

#include <cstdint>
#include <time.h>
#include <sys/types.h>


/**
 * Absolute time, in microsecond units.
 *
 * Absolute time is measured from some arbitrary epoch shortly after
 * system startup.  It should never wrap or go backwards.
 */
typedef uint64_t	hrt_abstime;

namespace time_literals
{

    // User-defined integer literals for different time units.
    // The base unit is hrt_abstime in microseconds

    constexpr hrt_abstime operator ""_s(unsigned long long seconds)
    {
        return hrt_abstime(seconds * 1000000ULL);
    }

    constexpr hrt_abstime operator ""_ms(unsigned long long milliseconds)
    {
        return hrt_abstime(milliseconds * 1000ULL);
    }

    constexpr hrt_abstime operator ""_us(unsigned long long microseconds)
    {
        return hrt_abstime(microseconds);
    }

} /* namespace time_literals */


/**
 * Convert a timespec to absolute time.
 */
static inline hrt_abstime ts_to_abstime(const struct timespec *ts)
{
    hrt_abstime	result;

    result = (hrt_abstime)(ts->tv_sec) * 1000000;
    result += (hrt_abstime)(ts->tv_nsec / 1000);

    return result;
}

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp);

hrt_abstime hrt_absolute_time();