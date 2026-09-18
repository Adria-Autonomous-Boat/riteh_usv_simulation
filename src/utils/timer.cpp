
#include "utils/timer.hpp"



int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    int rv = clock_gettime(clk_id, tp);
    hrt_abstime temp_abstime = ts_to_abstime(tp);
    static int32_t dsp_offset = 0;

    if (dsp_offset < 0) {
        hrt_abstime temp_offset = -dsp_offset;

        if (temp_offset >= temp_abstime) { temp_abstime = 0; }

        else { temp_abstime -= temp_offset; }

    } else {
        temp_abstime += (hrt_abstime) dsp_offset;
    }

    tp->tv_sec = temp_abstime / 1000000;
    tp->tv_nsec = (temp_abstime % 1000000) * 1000;
    return rv;
}


hrt_abstime hrt_absolute_time()
{
    struct timespec ts;
    px4_clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts_to_abstime(&ts);
}