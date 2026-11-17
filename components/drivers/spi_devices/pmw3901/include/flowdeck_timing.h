#ifndef FLOWDECK_TIMING_H_
#define FLOWDECK_TIMING_H_

#include <stdbool.h>
#include <stdint.h>

/* Every burst consumes the sensor's accumulated motion, including bursts
 * rejected for low quality. Always advance the clock at the same boundary. */
static inline bool flowdeckSampleDt(uint64_t nowUs, uint64_t *previousUs,
                                    float *dt)
{
    const uint64_t elapsedUs = nowUs - *previousUs;
    *previousUs = nowUs;
    *dt = (float)elapsedUs / 1000000.0f;
    /* After a stall, discard the accumulated motion and resume next burst. */
    return elapsedUs > 0 && elapsedUs <= 50000U;
}

#endif
