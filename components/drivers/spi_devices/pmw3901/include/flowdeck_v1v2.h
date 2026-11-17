#ifndef FLOWDECKV1V2_H_
#define FLOWDECKV1V2_H_
#include <stdbool.h>
#include <stdint.h>


void flowdeck2Init();
bool flowdeck2Test();
bool flowdeckHasFreshMeasurement(uint32_t maxAgeMs);

typedef struct {
    bool seen;
    bool accepted;
    uint8_t motion;
    uint8_t squal;
    uint16_t shutter;
    int16_t dx;
    int16_t dy;
    float dt;
    uint32_t samples;
    uint32_t acceptedSamples;
    uint32_t ageMs;
    uint32_t acceptedAgeMs; /* UINT32_MAX until the first queued sample. */
    /* Accumulate every burst so low-rate diagnostics do not miss motion. */
    int64_t sumDx;
    int64_t sumDy;
    int64_t submittedDx;
    int64_t submittedDy;
    uint64_t submittedDtUs;
    /* Sensor axes before the mounting transform used by dx/dy. */
    int16_t rawDx;
    int16_t rawDy;
    uint8_t observation;
    uint8_t rawDataSum;
    uint8_t maxRawData;
    uint8_t minRawData;
} flowdeckDiagnostics_t;

void flowdeckGetDiagnostics(flowdeckDiagnostics_t *snapshot);

#endif
