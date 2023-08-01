#ifndef H_CEC
#define H_CEC

#include <Arduino.h>
#include <stdbool.h>
#include <limits.h>

#define MAX_RETRY_COUNT 5

typedef enum {
    CDT_TV,
    CDT_RECORDING_DEVICE,
    CDT_PLAYBACK_DEVICE,
    CDT_TUNER,
    CDT_AUDIO_SYSTEM,
    CDT_OTHER,
} CecDeviceType;

typedef enum {
    SUCCESS,
    UNKNOWN_ERROR = INT_MIN,
    ARBITRATION_ERROR,
    NO_ACKNOWLEDGEMENT,
    RETRY_FAILED,
    TIMING_ERROR,
    INVALID_TARGET,
    BUFFER_OVERFLOW,
} CecError;

void cec_init(int cecPin, CecDeviceType deviceType);
CecError cec_readMsg(unsigned char **data, int *count, bool promiscuous);
CecError cec_transmitMsg(int targetAddr, unsigned char *data, int count);

#endif