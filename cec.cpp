#include "cec.h"

#define EOM_BIT 1 < 8
#define ACK_BIT 1 < 9

#define BROADCAST_ADDR 0xf

#define MAKE_ADDRESS(t, d) ((((t) & 0xf) << 4) | ((d) & 0xf))

typedef enum {
    CLA_TV,
    CLA_RECORDING_DEVICE_1,
    CLA_RECORDING_DEVICE_2,
    CLA_TUNER_1,
    CLA_PLAYBACK_DEVICE_1,
    CLA_AUDIO_SYSTEM,
    CLA_TUNER_2,
    CLA_TUNER_3,
    CLA_PLAYBACK_DEVICE_2,
    CLA_RECORDING_DEVICE_3,
    CLA_TUNER_4,
    CLA_PLAYBACK_DEVICE_3,
    CLA_RESERVED_1,
    CLA_RESERVED_2,
    CLA_FREE_USE,
    CLA_UNREGISTERED,
} CecLogicalAddr;

const CecLogicalAddr LOGICAL_ADDRS[6][5] = {
    {CLA_TV,                    CLA_FREE_USE,           CLA_UNREGISTERED,       CLA_UNREGISTERED,       CLA_UNREGISTERED},
    {CLA_RECORDING_DEVICE_1,    CLA_RECORDING_DEVICE_2, CLA_RECORDING_DEVICE_3, CLA_UNREGISTERED,       CLA_UNREGISTERED},
    {CLA_PLAYBACK_DEVICE_1,     CLA_PLAYBACK_DEVICE_2,  CLA_PLAYBACK_DEVICE_3,  CLA_UNREGISTERED,       CLA_UNREGISTERED},
    {CLA_TUNER_1,               CLA_TUNER_2,            CLA_TUNER_3,            CLA_TUNER_4,            CLA_UNREGISTERED},
    {CLA_AUDIO_SYSTEM,          CLA_UNREGISTERED,       CLA_UNREGISTERED,       CLA_UNREGISTERED,       CLA_UNREGISTERED},
    {CLA_UNREGISTERED,          CLA_UNREGISTERED,       CLA_UNREGISTERED,       CLA_UNREGISTERED,       CLA_UNREGISTERED}
};

// Global variables
// --------------------------------------------------------------
int _cecPin;
CecDeviceType _deviceType;
unsigned char _dataBuffer[10];

// Static methods
// --------------------------------------------------------------
static void delayState(int time, int state) {
    pinMode(_cecPin, state == LOW ? OUTPUT : INPUT);
    delayMicroseconds(time);
}

static int waitState(int state) {
    int startTime = micros();
    pinMode(_cecPin, INPUT);
    while (digitalRead(_cecPin) == state);
    return micros() - startTime;
}

static CecError transmitMsg(unsigned char *data, int count) {
    waitState(HIGH);

    // Initiator signal free time
    delayMicroseconds(5 * 2400);

    // Write start bit
    delayState(3700, LOW);
    delayState(800, HIGH);

    if (digitalRead(_cecPin) == LOW) {
        return ARBITRATION_ERROR;
    }

    for (int i = 0; i < count; i++) {
        for (int x = 7; x >= 0; x--) {
            if (data[i] & (1 << x)) { // Bit 1
                delayState(600, LOW);
                delayState(1800, HIGH);
            } else { // Bit 0
                delayState(1500, LOW);
                delayState(900, HIGH);
            }

            // Arbitration takes place until the initiator address bits have been transmitted
            if (!i && x >= 4 && digitalRead(_cecPin) == LOW) {
                return ARBITRATION_ERROR;
            }
        }

        // Transmit EOM bit
        if (i == count - 1) { // Bit 1
            delayState(600, LOW);
            delayState(1800, HIGH);
        } else { // Bit 0
            delayState(1500, LOW);
            delayState(900, HIGH);
        }

        // Transmit ACK bit
        delayState(600, LOW);
        delayState(450, HIGH);

        int targetAddr = _dataBuffer[0] & 0xf;
        if (targetAddr != BROADCAST_ADDR && digitalRead(_cecPin) == HIGH) {
            return NO_ACKNOWLEDGEMENT;
        }

        delayMicroseconds(1350);
    }

    // Next transmit signal free time
    delayMicroseconds(7 * 2400);

    return SUCCESS;
}

static CecError transmitMsgWithRetry(unsigned char *data, int count) {
    for (int i = 0; i < MAX_RETRY_COUNT; i++) {
        if (!transmitMsg(data, count)) {
            return SUCCESS;
        }
        // Retry signal free time
        delayMicroseconds(3 * 2400);
    }

    return RETRY_FAILED;
}

static CecLogicalAddr getLogicalAddr() {
    for (int i = 0; i < 5; i++) {
        if (LOGICAL_ADDRS[_deviceType][i] == CLA_UNREGISTERED) {
            break;
        }

        unsigned char header = MAKE_ADDRESS(LOGICAL_ADDRS[_deviceType][i],
            LOGICAL_ADDRS[_deviceType][i]);
        if (!transmitMsg(&header, 1)) {
            // If not acknowledged, we can use this address
            return LOGICAL_ADDRS[_deviceType][i];
        }
    }

    return CLA_UNREGISTERED;
}

// Public methods
// --------------------------------------------------------------
void cec_init(int cecPin, CecDeviceType deviceType) {
    _cecPin = cecPin;
    _deviceType = deviceType;
}

CecError cec_readMsg(unsigned char **data, int *count, bool promiscuous) {
    CecLogicalAddr logicalAddr = !promiscuous ? getLogicalAddr() : 0;

    waitState(LOW);

    // Verify start bit timing
    int timer = waitState(HIGH);
    if (timer < 3500 || timer > 3900) {
        return TIMING_ERROR;
    }

    timer += waitState(LOW);
    if (timer < 4300 || timer > 4700) {
        return TIMING_ERROR;
    }

    for (int i = 0; i < sizeof(_dataBuffer); i++) {
        // Read block (excluding ack bit)
        for (int x = 7; x >= 0; x--) {
            timer = waitState(HIGH);
            if (timer >= 400 && timer <= 800) {
                _dataBuffer[i] |= 1 << x;
            } else if (timer < 1300 || timer > 1700) {
                return TIMING_ERROR;
            }

            timer += waitState(LOW);
            if (timer < 2050 || timer > 2750) {
                return TIMING_ERROR;
            }
        }

        // Read EOM
        bool eom = false;
        timer = waitState(HIGH);
        if (timer >= 400 && timer <= 800) {
            eom = true;
        }

        timer += waitState(LOW);
        if (timer < 2050 || timer > 2750) {
            return TIMING_ERROR;
        }

        // Handle ACK
        int targetAddr = _dataBuffer[0] & 0xf;
        if (targetAddr != BROADCAST_ADDR && !promiscuous) {
            if (targetAddr != logicalAddr) {
                return INVALID_TARGET;
            }

            // Acknowledge the initiator
            delayState(LOW, 1500);
            timer = 1500;
        } else {
            timer = waitState(HIGH);
        }

        if (eom) {
            *data = _dataBuffer;
            *count = i;
            return SUCCESS;
        }

        // Continue reading if not EOM
        timer += waitState(LOW);
        if (timer < 2050 || timer > 2750) {
           return TIMING_ERROR;
        }
    }

    return BUFFER_OVERFLOW;
}

CecError cec_transmitMsg(int targetAddr, unsigned char *data, int count) {
    CecLogicalAddr logicalAddr = getLogicalAddr();
    unsigned char addr = MAKE_ADDRESS(logicalAddr, targetAddr);
    // @TODO Append addr to buffer or maybe have seperate header block and data block vals
    transmitMsgWithRetry(data, count);
}
