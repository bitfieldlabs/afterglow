#ifndef AGCONFIG_H
#define AGCONFIG_H

#include <QtGlobal>

// Current version
#define AGCONFIG_VERSION "0.6.0"

// number of columns in the lamp matrix
#define NUM_COL 8

// number of rows in the lamp matrix
#define NUM_ROW 10

// glow duration scaling in the configuration
#define GLOWDUR_CFG_SCALE 10

// afterglow configuration data definition, v1
typedef struct AFTERGLOW_CFG_V1_s
{
    quint16 version;                      // afterglow version of the configuration
    quint16 res;                          // reserved bytes
    quint8 lampGlowDur[8][8];             // Lamp matrix glow duration configuration [ms * GLOWDUR_CFG_SCALE]
    quint8 lampBrightness[8][8];          // Lamp matrix maximum brightness configuration (0-7)
    quint32 crc;                          // data checksum
} AFTERGLOW_CFG_V1_t;

// afterglow configuration data definition, v2 (Stern SAM/Whitestar)
typedef struct AFTERGLOW_CFG_V2_s
{
    quint16 version;                      // afterglow version of the configuration
    quint16 res;                          // reserved bytes
    quint8 lampGlowDur[8][10];            // Lamp matrix glow duration configuration [ms * GLOWDUR_CFG_SCALE]
    quint8 lampBrightness[8][10];         // Lamp matrix maximum brightness configuration (0-7)
    quint32 crc;                          // data checksum
} AFTERGLOW_CFG_V2_t;

// afterglow configuration data definition, v3 (AG 3.0)
typedef struct AFTERGLOW_CFG_s
{
    uint16_t version;                         // afterglow version of the configuration
    uint16_t res;                             // reserved bytes
    uint8_t lampGlowDurOn[NUM_COL][NUM_ROW];  // Lamp glow duration turning on configuration [ms * GLOWDUR_CFG_SCALE]
    uint8_t lampGlowDurOff[NUM_COL][NUM_ROW]; // Lamp glow duration turning off configuration [ms * GLOWDUR_CFG_SCALE]
    uint8_t lampBrightness[NUM_COL][NUM_ROW]; // Lamp maximum brightness configuration (0-7)
    uint8_t lampDelay[NUM_COL][NUM_ROW];      // Lamp delay when turning on (skipping short on-times, anti-ghosting) [ms]
    uint32_t crc;                             // data checksum
} AFTERGLOW_CFG_t;

#endif // AGCONFIG_H
