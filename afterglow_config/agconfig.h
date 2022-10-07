#ifndef AGCONFIG_H
#define AGCONFIG_H

#include <QtGlobal>

// Current version
#define AGCONFIG_VERSION "0.5.0"

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
typedef struct AFTERGLOW_CFG_s
{
    quint16 version;                      // afterglow version of the configuration
    quint16 res;                          // reserved bytes
    quint8 lampGlowDur[NUM_COL][NUM_ROW];    // Lamp matrix glow duration configuration [ms * GLOWDUR_CFG_SCALE]
    quint8 lampBrightness[NUM_COL][NUM_ROW]; // Lamp matrix maximum brightness configuration (0-7)
    quint32 crc;                          // data checksum
} AFTERGLOW_CFG_t;

#endif // AGCONFIG_H
