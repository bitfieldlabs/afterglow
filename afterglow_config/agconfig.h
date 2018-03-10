#ifndef AGCONFIG_H
#define AGCONFIG_H

// number of columns in the lamp matrix
#define NUM_COL 8

// number of rows in the lamp matrix
#define NUM_ROW 8

// glow duration scaling in the configuration
#define GLOWDUR_CFG_SCALE 10

// afterglow configuration data definition
typedef struct AFTERGLOW_CFG_s
{
    uint16_t version;                      // afterglow version of the configuration
    uint16_t res;                          // reserved bytes
    uint8_t lampGlowDur[NUM_COL][NUM_ROW];    // Lamp matrix glow duration configuration [ms * GLOWDUR_CFG_SCALE]
    uint8_t lampBrightness[NUM_COL][NUM_ROW]; // Lamp matrix maximum brightness configuration (0-7)
    uint32_t crc;                          // data checksum
} AFTERGLOW_CFG_t;

#endif // AGCONFIG_H
