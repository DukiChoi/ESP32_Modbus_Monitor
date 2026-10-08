#ifndef LV_SENSOR_PROFILES_H
#define LV_SENSOR_PROFILES_H

#include <stdint.h>
#include <stdbool.h>

#define SENSOR_PROFILE_COUNT 5
#define SENSOR_MAX_CHANNELS 12
#define SENSOR_FUNCTION_CODE 0x03

typedef struct {
    const char *name;
    uint16_t device_address;
    bool show_tooltip; /* First channel of each physical section. */
} sensor_channel_t;

typedef struct {
    const char *name;
    uint8_t slave_id;
    uint8_t channel_count;
    const sensor_channel_t *channels;
    double scale;
    double offset;
    uint8_t decimals;
    const char *unit;
    double sensor_min;
    double sensor_max;
    bool serial_enabled;
} sensor_profile_t;

/* Device Address is sent verbatim, in decimal: no prefix stripping or -1. */
extern const sensor_profile_t sensor_profiles[SENSOR_PROFILE_COUNT];

/* Engineering value = received raw register * scale + offset. */
double sensor_profile_convert_raw(const sensor_profile_t *profile, uint16_t raw);

#endif
