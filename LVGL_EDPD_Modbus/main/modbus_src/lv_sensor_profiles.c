#include "lv_sensor_profiles.h"

static const sensor_channel_t lwtm[] = {
    {"PS_Cyl1", 11083, true}, {"PS_Cyl2", 11084}, {"PS_Cyl3", 11085},
    {"PS_Cyl4", 11086}, {"PS_Cyl5", 11087},
    {"Exh_Cyl1", 11088, true}, {"Exh_Cyl2", 11089}, {"Exh_Cyl3", 11090},
    {"Exh_Cyl4", 11091}, {"Exh_Cyl5", 11092},
};

static const sensor_channel_t btm[] = {
    {"CrkP_Cyl1", 11011, true}, {"CrkP_Cyl2", 11012}, {"CrkP_Cyl3", 11013},
    {"CrkP_Cyl4", 11014}, {"CrkP_Cyl5", 11015},
    {"MB_Fore", 11003, true}, {"MB_Cyl1", 11004}, {"MB_Cyl2", 11005},
    {"MB_Cyl3", 11006}, {"MB_Cyl4", 11007}, {"MB_Cyl5", 11008},
    {"MB_Thrust", 11321},
};

static const sensor_channel_t bwacs[] = {
    {"CYL1_Wear_FS", 10004, true}, {"CYL2_Wear_FS", 10006},
    {"CYL3_Wear_FS", 10008}, {"CYL4_Wear_FS", 10010},
    {"CYL5_Wear_FS", 10012},
    {"CYL1_Wear_AS", 10005, true}, {"CYL2_Wear_AS", 10007},
    {"CYL3_Wear_AS", 10009}, {"CYL4_Wear_AS", 10011},
    {"CYL5_Wear_AS", 10013},
};

static const sensor_channel_t owacs[] = {{"Water Activity", 20, true}};
static const sensor_channel_t avm[] = {{"AVM", 11129, true}};

#define PROFILE(label, slave, channels, scale, unit, minimum, maximum, enabled) \
    {label, slave, sizeof(channels) / sizeof((channels)[0]), channels, scale, 0.0, 2, unit, minimum, maximum, enabled}

const sensor_profile_t sensor_profiles[SENSOR_PROFILE_COUNT] = {
    PROFILE("LWTM", 2, lwtm, 0.1, "°C", 0, 300, true),
    PROFILE("BTM", 2, btm, 0.1, "°C", 0, 200, true),
    PROFILE("B-WACS", 0, bwacs, 1.0, "µm", 1000, 7000, false),
    PROFILE("O-WACS", 1, owacs, 0.01, "Aw", 0, 1, true),
    PROFILE("AVM", 2, avm, 0.01, "mm", 0, 20, true),
};

double sensor_profile_convert_raw(const sensor_profile_t *profile, uint16_t raw)
{
    return (double)raw * profile->scale + profile->offset;
}

