#include "lv_sensor_profiles.h"

static const sensor_channel_t lwtm[] = {
    {"PS_Cyl1", 11083}, {"PS_Cyl2", 11084}, {"PS_Cyl3", 11085},
    {"PS_Cyl4", 11086}, {"PS_Cyl5", 11087},
    {"Exh_Cyl1", 11088}, {"Exh_Cyl2", 11089}, {"Exh_Cyl3", 11090},
    {"Exh_Cyl4", 11091}, {"Exh_Cyl5", 11092},
};

static const sensor_channel_t btm[] = {
    {"CrkP_Cyl1", 11011}, {"CrkP_Cyl2", 11012}, {"CrkP_Cyl3", 11013},
    {"CrkP_Cyl4", 11014}, {"CrkP_Cyl5", 11015},
    {"MB_Fore", 11003}, {"MB_Cyl1", 11004}, {"MB_Cyl2", 11005},
    {"MB_Cyl3", 11006}, {"MB_Cyl4", 11007}, {"MB_Cyl5", 11008},
    {"MB_Thrust", 11321},
};

static const sensor_channel_t bwacs[] = {
    {"CYL1_Wear_FS", 10004}, {"CYL2_Wear_FS", 10006},
    {"CYL3_Wear_FS", 10008}, {"CYL4_Wear_FS", 10010},
    {"CYL5_Wear_FS", 10012},
    {"CYL1_Wear_AS", 10005}, {"CYL2_Wear_AS", 10007},
    {"CYL3_Wear_AS", 10009}, {"CYL4_Wear_AS", 10011},
    {"CYL5_Wear_AS", 10013},
};

static const sensor_channel_t owacs[] = {{"Water Activity", 20}};
static const sensor_channel_t avm[] = {{"AVM", 11129}};

#define PROFILE(label, slave, channels, divisor, decimals, unit) \
    {label, slave, sizeof(channels) / sizeof((channels)[0]), channels, divisor, decimals, unit}

const sensor_profile_t sensor_profiles[SENSOR_PROFILE_COUNT] = {
    PROFILE("LWTM", 2, lwtm, 10, 1, "'C"),
    PROFILE("BTM", 2, btm, 10, 1, "'C"),
    PROFILE("B-WACS", 0, bwacs, 1, 0, "um"),
    PROFILE("O-WACS", 1, owacs, 100, 2, "Aw"),
    PROFILE("AVM", 2, avm, 100, 2, "mm"),
};
