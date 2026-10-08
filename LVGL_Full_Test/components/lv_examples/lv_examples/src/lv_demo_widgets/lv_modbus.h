/**
 * @file lv_modbus.h
 *
 */

#ifndef LV_MODBUS_H
#define LV_MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void lv_modbus(void);
void modbus_set_target(uint8_t slave_id, uint16_t reg_addr);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_MODBUS_H*/
