/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *        Copyright 2014, 2015 Big Switch Networks, Inc.
 *        Copyright 2020 Alpha Networks Incorporation.
 *
 * Licensed under the Eclipse Public License, Version 1.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *        http://www.eclipse.org/legal/epl-v10.html
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 *
 * </bsn.cl>
 ************************************************************
 *
 * LED Management
 *
 ***********************************************************/
#include <onlp/platformi/ledi.h>
#include <onlp/platformi/fani.h>
#include <onlp/platformi/sysi.h>
#include <sys/mman.h>
#include <stdio.h>
#include "platform_lib.h"
#include <onlplib/mmap.h>
#include <onlplib/file.h>

/* LED_POWER (map to driver) */
#define PWR_LED_MODE_OFF                    0x0         
#define PWR_LED_MODE_GREEN_SOLID            0x1         
#define PWR_LED_MODE_AMBER_BLINKING         0x2            


/* LED_PSU1 (map to driver) */
#define PSU1_LED_MODE_OFF                   0x0           
#define PSU1_LED_MODE_GREEN_SOLID           0x1          
#define PSU1_LED_MODE_AMBER_BLINKING        0x2              

/* LED_PSU2 (map to driver) */
#define PSU2_LED_MODE_OFF                   0x0          
#define PSU2_LED_MODE_GREEN_SOLID           0x1       
#define PSU2_LED_MODE_AMBER_BLINKING        0x2           

/* LED_SYSTEM (map to driver) */
#define SYS_LED_MODE_OFF                    0x0         
#define SYS_LED_MODE_GREEN_SOLID            0x1         
#define SYS_LED_MODE_AMBER_BLINKING         0x2             
#define SYS_LED_MODE_GREEN_BLINKING         0x3            
 
/* LED_FAN (map to driver) */
#define FAN_LED_MODE_OFF                    0x0         
#define FAN_LED_MODE_GREEN_SOLID            0x1         
#define FAN_LED_MODE_AMBER_BLINKING         0x2            

/* LED LOC (map to driver) */
#define LOC_LED_MODE_OFF                    0x0          
#define LOC_LED_MODE_BLUE_BLINKING          0x1          


#define LED_FORMAT          "/sys/devices/platform/spx70d0_led/"
#define CPLD_LED_CONTROL_ADDRESS_OFFSET1           0xA  /* LOC FAN SYS */
#define CPLD_LED_CONTROL_ADDRESS_OFFSET2           0xB  /* PSU1 PSU2 POWER */


#define VALIDATE(_id)                           \
    do {                                        \
        if(!ONLP_OID_IS_LED(_id)) {             \
            return ONLP_STATUS_E_INVALID;       \
        }                                       \
    } while(0)

typedef struct led_id_mode
{
    enum onlp_led_id led_id;
	int driver_led_mode;
    onlp_led_mode_t onlp_led_mode;
} led_light_mode_map_t;

led_light_mode_map_t led_id_mode_data[] = {
	{LED_POWER,   PWR_LED_MODE_OFF,             ONLP_LED_MODE_OFF},
	{LED_POWER,   PWR_LED_MODE_GREEN_SOLID,     ONLP_LED_MODE_GREEN},
    {LED_POWER,   PWR_LED_MODE_AMBER_BLINKING,  ONLP_LED_MODE_ORANGE_BLINKING},

    {LED_PSU1,    PSU1_LED_MODE_OFF,            ONLP_LED_MODE_OFF},
    {LED_PSU1,    PSU1_LED_MODE_GREEN_SOLID,    ONLP_LED_MODE_GREEN},
    {LED_PSU1,    PSU1_LED_MODE_AMBER_BLINKING, ONLP_LED_MODE_ORANGE_BLINKING},

    {LED_PSU2,    PSU2_LED_MODE_OFF,            ONLP_LED_MODE_OFF},
    {LED_PSU2,    PSU2_LED_MODE_GREEN_SOLID,    ONLP_LED_MODE_GREEN},
    {LED_PSU2,    PSU2_LED_MODE_AMBER_BLINKING, ONLP_LED_MODE_ORANGE_BLINKING},

    {LED_SYSTEM,  SYS_LED_MODE_OFF,             ONLP_LED_MODE_OFF},
    {LED_SYSTEM,  SYS_LED_MODE_GREEN_SOLID,     ONLP_LED_MODE_GREEN},
    {LED_SYSTEM,  SYS_LED_MODE_AMBER_BLINKING,  ONLP_LED_MODE_ORANGE_BLINKING},
    {LED_SYSTEM,  SYS_LED_MODE_GREEN_BLINKING,  ONLP_LED_MODE_GREEN_BLINKING},

	{LED_FAN,     FAN_LED_MODE_OFF,             ONLP_LED_MODE_OFF},
	{LED_FAN,     FAN_LED_MODE_GREEN_SOLID,     ONLP_LED_MODE_GREEN},
    {LED_FAN,     FAN_LED_MODE_AMBER_BLINKING,  ONLP_LED_MODE_ORANGE_BLINKING},

	{LED_LOC,     LOC_LED_MODE_OFF,             ONLP_LED_MODE_OFF},
    {LED_LOC,     LOC_LED_MODE_BLUE_BLINKING,   ONLP_LED_MODE_BLUE_BLINKING},
};

static char *leds[] =  /* must map with onlp_led_id (platform_lib.h) */
{
    "reserved",
    "led_pwr",       
    "led_psu1",       
    "led_psu2",       
    "led_sys",        
    "led_fan",        
    "led_loc",        
};

/*
 * Get the information for the given LED OID.
 */
static onlp_led_info_t linfo[] =
{
    { }, /* Not used */
    {
        { ONLP_LED_ID_CREATE(LED_POWER), "Chassis LED 1 (POWER LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_ORANGE_BLINKING,
    },
    {
        { ONLP_LED_ID_CREATE(LED_PSU1), "Chassis LED 2 (PSU1 LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_ORANGE_BLINKING,
    },
    {
        { ONLP_LED_ID_CREATE(LED_PSU2), "Chassis LED 3 (PSU2 LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_ORANGE_BLINKING,
    },
    {
        { ONLP_LED_ID_CREATE(LED_SYSTEM), "Chassis LED 4 (SYSTEM LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_GREEN_BLINKING | ONLP_LED_CAPS_ORANGE,
    },
    {
        { ONLP_LED_ID_CREATE(LED_FAN), "Chassis LED 5 (FAN LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_ORANGE,
    },
    {
        { ONLP_LED_ID_CREATE(LED_LOC), "Chassis LED 6 (LOC LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_BLUE_BLINKING,
    },
};

static int driver_to_onlp_led_mode(enum onlp_led_id id, int driver_led_mode)
{
    int i, nsize = sizeof(led_id_mode_data) / sizeof(led_id_mode_data[0]);
	
    for (i = 0; i < nsize; i++)
    {	
        if ((led_id_mode_data[i].led_id == id) &&
            (led_id_mode_data[i].driver_led_mode == driver_led_mode))
        {
            DIAG_PRINT("%s, id:%d, driver_led_mode:%d onlp_led_mode:%d", __FUNCTION__, id, driver_led_mode, led_id_mode_data[i].onlp_led_mode);
            return led_id_mode_data[i].onlp_led_mode;
        }
    }

    return -1;
}

static int onlp_to_driver_led_mode(enum onlp_led_id id, onlp_led_mode_t onlp_led_mode)
{
    int i, nsize = sizeof(led_id_mode_data) / sizeof(led_id_mode_data[0]);
	
    for (i = 0; i < nsize; i++)
    {
        if ((led_id_mode_data[i].led_id == id) &&
            (led_id_mode_data[i].onlp_led_mode == onlp_led_mode))
        {
            DIAG_PRINT("%s, id:%d, onlp_led_mode:%d driver_led_mode:%d", __FUNCTION__, id, onlp_led_mode, led_id_mode_data[i].driver_led_mode);

			return led_id_mode_data[i].driver_led_mode;
        }
    }
	
    return -1;
}


/*
 * This function will be called prior to any other onlp_ledi_* functions.
 */
int
onlp_ledi_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);

    return ONLP_STATUS_OK;

}

int
onlp_ledi_info_get(onlp_oid_t id, onlp_led_info_t *info)
{
    DIAG_PRINT("%s, id=%d", __FUNCTION__, id);
    int value;
    int local_id = 0;
    VALIDATE(id);

	local_id = ONLP_OID_ID_GET(id);

    /* Set the onlp_oid_hdr_t and capabilities */
    *info = linfo[ONLP_OID_ID_GET(id)];

	/* Get LED mode */
	switch (local_id)
    {
        case LED_POWER:			
        case LED_PSU1:
        case LED_PSU2:
        case LED_SYSTEM:
        case LED_FAN:
        case LED_LOC:    
			if (onlp_file_read_int(&value, "%s%s", LED_FORMAT, leds[local_id]) < 0) 
            {
                AIM_LOG_ERROR("Unable to read led_status from file (%s%s)"LED_FORMAT, leds[local_id]);
                return ONLP_STATUS_E_INTERNAL;
            }

            info->mode = driver_to_onlp_led_mode(local_id, value);
			break;
        default :
            AIM_LOG_INFO("%s:%d %d is a wrong ID\n", __FUNCTION__, __LINE__, local_id);
            return ONLP_STATUS_E_PARAM;
    }

    /* Set the on/off status */
    if (info->mode != ONLP_LED_MODE_OFF)
    {
        info->status |= ONLP_LED_STATUS_ON;
    }

    DIAG_PRINT("[%s] local_id:%d info->mode:%d info->status:%d\n", __FUNCTION__, local_id, info->mode, info->status);

    return ONLP_STATUS_OK;
}

/*
 * Turn an LED on or off.
 *
 * This function will only be called if the LED OID supports the ONOFF
 * capability.
 *
 * What 'on' means in terms of colors or modes for multimode LEDs is
 * up to the platform to decide. This is intended as baseline toggle mechanism.
 */
int
onlp_ledi_set(onlp_oid_t id, int on_or_off)
{
    DIAG_PRINT("%s, id=%d, on_or_off=%d", __FUNCTION__, id, on_or_off);
    VALIDATE(id);

    if (!on_or_off)
    {
        return onlp_ledi_mode_set(id, ONLP_LED_MODE_OFF);
    }

    return onlp_ledi_mode_set(id, ONLP_LED_MODE_ON);
}

/*
 * This function puts the LED into the given mode. It is a more functional
 * interface for multimode LEDs.
 *
 * Only modes reported in the LED's capabilities will be attempted.
 */
int
onlp_ledi_mode_set(onlp_oid_t id, onlp_led_mode_t onlp_led_mode)
{
    DIAG_PRINT("%s, id=%d, onlp_led_mode=%d", __FUNCTION__, id, onlp_led_mode);
    int local_id = 0, driver_led_mode = 0;
	int ret;
    VALIDATE(id);

    local_id = ONLP_OID_ID_GET(id);
	
	driver_led_mode = onlp_to_driver_led_mode(local_id, onlp_led_mode);
    if (driver_led_mode < 0)
        return ONLP_STATUS_E_PARAM;

    switch (local_id)
    {
        case LED_POWER:
        case LED_PSU1:
		case LED_PSU2:
        case LED_SYSTEM:
        case LED_FAN:
        case LED_LOC:
			ret = onlp_file_write_int(driver_led_mode, "%s%s", LED_FORMAT, leds[local_id]);
			if (ret < 0) {
				AIM_LOG_ERROR("%s:%d Unable to write ledi_mode_set from (%s%s), file_write_ERROR=%d\r\n", __FUNCTION__, __LINE__, LED_FORMAT, leds[local_id], ret);
				return ONLP_STATUS_E_INTERNAL;
			}
        	break;	
        default :
            AIM_LOG_INFO("%s:%d %d is a wrong ID\n", __FUNCTION__, __LINE__, local_id);
            return ONLP_STATUS_E_PARAM;
    }
    return ONLP_STATUS_OK;
}


