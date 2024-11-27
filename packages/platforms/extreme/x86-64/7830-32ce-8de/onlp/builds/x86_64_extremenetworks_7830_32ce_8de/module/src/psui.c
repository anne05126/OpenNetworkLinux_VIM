/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2018 Alpha Networks Incorporation.
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
 *
 *
 ***********************************************************/
#include <onlp/platformi/psui.h>
#include <onlplib/file.h>
#include <stdio.h>
#include <string.h>
#include "platform_lib.h"

#define PSU_STATUS_PRESENT      1
#define PSU_STATUS_POWER_GOOD   0   /* power CPLD spec version v10 */

#define VALIDATE(_id)                           \
    do {                                        \
        if(!ONLP_OID_IS_PSU(_id)) {             \
            return ONLP_STATUS_E_INVALID;       \
        }                                       \
    } while(0)

int
onlp_psui_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);
    return ONLP_STATUS_OK;
}

static int
psu_info_get(onlp_psu_info_t *info)
{
    DIAG_PRINT("%s, Model:%s", __FUNCTION__, info->model);
    int val   = 0;
    int index = ONLP_OID_ID_GET(info->hdr.id);
	int i = 0;

	/* Get PSU serial number */
	char *string = NULL;
    int len = onlp_file_read_str(&string, "%s""psu%d_serial", PSU_HWMON_PREFIX, index);
    DIAG_PRINT("%s psu%d_serial=%s", __FUNCTION__, index, string);
    if (string && len) {
        aim_strlcpy(info->serial, string, len+1);
    }

    if (string) {
        aim_free(string);
        string = NULL;
    }

    if (info->status & ONLP_PSU_STATUS_FAILED)
    {
        return ONLP_STATUS_OK;
    }

    /* Set the associated oid_table */
	switch(index)
    {
    case PSU1_ID:        
        for(i=0; i<PER_PSU_FAN_COUNT; i++)
		{
			info->hdr.coids[i] = ONLP_FAN_ID_CREATE(index + CHASSIS_FAN_COUNT + i);
		}
		for(i=PER_PSU_FAN_COUNT; i<(PER_PSU_FAN_COUNT+PER_PSU_THERMAL_COUNT); i++)
		{
			info->hdr.coids[i] = ONLP_THERMAL_ID_CREATE(index + CHASSIS_THERMAL_COUNT + (i-PER_PSU_FAN_COUNT));
		}	
        break;
    case PSU2_ID:
        for(i=0; i<PER_PSU_FAN_COUNT; i++)
		{
			info->hdr.coids[i] = ONLP_FAN_ID_CREATE(index + CHASSIS_FAN_COUNT + i + (PER_PSU_FAN_COUNT-1));
		}
		for(i=PER_PSU_FAN_COUNT; i<(PER_PSU_FAN_COUNT+PER_PSU_THERMAL_COUNT); i++)
		{
			info->hdr.coids[i] = ONLP_THERMAL_ID_CREATE(index + CHASSIS_THERMAL_COUNT + (i-PER_PSU_FAN_COUNT) + (PER_PSU_THERMAL_COUNT-1));
		}	
        break;
    default:
        break;
    }


	/* Read voltage, current and power */
    val = 0;
    if (onlp_file_read_int(&val, "%s""psu%d_vin", PSU_HWMON_PREFIX, index) == 0 && val) {
        info->mvin  = val;
        info->caps |= ONLP_PSU_CAPS_VIN;
    }
    
    val = 0;
    if (onlp_file_read_int(&val, "%s""psu%d_vout", PSU_HWMON_PREFIX, index) == 0 && val) {
        info->mvout = val;
        info->caps |= ONLP_PSU_CAPS_VOUT;
    }
	
	val = 0;
	if (onlp_file_read_int(&val, "%s""psu%d_iin", PSU_HWMON_PREFIX, index) == 0 && val) {
		info->miin = val;
		info->caps |= ONLP_PSU_CAPS_IIN;
	}

    val = 0;
    if (onlp_file_read_int(&val, "%s""psu%d_iout", PSU_HWMON_PREFIX, index) == 0 && val) {
        info->miout = val;
        info->caps |= ONLP_PSU_CAPS_IOUT;
    }
	
	val = 0;
	if (onlp_file_read_int(&val, "%s""psu%d_pin", PSU_HWMON_PREFIX, index) == 0 && val) {
		info->mpin = val;
		info->caps |= ONLP_PSU_CAPS_PIN;
	}

    val = 0;
    if (onlp_file_read_int(&val, "%s""psu%d_pout", PSU_HWMON_PREFIX, index) == 0 && val) {
        info->mpout = val;
        info->caps |= ONLP_PSU_CAPS_POUT;
    }

    return ONLP_STATUS_OK;
}

/*
 * Get all information about the given PSU oid.
 */
static onlp_psu_info_t pinfo[] =
{
    { }, /* Not used */
    {    
        { ONLP_PSU_ID_CREATE(PSU1_ID), "PSU-1", 0 },
    },
    {    
        { ONLP_PSU_ID_CREATE(PSU2_ID), "PSU-2", 0 },
    }
};

int
onlp_psui_info_get(onlp_oid_t id, onlp_psu_info_t* info)
{
    int val   = 0;
    int ret   = ONLP_STATUS_OK;
    int index = ONLP_OID_ID_GET(id);
    psu_type_t psu_type;
    char *prefix = NULL;

    VALIDATE(id);

    memset(info, 0, sizeof(onlp_psu_info_t));
    *info = pinfo[index]; /* Set the onlp_oid_hdr_t */

	prefix = PSU_HWMON_PREFIX;

    /* Get PSU present status */	
	if (onlp_file_read_int(&val, "%s""psu%d_present", prefix, index) < 0) 
    {
        AIM_LOG_ERROR("Unable to read present status from PSU(%d)\r\n", index);		
        return ONLP_STATUS_E_INTERNAL;
    }

    if (val != PSU_STATUS_PRESENT) {
        info->status &= ~ONLP_PSU_STATUS_PRESENT;
        return ONLP_STATUS_OK;
    }
	
    info->status |= ONLP_PSU_STATUS_PRESENT;

    /* Get PSU power good status */
	if (onlp_file_read_int(&val, "%s""psu%d_power_good", prefix, index) < 0) 
    {
        AIM_LOG_ERROR("Unable to read power status from PSU(%d)\r\n", index);	
        return ONLP_STATUS_E_INTERNAL;
    }

    if (val != PSU_STATUS_POWER_GOOD) 
    {
        info->status |=  ONLP_PSU_STATUS_FAILED;
    }

	/* Get PSU type and Set capability */   
	psu_type = psu_type_get(index, info->model, sizeof(info->model));

    DIAG_PRINT("%s psu_type=%d", __FUNCTION__, psu_type);

    switch (psu_type) {
        case PSU_TYPE_AC_F2B:
        case PSU_TYPE_AC_B2F:
            info->caps |= ONLP_PSU_CAPS_AC;
            ret = psu_info_get(info);
        case PSU_TYPE_DC_48V_F2B:
        case PSU_TYPE_DC_48V_B2F:
            info->caps |= ONLP_PSU_CAPS_DC48;
            ret = psu_info_get(info);
            break;
        case PSU_TYPE_UNKNOWN:  /* User insert a unknown PSU or unplugged.*/
            info->status |= ONLP_PSU_STATUS_UNPLUGGED;
            info->status &= ~ONLP_PSU_STATUS_FAILED;
            ret = ONLP_STATUS_OK;
            break;
        default:
            ret = ONLP_STATUS_E_UNSUPPORTED;
            break;
    }

    return ret;
}

