/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2020 Alpha Networks Incorporation.
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
#include "platform_lib.h"


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

static int
psu_info_get(onlp_psu_info_t *info)
{
    DIAG_PRINT("%s, Model:%s", __FUNCTION__, info->model);
    int val   = 0;
    int pid = ONLP_OID_ID_GET(info->hdr.id);

    /* Set capability
     */
    info->caps = ONLP_PSU_CAPS_AC;
    psu_serial_number_get(pid, info->serial, sizeof(info->serial));

    if (info->status & ONLP_PSU_STATUS_FAILED)
    {
        return ONLP_STATUS_OK;
    }

    /* Set the associated oid_table */
    info->hdr.coids[0] = ONLP_FAN_ID_CREATE(pid + CHASSIS_FAN_COUNT);
    info->hdr.coids[1] = ONLP_THERMAL_ID_CREATE(pid + CHASSIS_THERMAL_COUNT);

    /* Read voltage, current and power */
    if (psu_pmbus_info_get(pid, "vout", &val) == 0) {
        info->mvout = val;
        info->caps |= ONLP_PSU_CAPS_VOUT;
    }

    if (psu_pmbus_info_get(pid, "vin", &val) == 0) {
        info->mvin = val;
        info->caps |= ONLP_PSU_CAPS_VIN;
    }

    if (psu_pmbus_info_get(pid, "iout", &val) == 0) {
        info->miout = val;
        info->caps |= ONLP_PSU_CAPS_IOUT;
    }
    
    if (psu_pmbus_info_get(pid, "iin", &val) == 0) {
        info->miin = val;
        info->caps |= ONLP_PSU_CAPS_IIN;
    }

    if (psu_pmbus_info_get(pid, "pout", &val) == 0) {
        info->mpout = val;
        info->caps |= ONLP_PSU_CAPS_POUT;
    }   

    if (psu_pmbus_info_get(pid, "pin", &val) == 0) {
        info->mpin = val;
        info->caps |= ONLP_PSU_CAPS_PIN;
    }  

    return ONLP_STATUS_OK;
}

int
onlp_psui_info_get(onlp_oid_t id, onlp_psu_info_t* info)
{
    int val   = 0;
    int ret   = ONLP_STATUS_OK;
    int pid = ONLP_OID_ID_GET(id);
	psu_type_t psu_type;

    VALIDATE(id);

    memset(info, 0, sizeof(onlp_psu_info_t));
    *info = pinfo[pid]; /* Set the onlp_oid_hdr_t */

    /* Get the present state */
    ret = onlp_file_read_int(&val, "%s""psu%d_present", PSU_HWMON_PATH, pid);
    if (ret < 0) 
	{
        AIM_LOG_ERROR("Unable to read status from (%s""psu%d_present)\r\n", PSU_HWMON_PATH, pid);
        return ONLP_STATUS_E_INTERNAL;
    }

    if (val != PSU_STATUS_PRESENT) 
	{
        info->status &= ~ONLP_PSU_STATUS_PRESENT;
        return ONLP_STATUS_OK;
    }
    info->status |= ONLP_PSU_STATUS_PRESENT;


    /* Get power good status */
    ret = onlp_file_read_int(&val, "%s""psu%d_power_good", PSU_HWMON_PATH, pid);
    if (ret < 0) 
	{
        AIM_LOG_ERROR("Unable to read status from (%s""psu%d_power_good)\r\n", PSU_HWMON_PATH, pid);
        return ONLP_STATUS_E_INTERNAL;
    }

    if (val != PSU_STATUS_POWER_GOOD) 
	{
        info->status |=  ONLP_PSU_STATUS_FAILED;
    }

	if (info->status & ONLP_PSU_STATUS_FAILED) 
	{
	    return ONLP_STATUS_OK;
	}
	
	/* Get PSU type */
    psu_type = psu_type_get(pid, info->model, sizeof(info->model));
    switch (psu_type) {
    case PSU_TYPE_AC_F2B:
    case PSU_TYPE_AC_B2F:
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

