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
 * Fan Platform Implementation Defaults.
 *
 ***********************************************************/
#include <onlp/platformi/fani.h>
#include <onlplib/mmap.h>
#include <fcntl.h>
#include <math.h>
#include "platform_lib.h"
#include <onlplib/file.h>
#include <onlp/platformi/psui.h>

#define FAN_MAX_RPM                         22800         
#define FAN_DIR_RIGHT_TO_LEFT               0
#define FAN_DIR_LEFT_TO_RIGHT               1

/* Static fan information */
#define CHASSIS_FAN_INFO(local_id)		\
    { \
        { ONLP_FAN_ID_CREATE(FAN_##local_id), "Chassis Fan "#local_id, 0 },\
        0x0,\
        ONLP_FAN_CAPS_GET_PERCENTAGE | ONLP_FAN_CAPS_GET_RPM | ONLP_FAN_CAPS_SET_PERCENTAGE,\
        0,\
        0,\
        ONLP_FAN_MODE_INVALID,\
    }

#define PSU_FAN_INFO(pid) \
    { \
        { ONLP_FAN_ID_CREATE(FAN_PSU##pid##_0), "PSU"#pid" Fan 0 ", 0 },\
        0x0,\
        ONLP_FAN_CAPS_GET_PERCENTAGE | ONLP_FAN_CAPS_GET_RPM,\
        0,\
        0,\
        ONLP_FAN_MODE_INVALID,\
    }

onlp_fan_info_t finfo[] = {
    { }, /* Not used */
    CHASSIS_FAN_INFO(1),
    CHASSIS_FAN_INFO(2),
    CHASSIS_FAN_INFO(3),
    PSU_FAN_INFO(1),
    PSU_FAN_INFO(2),
};

#define VALIDATE(_id)                           \
    do {                                        \
        if(!ONLP_OID_IS_FAN(_id)) {             \
            return ONLP_STATUS_E_INVALID;       \
        }                                       \
    } while(0)


static int 
_onlp_fani_rpm_to_percentage(int rpm)
{
    int percentage = 0;
    percentage = round(rpm * 100 / FAN_MAX_RPM);

    if (percentage == 0 && rpm != 0)
    {
        percentage = 1;
    }
	if (rpm >= FAN_MAX_RPM)
    {
        percentage = 100;
    }
    DIAG_PRINT("%s, rpm=%d to percentage=%d", __FUNCTION__, rpm, percentage);

    return percentage;
}

static int
_onlp_fani_info_get_fan(int local_id, onlp_fan_info_t *info)
{
    DIAG_PRINT("%s, local_id=%d", __FUNCTION__, local_id);
    int value;

    /* get fan present status
     */
    if (onlp_file_read_int(&value, "%s""fan%d_present", FAN_BOARD_PATH, local_id) < 0) {
        AIM_LOG_ERROR("Unable to read fan_present from (%s%s)\r\n", FAN_BOARD_PATH, local_id);
        return ONLP_STATUS_E_INTERNAL;
    }

    if (value == 0) {
        return ONLP_STATUS_OK; /* fan is not present */
    }
    info->status |= ONLP_FAN_STATUS_PRESENT;



    /* get fan fault status (turn on when any one fails)
    value=0 : fan speed OK.
    value=1 : fan speed is 0, error.
     */
    if (onlp_file_read_int(&value, "%s""fan%d_fault", FAN_BOARD_PATH, local_id) < 0) {
        AIM_LOG_ERROR("Unable to read fan_status from (%s%s)\r\n", FAN_BOARD_PATH, local_id);
        return ONLP_STATUS_E_INTERNAL;
    }

    if (value > 0) {
        info->status |= ONLP_FAN_STATUS_FAILED;
    }



    /* get fan speed
     */
    if (onlp_file_read_int(&value, "%s""fan%d_input", FAN_BOARD_PATH, local_id) < 0) {
        AIM_LOG_ERROR("Unable to read fan_speed from (%s%s)\r\n", FAN_BOARD_PATH, local_id);
        return ONLP_STATUS_E_INTERNAL;
    }
    /* Get fan rpm */
    info->rpm = value;

    /* Get fan percentage */
    info->percentage = _onlp_fani_rpm_to_percentage(info->rpm); 

	/* Get fan air flow direction 
	value=0 : ( <--- ) air in  (B2F)
	value=1 : ( ---> ) air out (F2B)
	*/
	if (onlp_file_read_int(&value, "%s""fan%d_dir", FAN_BOARD_PATH, local_id) < 0) {
        AIM_LOG_ERROR("Unable to read fan_dir from (%s%s)\r\n", FAN_BOARD_PATH, local_id);
        return ONLP_STATUS_E_INTERNAL;
    }

	if (value == FAN_DIR_RIGHT_TO_LEFT)
	{
		info->status |= ONLP_FAN_STATUS_B2F;
	}
    else
	{
        info->status |= ONLP_FAN_STATUS_F2B;
	}

	return ONLP_STATUS_OK;
}

static uint32_t
_onlp_get_fan_direction_on_psu(void)
{
    int i = 0;

    for (i = PSU1_ID; i <= PSU2_ID; i++) {
        psu_type_t psu_type;
        psu_type = psu_type_get(i, NULL, 0);

        if (psu_type == PSU_TYPE_UNKNOWN) {
            continue;
        }

        if (PSU_TYPE_AC_F2B == psu_type) {
            return ONLP_FAN_STATUS_F2B;
        }
        else {
            return ONLP_FAN_STATUS_B2F;
        }
    }

    return 0;
}


static int
_onlp_fani_info_get_fan_on_psu(int local_id, onlp_fan_info_t* info)
{
    int value;

	/* Get the present state */
    if (onlp_file_read_int(&value, "%s""psu%d_present", PSU_HWMON_PATH, local_id) < 0) 
	{
        AIM_LOG_ERROR("Unable to read status from (%s""psu%d_present)\r\n", PSU_HWMON_PATH, local_id);
        return ONLP_STATUS_E_INTERNAL;
    }

    if (value != PSU_STATUS_PRESENT) 
	{
        return ONLP_STATUS_OK;
    }
    info->status |= ONLP_PSU_STATUS_PRESENT;

    /* get fan direction
     */
    info->status |= _onlp_get_fan_direction_on_psu();

    /* get fan speed
     */
    if (onlp_file_read_int(&value, "%s""psu%d_fan1_input", PSU_PMBUS_PATH, local_id) < 0) {
        AIM_LOG_ERROR("Unable to read psu fan_speed from (%s%s)\r\n", PSU_PMBUS_PATH, local_id);
    	return ONLP_STATUS_E_INTERNAL;
    }
    
	info->rpm = value;
	info->percentage = _onlp_fani_rpm_to_percentage(info->rpm); 

	/* get fan fault status
     */
    if (!info->rpm)
        info->status |= ONLP_FAN_STATUS_FAILED;
	

    return ONLP_STATUS_OK;

}


int
onlp_fani_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);
    return ONLP_STATUS_OK;
}

int
onlp_fani_info_get(onlp_oid_t id, onlp_fan_info_t *info)
{
    DIAG_PRINT("%s, id=%d", __FUNCTION__, id);

    int ret = ONLP_STATUS_OK;
    int local_id;

    VALIDATE(id);

    local_id = ONLP_OID_ID_GET(id);

    *info = finfo[local_id];

    switch(local_id)
    {
        case FAN_PSU1_0:
            ret = _onlp_fani_info_get_fan_on_psu(PSU1_ID, info);
            break;
        case FAN_PSU2_0:
            ret = _onlp_fani_info_get_fan_on_psu(PSU2_ID, info);
            break;
		case FAN_1:
        case FAN_2:
        case FAN_3:
            ret = _onlp_fani_info_get_fan(local_id, info);
            break;
        default:
            ret = ONLP_STATUS_E_INVALID;
            break;
    }
	
    return ret;
}


/*
 * This function sets the speed of the given fan in RPM.
 *
 * This function will only be called if the fan supprots the RPM_SET
 * capability.
 *
 * It is optional if you have no fans at all with this feature.
 */
/* 
  [Note]: By H/W design, fan speed is controlled using percentage.
          RPM value will be translated to percentage and it may produce some deviation.
          (the register size is 8-bit, so there is only 255 units to present the value.)
*/
int
onlp_fani_rpm_set(onlp_oid_t id, int rpm)
{
    DIAG_PRINT("%s, id=%d, rpm=%d", __FUNCTION__, id, rpm);

    if (rpm > FAN_MAX_RPM || rpm <= 0)
    {
        AIM_LOG_INFO("%s:%d rpm:%d is out of range. (1~%d)\n", __FUNCTION__, __LINE__, rpm, FAN_MAX_RPM);
        return ONLP_STATUS_E_PARAM;
    }
    return onlp_fani_percentage_set(id, _onlp_fani_rpm_to_percentage(rpm));
}

/*
 * This function sets the fan speed of the given OID as a percentage.
 *
 * This will only be called if the OID has the PERCENTAGE_SET
 * capability.
 *
 * It is optional if you have no fans at all with this feature.
 */
int
onlp_fani_percentage_set(onlp_oid_t id, int p)
{
	int local_id;
	char data = (char)p;

    DIAG_PRINT("%s, id=%d, p=%d (0x%2X)", __FUNCTION__, id, p, (unsigned char)data);

    VALIDATE(id);

	local_id = ONLP_OID_ID_GET(id);


    switch (local_id)
    {
		case FAN_1:
        case FAN_2:
        case FAN_3:		
            if (onlp_file_write_int(p, "%s""fan%d_pwm", FAN_BOARD_PATH, local_id) < 0) {
                AIM_LOG_ERROR("Unable to read fan_pwm from (%sfan%d_pwm)\r\n", FAN_BOARD_PATH, local_id);
                return ONLP_STATUS_E_INTERNAL;
            }
            break;		
        default:
            return ONLP_STATUS_E_INVALID;
    }

    return ONLP_STATUS_OK;
}


