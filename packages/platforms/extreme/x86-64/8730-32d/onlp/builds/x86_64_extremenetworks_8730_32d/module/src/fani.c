/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2017 Accton Technology Corporation.
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
#include "platform_lib.h"

#define MAX_FAN_SPEED_INLET     30500
#define MAX_FAN_SPEED_OUTLET    24600
#define MAX_PSU_FAN_SPEED       32000

#define FAN_STATUS_PRESENT      1
#define FAN_STATUS_GOOD         1
#define FAN_STATUS_F2B		    0

#define TLV_CODE_PART_NUM       0x800d
#define TLV_CODE_SERIAL_NUM     0x800e

#define CHASSIS_FAN_INFO(fid)		\
    { \
        { ONLP_FAN_ID_CREATE(FAN_##fid##_ON_FAN_BOARD), "Chassis Fan - "#fid, 0 },\
        0x0,\
        ONLP_FAN_CAPS_GET_RPM | ONLP_FAN_CAPS_GET_PERCENTAGE,\
        0,\
        0,\
        ONLP_FAN_MODE_INVALID,\
    }

#define PSU_FAN_INFO(pid, fid) \
    { \
        { ONLP_FAN_ID_CREATE(FAN_##fid##_ON_PSU_##pid), "PSU "#pid" - Fan "#fid, 0 },\
        0x0,\
        ONLP_FAN_CAPS_GET_RPM | ONLP_FAN_CAPS_GET_PERCENTAGE,\
        0,\
        0,\
        ONLP_FAN_MODE_INVALID,\
    }


/* Static fan information */
onlp_fan_info_t finfo[] = {
    { }, /* Not used */
    CHASSIS_FAN_INFO(1),
    CHASSIS_FAN_INFO(2),
    CHASSIS_FAN_INFO(3),
    CHASSIS_FAN_INFO(4),
    CHASSIS_FAN_INFO(5),
    CHASSIS_FAN_INFO(6),
    CHASSIS_FAN_INFO(7),
    PSU_FAN_INFO(1,1),
    PSU_FAN_INFO(1,2),
    PSU_FAN_INFO(2,1),
    PSU_FAN_INFO(2,2),
};

#define VALIDATE(_id)                           \
    do {                                        \
        if(!ONLP_OID_IS_FAN(_id)) {             \
            return ONLP_STATUS_E_INVALID;       \
        }                                       \
    } while(0)

static int
_onlp_fani_info_get_fan(int fid, onlp_fan_info_t* info)
{
    int   value, inlet, outlet, max_speed;
    uint8_t rdata[256] = { 0 };
    int size = 0;

    /* get fan present status
     */
    if (onlp_file_read_int(&value, "%s""fan%d_present", FAN_BOARD_PATH, fid) < 0) {
        AIM_LOG_ERROR("Unable to read present status from (%s)\r\n", FAN_BOARD_PATH);
        return ONLP_STATUS_E_INTERNAL;
    }

    if (value != FAN_STATUS_PRESENT) {
        return ONLP_STATUS_OK; /* fan is not present */
    }
    info->status |= ONLP_FAN_STATUS_PRESENT;

	/* get fan direction status
     */
    if (onlp_file_read_int(&value, "%s""fan%d_dir", FAN_BOARD_PATH, fid) < 0) {
        AIM_LOG_ERROR("Unable to read direction status from (%s)\r\n", FAN_BOARD_PATH);
        return ONLP_STATUS_E_INTERNAL;
    }

    if (value == FAN_STATUS_F2B) {
        info->status |= ONLP_FAN_STATUS_F2B;
    }
	else {
    	info->status |= ONLP_FAN_STATUS_B2F;
	}

    /* get fan fault status (turn on when any one fails)
     */
    if (onlp_file_read_int(&inlet, "%s""fan%d_fault_inlet", FAN_BOARD_PATH, fid) < 0) {
        AIM_LOG_ERROR("Unable to read fault inlet status from (%s)\r\n", FAN_BOARD_PATH);
        return ONLP_STATUS_E_INTERNAL;
    }

    if (onlp_file_read_int(&outlet, "%s""fan%d_fault_outlet", FAN_BOARD_PATH, fid) < 0) {
        AIM_LOG_ERROR("Unable to read fault outlet status from (%s)\r\n", FAN_BOARD_PATH);
        return ONLP_STATUS_E_INTERNAL;
    }

    if (inlet != FAN_STATUS_GOOD || outlet != FAN_STATUS_GOOD) {
        info->status |= ONLP_FAN_STATUS_FAILED;
    }

    /* get fan speed
     */
    if (onlp_file_read_int(&inlet, "%s""fan%d_input_inlet", FAN_BOARD_PATH, fid) < 0) {
        AIM_LOG_ERROR("Unable to read speed inlet from (%s)\r\n", FAN_BOARD_PATH);
        return ONLP_STATUS_E_INTERNAL;
    }

    if (onlp_file_read_int(&outlet, "%s""fan%d_input_outlet", FAN_BOARD_PATH, fid) < 0) {
        AIM_LOG_ERROR("Unable to read speed outlet from (%s)\r\n", FAN_BOARD_PATH);
        return ONLP_STATUS_E_INTERNAL;
    }

    if(inlet < outlet)
    {
        value = inlet;
        max_speed = MAX_FAN_SPEED_INLET;
    }
    else
    {
        value = outlet;
        max_speed = MAX_FAN_SPEED_OUTLET;
    }
    
    info->rpm = value;
	
    /* get speed percentage from rpm
     */
    info->percentage = (info->rpm * 100)/max_speed;

    /* get fan eeprom (model name and serial number)
    */
    if(onlp_file_read(rdata, 256, &size, "%s""fan%d_eeprom", FAN_BOARD_PATH, fid) != ONLP_STATUS_OK) 
    {
        AIM_LOG_ERROR("Unable to read eeprom from fan(%d)\r\n", fid);
        return ONLP_STATUS_E_INTERNAL;
    }

#if 0 /* debug */
    int i;
    for(i=0;i<256;i++)
    {
        printf("%02X ",rdata[i]);
    }
    printf("\n");
#endif

    eeprom_tlv_read(rdata, TLV_CODE_PART_NUM, (char *)info->model);
    eeprom_tlv_read(rdata, TLV_CODE_SERIAL_NUM, (char *)info->serial);

    return ONLP_STATUS_OK;
}

static uint32_t
_onlp_get_fan_direction_on_psu(int pid)
{
    psu_type_t psu_type;
    psu_type = psu_type_get(pid, NULL, 0);

    if (psu_type == PSU_TYPE_UNKNOWN) {
        return 0;
    }

    if (PSU_TYPE_AC_F2B == psu_type || PSU_TYPE_DC_48V_F2B == psu_type) {
        return ONLP_FAN_STATUS_F2B;
    }
    else {
        return ONLP_FAN_STATUS_B2F;
    }

    return 0;
}

static int
_onlp_fani_info_get_fan_on_psu(int pid, int fid, onlp_fan_info_t* info)
{
    int val = 0;

    info->status |= ONLP_FAN_STATUS_PRESENT;

    /* get fan direction
     */
    info->status |= _onlp_get_fan_direction_on_psu(pid);
		
	val = 0;
	if (onlp_file_read_int(&val, "%s""psu%d_fan%d_input", PSU_HWMON_PREFIX, pid, fid) == 0 && val) {
		info->rpm = val;
        info->percentage = (info->rpm * 100) / MAX_PSU_FAN_SPEED;
	}

	/* get fan fault status
     */
    if (!info->rpm) {
        info->status |= ONLP_FAN_STATUS_FAILED;
    }

    return ONLP_STATUS_OK;
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
	return ONLP_STATUS_E_UNSUPPORTED;
}


/*
 * This function will be called prior to all of onlp_fani_* functions.
 */
int
onlp_fani_init(void)
{
    return ONLP_STATUS_OK;
}

int
onlp_fani_info_get(onlp_oid_t id, onlp_fan_info_t* info)
{
    DIAG_PRINT("%s, id=%d", __FUNCTION__, id);

    int rc = 0;
    int fid;
    VALIDATE(id);

    fid = ONLP_OID_ID_GET(id);
    *info = finfo[fid];

    switch (fid)
    {
    case FAN_1_ON_PSU_1:
		rc = _onlp_fani_info_get_fan_on_psu(PSU1_ID, PSU_FAN1_ID, info);
        break;
	case FAN_2_ON_PSU_1:
        rc = _onlp_fani_info_get_fan_on_psu(PSU1_ID, PSU_FAN2_ID, info);
        break;
    case FAN_1_ON_PSU_2:
		rc = _onlp_fani_info_get_fan_on_psu(PSU2_ID, PSU_FAN1_ID, info);
        break;
	case FAN_2_ON_PSU_2:
        rc = _onlp_fani_info_get_fan_on_psu(PSU2_ID, PSU_FAN2_ID, info);
        break;
    case FAN_1_ON_FAN_BOARD:
    case FAN_2_ON_FAN_BOARD:
    case FAN_3_ON_FAN_BOARD:
    case FAN_4_ON_FAN_BOARD:
    case FAN_5_ON_FAN_BOARD:
    case FAN_6_ON_FAN_BOARD:
	case FAN_7_ON_FAN_BOARD:
        rc =_onlp_fani_info_get_fan(fid, info);
        break;
    default:
        rc = ONLP_STATUS_E_INVALID;
        break;
    }

    return rc;
}
