/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *        Copyright 2014, 2015 Big Switch Networks, Inc.
 *        Copyright 2020 Alpha Networks Incorporation
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
 * Thermal Sensor Platform Implementation.
 *
 ***********************************************************/
#include <unistd.h>
#include <onlplib/mmap.h>
#include <onlplib/file.h>
#include <onlp/platformi/thermali.h>
#include <fcntl.h>
#include "platform_lib.h"
#include <onlplib/file.h>
#include <onlp/platformi/psui.h>

#define VALIDATE(_id)                           \
    do {                                        \
        if(!ONLP_OID_IS_THERMAL(_id)) {         \
            return ONLP_STATUS_E_INVALID;       \
        }                                       \
    } while(0)

static char* devfiles__[] =  /* must map with onlp_thermal_id (platform_lib.h) */
{
    "reserved",
	"/sys/devices/platform/coretemp.0/hwmon/hwmon1/temp1_input", 
    "/sys/devices/platform/spx70d0_thermal/temp1_input",
    "/sys/devices/platform/spx70d0_thermal/temp2_input",
    "/sys/devices/platform/spx70d0_thermal/temp3_input",
    "/sys/devices/platform/spx70d0_thermal/temp4_input",
    "/sys/devices/platform/spx70d0_thermal/temp5_input",
    "/sys/devices/platform/spx70d0_thermal/temp6_input",
    "/sys/devices/platform/spx70d0_thermal/temp7_input",
    "/sys/devices/platform/spx70d0_168f_psu/psu1_temp1_input",
    "/sys/devices/platform/spx70d0_168f_psu/psu2_temp1_input"
};


/* Static values */
static onlp_thermal_info_t linfo[] = {
    { }, /* Not used */
	{ { ONLP_THERMAL_ID_CREATE(THERMAL_CPU_CORE), "CPU Core", 0},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, ONLP_THERMAL_THRESHOLD_INIT_DEFAULTS
    },
    { { ONLP_THERMAL_ID_CREATE(THERMAL_1_ON_MAIN_BROAD_AMBIENT), "Chassis Thermal Sensor 1 (TMP1075#0 Ambient)", 0 },
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, ONLP_THERMAL_THRESHOLD_INIT_DEFAULTS
    },
    { { ONLP_THERMAL_ID_CREATE(THERMAL_2_ON_MAIN_BROAD_100G_PORTS), "Chassis Thermal Sensor 2 (TMP1075#1 Hot Spot nearby 100G ports)", 0 },
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, ONLP_THERMAL_THRESHOLD_INIT_DEFAULTS
    },
    { { ONLP_THERMAL_ID_CREATE(THERMAL_3_ON_PON_BROAD_PSU_IN), "Chassis Thermal Sensor 3 (TMP1075#2 PSU IN)", 0 },
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, ONLP_THERMAL_THRESHOLD_INIT_DEFAULTS
    },
    { { ONLP_THERMAL_ID_CREATE(THERMAL_4_ON_PON_BROAD_PON_MAC), "Chassis Thermal Sensor 4 (TMP1075#3 Hot Spot nearby PON MAC)", 0 },
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, ONLP_THERMAL_THRESHOLD_INIT_DEFAULTS
    },
    { { ONLP_THERMAL_ID_CREATE(THERMAL_5_ON_PON_BROAD_PON_PORTS), "Chassis Thermal Sensor 5 (TMP1075#4 Hot Spot nearby PON Ports)", 0 },
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, ONLP_THERMAL_THRESHOLD_INIT_DEFAULTS
    },
    { { ONLP_THERMAL_ID_CREATE(THERMAL_6_ON_MAIN_BROAD_TMP435_local), "Chassis Thermal Sensor 6 (TMP435_local)", 0 },
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, ONLP_THERMAL_THRESHOLD_INIT_DEFAULTS
    },
    { { ONLP_THERMAL_ID_CREATE(THERMAL_7_ON_MAIN_BROAD_TMP435_remote), "Chassis Thermal Sensor 7 (TMP435_remote)", 0 },
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, ONLP_THERMAL_THRESHOLD_INIT_DEFAULTS
    },
    { { ONLP_THERMAL_ID_CREATE(THERMAL_8_psu1_temp1), "Chassis Thermal Sensor 8 (psu1_temp1)", ONLP_PSU_ID_CREATE(PSU1_ID) },
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, ONLP_THERMAL_THRESHOLD_INIT_DEFAULTS
    },
    { { ONLP_THERMAL_ID_CREATE(THERMAL_9_psu2_temp1), "Chassis Thermal Sensor 9 (psu2_temp1)", ONLP_PSU_ID_CREATE(PSU2_ID) },
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, ONLP_THERMAL_THRESHOLD_INIT_DEFAULTS
    }
};


/*
 * This will be called to intiialize the thermali subsystem.
 */
int
onlp_thermali_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);
    return ONLP_STATUS_OK;
}


/*
 * Retrieve the information structure for the given thermal OID.
 *
 * If the OID is invalid, return ONLP_E_STATUS_INVALID.
 * If an unexpected error occurs, return ONLP_E_STATUS_INTERNAL.
 * Otherwise, return ONLP_STATUS_OK with the OID's information.
 *
 * Note -- it is expected that you fill out the information
 * structure even if the sensor described by the OID is not present.
 */
int
onlp_thermali_info_get(onlp_oid_t id, onlp_thermal_info_t *info)
{
    DIAG_PRINT("%s, id=%d", __FUNCTION__, id);
    int local_id;

    VALIDATE(id);

    local_id = ONLP_OID_ID_GET(id);

    /* Set the onlp_oid_hdr_t and capabilities */
    *info = linfo[local_id];

    return onlp_file_read_int(&info->mcelsius, devfiles__[local_id]);
}


