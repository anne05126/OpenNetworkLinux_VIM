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
#include <onlp/platformi/sysi.h>
#include <onlp/platformi/ledi.h>
#include <onlp/platformi/thermali.h>
#include <onlp/platformi/fani.h>
#include <onlp/platformi/psui.h>
#include <onlp/platformi/sfpi.h>
#include <onlplib/file.h>
#include <onlplib/i2c.h>

#include "x86_64_extremenetworks_7830_32ce_8de_int.h"
#include "x86_64_extremenetworks_7830_32ce_8de_log.h"
#include "platform_lib.h"
#include "vimi.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG                               0
#define SYSTEM_CPLD_MAX_STRLEN              8

#define PORT_CPLD_REVISION_PATH		   	    "/sys/bus/i2c/devices/%s"
#define POWER_CPLD_REVISION_PATH		    "/sys/bus/platform/devices/7830_pwr_cpld/%s"
#define VIM_POWER_CPLD_REVISION_PATH		"/sys/bus/i2c/devices/%d-005c/vim%d_cpld_version"
#define VIM_PORT_CPLD_REVISION_PATH		   	"/sys/bus/i2c/devices/%d-0058/vim%d_cpld_version"
#define SYS_CPLD_REVISION_PATH		   	    "/sys/bus/i2c/devices/0-006e/version"

#define PLATFORM_STRING "x86-64-extremenetworks-7830-32ce-8de-r0"

typedef struct cpld_version {
	char *cpld_path_format;
	char *attr_name;
	int   version;
	char *description;
} cpld_version_t;

typedef struct vim_cpld_version {
	char *cpld_path_format;
	int   i2c_bus_id;
    int   vim_id;
	int   version;
	char *description;
} vim_cpld_version_t;

void vim_history_status_init(void)
{
    history_vim_status.history_vim1_present = VIM_NOT_PRESENT;      /* 0 = present, 1 = not present */
    history_vim_status.history_vim2_present = VIM_NOT_PRESENT;
    history_vim_status.history_vim1_board_id = VIM_NONE;            /* 0 = 8DE, 1 = 16CE, 2 = 24CE, 3 = 24YE, 4 = VIM_NONE */
    history_vim_status.history_vim2_board_id = VIM_NONE;
}

const char*
onlp_sysi_platform_get(void)
{
    DIAG_PRINT("%s, platform string: %s", __FUNCTION__, PLATFORM_STRING);
    return PLATFORM_STRING;
}

int
onlp_sysi_onie_data_get(uint8_t** data, int* size)
{
    DIAG_PRINT("%s ONIE_EEPROM_PATH: %s", __FUNCTION__, ONIE_EEPROM_PATH);
    uint8_t* rdata = aim_zmalloc(256);

    if (onlp_file_read(rdata, 256, size, ONIE_EEPROM_PATH) == ONLP_STATUS_OK)
    {
        if(*size == 256)
        {
            *data = rdata;
            return ONLP_STATUS_OK;
        }
    }

    aim_free(rdata);
    *size = 0;
    return ONLP_STATUS_E_INTERNAL;
}

void onlp_sysi_onie_data_free(uint8_t *data)
{
    DIAG_PRINT("%s", __FUNCTION__);
    if (data)
        aim_free(data);
}

int
onlp_sysi_platform_info_get(onlp_platform_info_t *pi)
{
    DIAG_PRINT("%s", __FUNCTION__);

	int i, ret;
	cpld_version_t cplds[] = { { PORT_CPLD_REVISION_PATH, "12-0057/version", 0, "Port-CPLD#0"},
				   			   { PORT_CPLD_REVISION_PATH, "15-0057/version", 0, "Port-CPLD#1"},
				   			   { POWER_CPLD_REVISION_PATH, "pwr_cpld_ver", 0, "Power-CPLD"} };

	/* Read CPLD version
	 */
	for (i = 0; i < AIM_ARRAYSIZE(cplds); i++) {
		ret = onlp_file_read_int(&cplds[i].version,
					 			cplds[i].cpld_path_format,
					 			cplds[i].attr_name);

		if (ret < 0) {
			AIM_LOG_ERROR("Unable to read version from CPLD(%s)\r\n", cplds[i].attr_name);
			return ONLP_STATUS_E_INTERNAL;
		}
	}

	pi->cpld_versions = aim_fstrdup("%s:%02x, %s:%02x, %s:%02x",
									cplds[0].description,
									cplds[0].version,
									cplds[1].description,
									cplds[1].version,
									cplds[2].description,
									cplds[2].version);

    return ONLP_STATUS_OK;
}

void
onlp_sysi_platform_info_free(onlp_platform_info_t *pi)
{
    DIAG_PRINT("%s", __FUNCTION__);
}

int
onlp_sysi_oids_get(onlp_oid_t *table, int max)
{
    DIAG_PRINT("%s, max:%d", __FUNCTION__, max);
    onlp_oid_t *e = table;
    memset(table, 0, max * sizeof(onlp_oid_t));
    int i;

    uint32_t oid = 0;

    /* PSUs */
    for (i = 1; i <= CHASSIS_PSU_COUNT; i++)
    {
        oid = ONLP_PSU_ID_CREATE(i);
        *e++ = oid;
        DIAG_PRINT("PSU#%d oid:%d", i, oid);
    }

    /* LEDs */
    for (i = 1; i <= CHASSIS_LED_COUNT; i++)
    {
        oid = ONLP_LED_ID_CREATE(i);
        *e++ = oid;
        DIAG_PRINT("LED#%d oid:%d", i, oid);
    }

    /* Thermal sensors */
    for (i = 1; i <= CHASSIS_THERMAL_COUNT; i++)
    {
        oid = ONLP_THERMAL_ID_CREATE(i);
        *e++ = oid;
        DIAG_PRINT("THERMAL#%d oid:%d", i, oid);
    }

    /* Fans */
    for (i = 1; i <= CHASSIS_FAN_COUNT; i++)
    {
        oid = ONLP_FAN_ID_CREATE(i);
        *e++ = oid;
        DIAG_PRINT("FAN#%d oid:%d", i, oid);
    }

    return 0;
}

int onlp_sysi_platform_manage_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);
    return 0;
}

int
onlp_sysi_platform_manage_fans(void)
{
    /* Fan management is controlled by BMC automatically. */
    return ONLP_STATUS_OK;
}

int
onlp_sysi_platform_manage_leds(void)
{
    /* LED management is controlled by BMC automatically. */
    return ONLP_STATUS_OK;
}

int
onlp_sysi_platform_manage_vims(void)
{
    int vim1_present = -1;
    int vim2_present = -1;
    int vim1_board_id = -1;
    int vim2_board_id = -1;
    int vim1_power_good = -1;
    int vim2_power_good = -1;
    int rv = -1;

    /*
     * STEP 1
     * Get VIM present from system CPLD
     * 0 = present, 1 = not present
     */
    vim1_present = onlp_vimi_present_get(VIM1_ID);
    if (vim1_present < 0)
    {
        AIM_LOG_ERROR("Get VIM 1 present failed(%d)\r\n", vim1_present);
        return ONLP_STATUS_E_INTERNAL;
    }

    vim2_present = onlp_vimi_present_get(VIM2_ID);
    if (vim2_present < 0)
    {
        AIM_LOG_ERROR("Get VIM 2 present failed(%d)\r\n", vim2_present);
        return ONLP_STATUS_E_INTERNAL;
    }

    //AIM_SYSLOG_INFO("[SYSLOG_INFO]",
    //                "[SYSLOG_INFO]",
    //                "[SYSLOG_INFO] Get VIM present: VIM1(%d) %s, VIM2(%d) %s", vim1_present, (vim1_present? "NOT PRESENT" : "PRESENT"), vim2_present, (vim2_present? "NOT PRESENT" : "PRESENT"));

    /*
     * STEP 2
     * Alpha detects the insertion/removal of the VIM, then call Extreme VIM handler function
     */
    /* Detects the insertion/removal of the VIM 1 */
    if (history_vim_status.history_vim1_present == VIM_NOT_PRESENT && vim1_present == VIM_PRESENT)
    {
        /* It is detected that a VIM is inserted into VIM 1 slot */
        AIM_SYSLOG_INFO("[SYSLOG_INFO]",
                        "[SYSLOG_INFO]",
                        "[SYSLOG_INFO] Insert VIM 1 (old: %s, new: %s)", (history_vim_status.history_vim1_present ? "NOT PRESENT" : "PRESENT"), (vim1_present ? "NOT PRESENT" : "PRESENT"));

        /*
         * Get VIM power good from system CPLD
         * 0 = Power fail, 1 = Power good
         */
        vim1_power_good = onlp_vimi_power_good_get(VIM1_ID);
        if (vim1_power_good < 0)
        {
            AIM_LOG_ERROR("Get VIM 1 power good failed(%d)\r\n", vim1_power_good);
            return ONLP_STATUS_E_INTERNAL;
        }

        /*
         * Enable VIM power
         * 0 = Power fail, 1 = Power good
         */
        if ((vim1_present == VIM_PRESENT) && (vim1_power_good == VIM_POWER_FAIL))
        {
            /* Enable VIM power */
            onlp_vimi_power_control(ON, VIM1_ID);
            sleep(1);
        }

        /*
         * Get VIM board id from VIM CPLD by BMC
         * 0 = 8DE, 1 = 16CE, 2 = 24CE, 3 = 24YE, 4 = VIM_NONE
         */
        vim1_board_id = onlp_vimi_board_id_get(VIM1_ID);
        if (vim1_board_id < 0)
        {
            AIM_LOG_ERROR("Get VIM 1 board id failed(%d)\r\n", vim1_board_id);
            return ONLP_STATUS_E_INTERNAL;
        }

        /* Call execute_update_vim_i2c_tree() to execute Extreme VIM handler function(extreme_update_vim_i2c_tree_sample) */
        rv = execute_update_vim_i2c_tree(VIM1_ID, INSERT, vim1_board_id);
        if (rv < 1)
        {
            AIM_LOG_ERROR("Insert VIM %d (board_id=%d) failed(%d)\r\n", VIM1_ID, vim1_board_id, rv);
            return ONLP_STATUS_E_INTERNAL;
        }

        /* Update the history of VIM 1 present and board id */
        history_vim_status.history_vim1_present = vim1_present;
        history_vim_status.history_vim1_board_id = vim1_board_id;
    }
    else if (history_vim_status.history_vim1_present == VIM_PRESENT && vim1_present == VIM_NOT_PRESENT)
    {
        /* It is detected that a VIM is removed from VIM 1 slot */
        AIM_SYSLOG_INFO("[SYSLOG_INFO]",
                        "[SYSLOG_INFO]",
                        "[SYSLOG_INFO] Remove VIM 1 (old: %s, new: %s)", (history_vim_status.history_vim1_present ? "NOT PRESENT" : "PRESENT"), (vim1_present ? "NOT PRESENT" : "PRESENT"));

        /* Disable VIM power */
        onlp_vimi_power_control(OFF, VIM1_ID);

        /* Call execute_update_vim_i2c_tree() to execute Extreme VIM handler function */
        rv = execute_update_vim_i2c_tree(VIM1_ID, REMOVE, history_vim_status.history_vim1_board_id);
        if (rv < 1)
        {
            AIM_LOG_ERROR("Removed VIM %d (board_id=%d) failed(%d)\r\n", VIM1_ID, history_vim_status.history_vim1_board_id, rv);
            return ONLP_STATUS_E_INTERNAL;
        }

        /* Update the history of VIM 1 present and board id */
        history_vim_status.history_vim1_present = VIM_NOT_PRESENT;
        history_vim_status.history_vim1_board_id = VIM_NONE;
    }

    /* Detects the insertion/removal of the VIM 2 */
    if (history_vim_status.history_vim2_present == VIM_NOT_PRESENT && vim2_present == VIM_PRESENT)
    {
        /* It is detected that a VIM is inserted into VIM 2 slot */
        AIM_SYSLOG_INFO("[SYSLOG_INFO]",
                        "[SYSLOG_INFO]",
                        "[SYSLOG_INFO] Insert VIM 2 (old: %s, new: %s)", (history_vim_status.history_vim2_present ? "NOT PRESENT" : "PRESENT"), (vim2_present ? "NOT PRESENT" : "PRESENT"));

        /*
         * Get VIM power good from system CPLD
         * 0 = Power fail, 1 = Power good
         */
        vim2_power_good = onlp_vimi_power_good_get(VIM2_ID);
        if (vim2_power_good < 0)
        {
            AIM_LOG_ERROR("Get VIM 2 power good failed(%d)\r\n", vim2_power_good);
            return ONLP_STATUS_E_INTERNAL;
        }

        /*
         * Enable VIM power
         * 0 = Power fail, 1 = Power good
         */
        if ((vim2_present == VIM_PRESENT) && (vim2_power_good == VIM_POWER_FAIL))
        {
            /* Enable VIM power */
            onlp_vimi_power_control(ON, VIM2_ID);
            sleep(1);
        }

        /*
         * Get VIM board id from VIM CPLD by BMC
         * 0 = 8DE, 1 = 16CE, 2 = 24CE, 3 = 24YE, 4 = VIM_NONE
         */
        vim2_board_id = onlp_vimi_board_id_get(VIM2_ID);
        if (vim2_board_id < 0)
        {
            AIM_LOG_ERROR("Get VIM 2 board id failed(%d)\r\n", vim2_board_id);
            return ONLP_STATUS_E_INTERNAL;
        }

        /* Call execute_update_vim_i2c_tree() to execute Extreme VIM handler function */
        rv = execute_update_vim_i2c_tree(VIM2_ID, INSERT, vim2_board_id);
        if (rv < 1)
        {
            AIM_LOG_ERROR("Insert VIM %d (board_id=%d) failed(%d)\r\n", VIM2_ID, vim2_board_id, rv);
            return ONLP_STATUS_E_INTERNAL;
        }

        /* Update the history of VIM 2 present and board id */
        history_vim_status.history_vim2_present = vim2_present;
        history_vim_status.history_vim2_board_id = vim2_board_id;
    }
    else if (history_vim_status.history_vim2_present == VIM_PRESENT && vim2_present == VIM_NOT_PRESENT)
    {
        /* It is detected that a VIM is removed from VIM 2 slot */
        AIM_SYSLOG_INFO("[SYSLOG_INFO]",
                        "[SYSLOG_INFO]",
                        "[SYSLOG_INFO] Remove VIM 2 (old: %s, new: %s)", (history_vim_status.history_vim2_present ? "NOT PRESENT" : "PRESENT"), (vim2_present ? "NOT PRESENT" : "PRESENT"));

        /* Disable VIM power */
        onlp_vimi_power_control(OFF, VIM2_ID);

        /* Call execute_update_vim_i2c_tree() to execute Extreme VIM handler function */
        rv = execute_update_vim_i2c_tree(VIM2_ID, REMOVE, history_vim_status.history_vim2_board_id);
        if (rv < 1)
        {
            AIM_LOG_ERROR("Removed VIM %d (board_id=%d) failed(%d)\r\n", VIM2_ID, history_vim_status.history_vim2_board_id, rv);
            return ONLP_STATUS_E_INTERNAL;
        }

        /* Update the history of VIM 2 present and board id */
        history_vim_status.history_vim2_present = VIM_NOT_PRESENT;
        history_vim_status.history_vim2_board_id = VIM_NONE;
    }

    return ONLP_STATUS_OK;
}

int
onlp_sysi_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);

    vim_history_status_init();
    onlp_vimi_init();
    return ONLP_STATUS_OK;
}

int onlp_sysi_debug_diag_sfp_status(void)
{
    int i = 0;
    int status = 0;
    for (i = 0; i < (NUM_OF_SFP_PORT + NUM_OF_IOBM_PORT); i++)
    {
        status = onlp_sfpi_is_present(i);
        printf("SFP#%d \n", i+1);
        printf("Status: 0x%x [%s]\n", status,
               (status) ? "PRESENT" : "NOT PRESENT");
    }

    /* VIM card sfp status */
    int vim_end_index = onlp_vimi_get_vim_end_index();
    for (i = VIM_START_INDEX; i < vim_end_index; i++)
    {
        status = onlp_sfpi_is_present(i);
        printf("VIM SFP#%d \n", i+1);
        printf("Status: 0x%x [%s]\n", status,
               (status) ? "PRESENT" : "NOT PRESENT");
    }
    return 0;
}

int onlp_sysi_debug_diag_fan_status(void)
{
    int oid = 0;
    int i = 0;
    uint32_t status = 0;

    /*
     *  Get each fan status
     */
    for (i = 1; i <= CHASSIS_FAN_COUNT; i++)
    {
        onlp_fan_info_t fan_info;
        oid = ONLP_FAN_ID_CREATE(i);
        if (onlp_fani_info_get(oid, &fan_info) != ONLP_STATUS_OK)
        {
            AIM_LOG_ERROR("Unable to get fan(%d) status\r\n", i);
            return ONLP_STATUS_E_INTERNAL;
        }
        status = fan_info.status;
        printf("FAN#%d oid:%d\n", i, oid);
        printf("Status: 0x%x [%s %s]\n", status,
               (status & ONLP_FAN_STATUS_PRESENT) ? "PRESENT" : "NOT PRESENT",
               (status & ONLP_FAN_STATUS_FAILED) ? "FAILED" : "");
    }
    return 0;

}

int onlp_sysi_debug_diag_led(void)
{
    int i = 0;
    int value = 0;
    int vim_id = 0;
    char chan_led_command[64];
    char vim_pwr_led_command[128];

    printf("POWER o     STATUS  o     FAN o     PSU o     SECURITY o   \n");
    printf("\n");

    printf("[Stop platform manage ...]\n");

    #if 0
    diag_debug_pause_platform_manage_on();
    #else
    /* Set BMC Test Mode */
    /* 0x00: Disabled
       0x01: Enabled
       During test mode, the BMC will temporarily suspend the system control function as follows:
       1. FAN/PSU control
       2. Fan algorithm/Thermal profile
       3. Sensor polling
    */
    system("ipmitool raw 0x34 0xf1 0x01");
    #endif

    sleep(1);

    printf("[Set All LED to OFF ...]\n");
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_PWR), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_FAN), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_SEC), ONLP_LED_MODE_OFF);
    printf("<Press Enter Key to Continue>\n");
    getchar();

    /* POWER LED */
    printf("[Set POWER LED to ONLP_LED_MODE_GREEN ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PWR), ONLP_LED_MODE_GREEN);
    printf("<Press Enter Key to Continue>\n");
    getchar();

    /* STATUS LED */
    printf("[Set STATUS LED to ONLP_LED_MODE_GREEN ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_GREEN);
    printf("<Press Enter Key to Continue>\n");
    getchar();

    printf("[Set STATUS LED to ONLP_LED_MODE_ORANGE ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_ORANGE);
    printf("<Press Enter Key to Continue>\n");
    getchar();

	printf("[Set STATUS LED to ONLP_LED_MODE_BLINKING(Blinking Amber-Green) ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_BLINKING);
    printf("<Press Enter Key to Continue>\n");
    getchar();

	/* FAN LED */
    printf("[Set FAN LED to ONLP_LED_MODE_GREEN ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_FAN), ONLP_LED_MODE_GREEN);
    printf("<Press Enter Key to Continue>\n");
    getchar();

    printf("[Set FAN LED to ONLP_LED_MODE_ORANGE ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_FAN), ONLP_LED_MODE_ORANGE);
    printf("<Press Enter Key to Continue>\n");
    getchar();

	printf("[Set FAN LED to ONLP_LED_MODE_BLINKING(Blinking Amber-Green) ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_FAN), ONLP_LED_MODE_BLINKING);
    printf("<Press Enter Key to Continue>\n");
    getchar();

    /* PSU LED */
    printf("[Set PSU LED to ONLP_LED_MODE_GREEN ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_GREEN);
    printf("<Press Enter Key to Continue>\n");
    getchar();
    printf("[Set PSU LED to ONLP_LED_MODE_GREEN_BLINKING ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_GREEN_BLINKING);
    printf("<Press Enter Key to Continue>\n");
    getchar();
    printf("[Set PSU LED to ONLP_LED_MODE_ORANGE ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_ORANGE);
    printf("<Press Enter Key to Continue>\n");
    getchar();
    printf("[Set PSU LED to ONLP_LED_MODE_ORANGE_BLINKING ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_ORANGE_BLINKING);
    printf("<Press Enter Key to Continue>\n");
    getchar();
	printf("[Set PSU LED to ONLP_LED_MODE_BLINKING(Blinking Amber-Green) ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_BLINKING);
    printf("<Press Enter Key to Continue>\n");
    getchar();

    /* SECURITY LED */
    printf("[Set SEC LED to ONLP_LED_MODE_BLUE ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_SEC), ONLP_LED_MODE_BLUE);
    printf("<Press Enter Key to Continue>\n");
    getchar();

    printf("[Set All LED to OFF ...]\n");
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_PWR), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_FAN), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_SEC), ONLP_LED_MODE_OFF);
    printf("<Press Enter Key to Continue>\n");
    getchar();

    printf("[Reverted power, fan, security, and PSU LEDs to hardware control ...]\n");
    onlp_file_write_int(0, LED_PATH_I2C, "led_control");
    onlp_file_write_int(0, LED_PATH, "led_control");
    printf("<Press Enter Key to Continue>\n");
    getchar();

    /* Channel LED */
    for (i = 0; i <= 8; i++)
    {
        printf("[Set Channel LED to %d ...]\n", i);
        sprintf(chan_led_command, "i2cset -y -f 0 0x5f 0x53 0x0%d", i);
        system(chan_led_command);
        if (onlp_file_read_int(&value, LED_PATH_I2C, "chan_sel_counter") < 0)
        {
            AIM_LOG_ERROR("[CPU] Unable to read status from file "LED_PATH, "chan_sel_counter");
        }
        printf("Push Button Counter: %d\n", value);
        printf("<Press Enter Key to Continue>\n");
        getchar();
    }

    /* VIM Power LED */
    for (vim_id = 1; vim_id <= VIM_ID_MAX; vim_id++)
    {
        for (i = 3; i >= 0; i--)
        {
            printf("[Set VIM#%d Power LED to %d ...]\n", vim_id, i);
            sprintf(vim_pwr_led_command, "i2cset -y -f 0 0x77 0x%d;i2cset -y -f 0 0x76 0x8;i2cset -y -f 0 0x5c 0x40 0x%02x", vim_id, i << 3 | 4);
            system(vim_pwr_led_command);
            printf("<Press Enter Key to Continue>\n");
            getchar();
        }
        sprintf(vim_pwr_led_command, "i2cset -y -f 0 0x77 0x%d;i2cset -y -f 0 0x76 0x8;i2cset -y -f 0 0x5c 0x40 0x0", vim_id);
        system(vim_pwr_led_command);
    }

    printf("[Restart platform manage ...]\n");
    onlp_ledi_init();

    #if 0
    diag_debug_pause_platform_manage_off();
    #else
    /* Disable BMC Test Mode */
    system("ipmitool raw 0x34 0xf1 0x00");
    #endif

    return 0;
}

int onlp_sysi_debug_vim_eeprom(int vim_id)
{
    uint8_t *data = NULL;
    int rv = 0;
    int vim_present = 0;

    vim_present = onlp_vimi_present_get(vim_id);

    if(vim_present == VIM_NOT_PRESENT)
    {
        AIM_LOG_ERROR("vim(%d) not present\r\n", vim_id);
        return ONLP_STATUS_OK;
    }

    data = aim_zmalloc(256);
    if ((rv = onlp_vimi_eeprom_read(vim_id, data)) < 0)
    {
        aim_printf(&aim_pvs_stdout, "Error reading eeprom: %{onlp_status}\n");
    }
    else
    {
        aim_printf(&aim_pvs_stdout, "dump eeprom:\n%{data}\n", data, 256);
    }
    aim_free(data);
    data = NULL;

    return rv;
}

#define SFP_DIAG_OFFSET             118     /* EXTENDED MODULE CONTROL/STATUS BYTES  in SFF-8472 standard */
#define SFP28_DIAG_OFFSET           148     /* EXTENDED MODULE CONTROL/STATUS BYTES  in SFF-8472 standard */
#define QSFP28_DIAG_PAGE_SELECT     127     /* page select byte in SFF-8636 */
#define QSFP28_DIAG_OFFSET          96      /* the reserved bytes 94~97 of page 00h support r/w in SFF-8636 */
#define QSFP28_DIAG_PAGE	        0x0	    /* the reserved bytes 94~97 of page 00h support r/w in SFF-8636 */
#define QSFPDD_DIAG_PAGE_SELECT     127     /* page select byte in QSFP-DD-CMIS standard */
#define QSFPDD_DIAG_OFFSET          222     /* the reserved bytes 214~223 of page 13h support r/w in QSFP-DD-CMIS standard */
#define QSFPDD_DIAG_PAGE	        0x13	/* the reserved bytes 214~223 of page 13h support r/w in QSFP-DD-CMIS standard */
#define SFP_DIAG_PATTEN_B 0xAA
#define SFP_DIAG_PATTEN_W 0xABCD

int onlp_sysi_debug_diag_sfp(int index, int offset, int page, int page_select)
{
    uint8_t *data = NULL;
    int rv = 0;

    uint8_t org_b = 0;
    uint16_t org_w = 0;
    uint8_t temp_b = 0;
    uint16_t temp_w = 0;
	uint8_t page_org_b = 0;

    int addr = 0;

    data = aim_zmalloc(256);
    if ((rv = onlp_sfpi_eeprom_read(index, data)) < 0)
    {

        aim_printf(&aim_pvs_stdout, "Error reading eeprom: %{onlp_status}\n");
    }
    else
    {
        aim_printf(&aim_pvs_stdout, "dump eeprom:\n%{data}\n", data, 256);
    }
    aim_free(data);
    data = NULL;

    if (index >= SFP_START_INDEX && index < (SFP_START_INDEX + NUM_OF_SFP_PORT))
    {
		addr = SFP_PLUS_EEPROM_I2C_ADDR;
        if (offset == -1 && page == -1 && page_select == -1)
        {
            if (IS_QSFPDD_PORT(index))
            {
                /* QSFP-DD 400G */
                page_select = QSFPDD_DIAG_PAGE_SELECT;
                offset = QSFPDD_DIAG_OFFSET;
                page = QSFPDD_DIAG_PAGE;
            }
            else if (IS_QSFP28_PORT(index))
            {
                /* QSFP28 100G */
                page_select = QSFP28_DIAG_PAGE_SELECT;
                offset = QSFP28_DIAG_OFFSET;
                page = QSFP28_DIAG_PAGE;
            }
            else
            {
                goto DONE;
            }
        }
    }

    /* BYTE */
    printf("Read/Write byte test...\n");

	if (IS_QSFP_PORT(index))
    {
        page_org_b = onlp_sfpi_dev_readb(index, addr, page_select);
    	if (page_org_b < 0)
    	{
        	printf("Error, read failed!\n");
        	goto DONE;
    	}

		rv = onlp_sfpi_dev_writeb(index, addr, page_select, page);
    	if (rv < 0)
    	{
        	printf("Error, write failed!\n");
        	goto RESTORE_PAGE;
    	}
    	sleep(2);
    	temp_b = onlp_sfpi_dev_readb(index, addr, page_select);
    	if (temp_b < 0)
    	{
        	printf("Error, read failed!\n");
        	goto RESTORE_PAGE;
    	}
    	if (temp_b != page)
    	{
        	printf("Error, can not change page!\n");
        	goto RESTORE_PAGE;
    	}
    }

    org_b = onlp_sfpi_dev_readb(index, addr, offset);
    if (org_b < 0)
    {
        printf("Error, read failed!\n");
        if (IS_QSFP_PORT(index))
        {
		    goto RESTORE_PAGE;
        }
        else
        {
            goto DONE;
        }
    }

    rv = onlp_sfpi_dev_writeb(index, addr, offset, SFP_DIAG_PATTEN_B);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto RESTORE_EEPROM_B;
    }
    sleep(2);
    temp_b = onlp_sfpi_dev_readb(index, addr, offset);
    if (temp_b < 0)
    {
        printf("Error, read failed!\n");
        goto RESTORE_EEPROM_B;
    }
    if (temp_b != SFP_DIAG_PATTEN_B)
    {
        printf("Error, mismatch!\n");
        goto RESTORE_EEPROM_B;
    }
    rv = onlp_sfpi_dev_writeb(index, addr, offset, org_b);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto RESTORE_EEPROM_B;
    }
    sleep(2);

    /* WORD */
    printf("Read/Write word test...\n");
    org_w = onlp_sfpi_dev_readw(index, addr, offset);
    if (org_w < 0)
    {
        printf("Error, read failed!\n");
        if (IS_QSFP_PORT(index))
        {
		    goto RESTORE_PAGE;
        }
        else
        {
            goto DONE;
        }
    }
    rv = onlp_sfpi_dev_writew(index, addr, offset, SFP_DIAG_PATTEN_W);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto RESTORE_EEPROM_W;
    }
    sleep(2);
    temp_w = onlp_sfpi_dev_readw(index, addr, offset);
    if (temp_w < 0)
    {
        printf("Error, read failed!\n");
        goto RESTORE_EEPROM_W;
    }
    if (temp_w != SFP_DIAG_PATTEN_W)
    {
        printf("Error, mismatch!\n");
        goto RESTORE_EEPROM_W;
    }
    rv = onlp_sfpi_dev_writew(index, addr, offset, org_w);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto RESTORE_EEPROM_W;
    }

	if (IS_QSFP_PORT(index))
    {
		goto RESTORE_PAGE;
    }

DONE:
    return 0;
RESTORE_PAGE:
    rv = onlp_sfpi_dev_writeb(index, addr, page_select, page_org_b);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
    }
    goto DONE;
RESTORE_EEPROM_B:
    rv = onlp_sfpi_dev_writeb(index, addr, offset, org_b);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
    }
    if (IS_QSFP_PORT(index))
    {
		goto RESTORE_PAGE;
    }
    else
    {
        goto DONE;
    }
RESTORE_EEPROM_W:
    rv = onlp_sfpi_dev_writew(index, addr, offset, org_w);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
    }
    if (IS_QSFP_PORT(index))
    {
		goto RESTORE_PAGE;
    }
    else
    {
        goto DONE;
    }
}

int onlp_sysi_debug_diag_sfp_dom(int index)
{
    uint8_t *data = NULL;
    int rv = 0;

    data = aim_zmalloc(256);
    if ((rv = onlp_sfpi_dom_read(index, data)) < 0)
    {

        aim_printf(&aim_pvs_stdout, "Error reading dom eeprom: %{onlp_status}\n", rv);
    }
    else
    {
        aim_printf(&aim_pvs_stdout, "dump DOM eeprom:\n%{data}\n", data, 256);
    }
    aim_free(data);
    data = NULL;

    return 0;
}

int onlp_sysi_debug_diag_sfp_ctrl(int index)
{
    int val = 0;

    /* ONLP_SFP_CONTROL_RESET (Read and Write)*/
    printf("[Option: %d(%s)...Set/Get]\n", ONLP_SFP_CONTROL_RESET, sfp_control_to_str(ONLP_SFP_CONTROL_RESET));
    printf("[Set %s... to 0]\n", sfp_control_to_str(ONLP_SFP_CONTROL_RESET));
    onlp_sfpi_control_set(index, ONLP_SFP_CONTROL_RESET, 0);
    sleep(1);
    printf("[Get %s... ]\n", sfp_control_to_str(ONLP_SFP_CONTROL_RESET));
    onlp_sfpi_control_get(index, ONLP_SFP_CONTROL_RESET, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    printf("[Set %s... to 1]\n", sfp_control_to_str(ONLP_SFP_CONTROL_RESET));
    onlp_sfpi_control_set(index, ONLP_SFP_CONTROL_RESET, 1);
	sleep(1);
    printf("[Get %s... ]\n", sfp_control_to_str(ONLP_SFP_CONTROL_RESET));
    onlp_sfpi_control_get(index, ONLP_SFP_CONTROL_RESET, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    /* ONLP_SFP_CONTROL_LP_MODE (Read and Write)*/
    printf("[Option: %d(%s)...Set/Get]\n", ONLP_SFP_CONTROL_LP_MODE, sfp_control_to_str(ONLP_SFP_CONTROL_LP_MODE));
    printf("[Set %s... to 1]\n", sfp_control_to_str(ONLP_SFP_CONTROL_LP_MODE));
    onlp_sfpi_control_set(index, ONLP_SFP_CONTROL_LP_MODE, 1);
    sleep(1);
    printf("[Get %s... ]\n", sfp_control_to_str(ONLP_SFP_CONTROL_LP_MODE));
    onlp_sfpi_control_get(index, ONLP_SFP_CONTROL_LP_MODE, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    printf("[Set %s... to 0]\n", sfp_control_to_str(ONLP_SFP_CONTROL_LP_MODE));
    onlp_sfpi_control_set(index, ONLP_SFP_CONTROL_LP_MODE, 0);
    sleep(1);
    printf("[Get %s... ]\n", sfp_control_to_str(ONLP_SFP_CONTROL_LP_MODE));
    onlp_sfpi_control_get(index, ONLP_SFP_CONTROL_LP_MODE, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    /* ONLP_SFP_CONTROL_RX_LOS (Read only)*/
    printf("[Option: %d(%s)...Get]\n", ONLP_SFP_CONTROL_RX_LOS, sfp_control_to_str(ONLP_SFP_CONTROL_RX_LOS));
    printf("[Get %s... ]\n", sfp_control_to_str(ONLP_SFP_CONTROL_RX_LOS));
    onlp_sfpi_control_get(index, ONLP_SFP_CONTROL_RX_LOS, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    /* ONLP_SFP_CONTROL_TX_FAULT (Read only)*/
    printf("[Option: %d(%s)...Get]\n", ONLP_SFP_CONTROL_TX_FAULT, sfp_control_to_str(ONLP_SFP_CONTROL_TX_FAULT));
    printf("[Get %s... ]\n", sfp_control_to_str(ONLP_SFP_CONTROL_TX_FAULT));
    onlp_sfpi_control_get(index, ONLP_SFP_CONTROL_TX_FAULT, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    /* ONLP_SFP_CONTROL_TX_DISABLE (Read and Write)*/
    printf("[Option: %d(%s)...Set/Get]\n", ONLP_SFP_CONTROL_TX_DISABLE, sfp_control_to_str(ONLP_SFP_CONTROL_TX_DISABLE));
    printf("[Set %s... to 1]\n", sfp_control_to_str(ONLP_SFP_CONTROL_TX_DISABLE));
    onlp_sfpi_control_set(index, ONLP_SFP_CONTROL_TX_DISABLE, 1);
    sleep(1);
    printf("[Get %s... ]\n", sfp_control_to_str(ONLP_SFP_CONTROL_TX_DISABLE));
    onlp_sfpi_control_get(index, ONLP_SFP_CONTROL_TX_DISABLE, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    printf("[Set %s... to 0]\n", sfp_control_to_str(ONLP_SFP_CONTROL_TX_DISABLE));
    onlp_sfpi_control_set(index, ONLP_SFP_CONTROL_TX_DISABLE, 0);
    sleep(1);
    printf("[Get %s... ]\n", sfp_control_to_str(ONLP_SFP_CONTROL_TX_DISABLE));
    onlp_sfpi_control_get(index, ONLP_SFP_CONTROL_TX_DISABLE, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();


    /* For VIM 24CE */
    /* ONLP_SFP_CONTROL_RX_LOS_B (Read only)*/
    printf("[Option: %d(%s)...Get]\n", ONLP_SFP_CONTROL_RX_LOS_B, vim_sfp_control_for_b_attr_to_str(ONLP_SFP_CONTROL_RX_LOS_B));
    printf("[Get %s... ]\n", vim_sfp_control_for_b_attr_to_str(ONLP_SFP_CONTROL_RX_LOS_B));
    onlp_sfpi_control_get_for_b_attr(index, ONLP_SFP_CONTROL_RX_LOS_B, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    /* ONLP_SFP_CONTROL_TX_FAULT_B (Read only)*/
    printf("[Option: %d(%s)...Get]\n", ONLP_SFP_CONTROL_TX_FAULT_B, vim_sfp_control_for_b_attr_to_str(ONLP_SFP_CONTROL_TX_FAULT_B));
    printf("[Get %s... ]\n", vim_sfp_control_for_b_attr_to_str(ONLP_SFP_CONTROL_TX_FAULT_B));
    onlp_sfpi_control_get_for_b_attr(index, ONLP_SFP_CONTROL_TX_FAULT_B, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    /* ONLP_SFP_CONTROL_TX_DISABLE_B (Read and Write)*/
    printf("[Option: %d(%s)...Set/Get]\n", ONLP_SFP_CONTROL_TX_DISABLE_B, vim_sfp_control_for_b_attr_to_str(ONLP_SFP_CONTROL_TX_DISABLE_B));
    printf("[Set %s... to 1]\n", vim_sfp_control_for_b_attr_to_str(ONLP_SFP_CONTROL_TX_DISABLE_B));
    onlp_sfpi_control_set_for_b_attr(index, ONLP_SFP_CONTROL_TX_DISABLE_B, 1);
    sleep(1);
    printf("[Get %s... ]\n", vim_sfp_control_for_b_attr_to_str(ONLP_SFP_CONTROL_TX_DISABLE_B));
    onlp_sfpi_control_get_for_b_attr(index, ONLP_SFP_CONTROL_TX_DISABLE_B, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    printf("[Set %s... to 0]\n", vim_sfp_control_for_b_attr_to_str(ONLP_SFP_CONTROL_TX_DISABLE_B));
    onlp_sfpi_control_set_for_b_attr(index, ONLP_SFP_CONTROL_TX_DISABLE_B, 0);
    sleep(1);
    printf("[Get %s... ]\n", vim_sfp_control_for_b_attr_to_str(ONLP_SFP_CONTROL_TX_DISABLE_B));
    onlp_sfpi_control_get_for_b_attr(index, ONLP_SFP_CONTROL_TX_DISABLE_B, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    return 0;
}

/* LED Control bit debug function */
int onlp_sysi_debug_diag_led_control_bit(void)
{
    int value = 0;
    int present = 0, board_id = 0;
    int vim_id = 0;
    int cpld_bus_id = 0;

    /* Set BMC Test Mode */
    /* 0x00: Disabled
       0x01: Enabled
       During test mode, the BMC will temporarily suspend the system control function as follows:
       1. FAN/PSU control
       2. Fan algorithm/Thermal profile
       3. Sensor polling
    */
    system("ipmitool raw 0x34 0xf1 0x01");

    /* LED Control Register 0 */
    if (onlp_file_read_int(&value, LED_PATH_I2C, "led_control") < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "LED_PATH_I2C, "led_control");
        return ONLP_STATUS_E_INTERNAL;
    }
    printf("Control bit on LED Control Register 0: %d \n", value);

    /* LED Control Register 1 */
    if (onlp_file_read_int(&value, LED_PATH, "led_control") < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "LED_PATH, "led_control");
        return ONLP_STATUS_E_INTERNAL;
    }
    printf("Control bit on LED Control Register 1: %d \n", value);

    /* Port LED on-off control on Port CPLD#1 */
    if (onlp_file_read_int(&value, PORT_CPLD1_PATH, "cpld_port_led_enable_1") < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "PORT_CPLD1_PATH, "cpld_port_led_enable_1");
        return ONLP_STATUS_E_INTERNAL;
    }
    printf("Port LED on-off control on Port CPLD#1: %d \n", value);

    /* Port LED on-off control on Port CPLD#2 */
    if (onlp_file_read_int(&value, PORT_CPLD2_PATH, "cpld_port_led_enable_2") < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "PORT_CPLD2_PATH, "cpld_port_led_enable_2");
        return ONLP_STATUS_E_INTERNAL;
    }
    printf("Port LED on-off control on Port CPLD#2: %d \n", value);


    for (vim_id = 1; vim_id <= VIM_ID_MAX; vim_id++)
    {
        present = onlp_vimi_present_get(vim_id);
        board_id = onlp_vimi_board_id_get(vim_id);

        if (present == VIM_NOT_PRESENT)
        {
            printf("VIM#%d not present\n", vim_id);
        }
        else {
            switch (board_id)
            {
                case VIM_8DE:
                case VIM_16CE:
                    /* VIM Power CPLD */
                    cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_POWER_CPLD_ID); /* Get Power CPLD I2C bus id */
                    if (onlp_file_read_int(&value, VIM_PWR_LED_CTRL_PWR_CPLD_PATH, cpld_bus_id, vim_id) < 0)
                    {
                        AIM_LOG_ERROR("[CPU] Unable to read status from file "VIM_PWR_LED_CTRL_PWR_CPLD_PATH, cpld_bus_id, vim_id);
                        return ONLP_STATUS_E_INTERNAL;
                    }
                    printf("VIM#%d Power LED Control Register: %d \n", vim_id, value);

                    if (onlp_file_read_int(&value, VIM_PORT_LED_CTRL_PWR_CPLD_PATH, cpld_bus_id, vim_id) < 0)
                    {
                        AIM_LOG_ERROR("[CPU] Unable to read status from file "VIM_PORT_LED_CTRL_PWR_CPLD_PATH, cpld_bus_id, vim_id);
                        return ONLP_STATUS_E_INTERNAL;
                    }
                    printf("VIM#%d Port LED on-off control: %d \n", vim_id, value);
                    break;

                case VIM_24CE:
                case VIM_24YE:
                    /* VIM Power CPLD */
                    cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_POWER_CPLD_ID); /* Get Power CPLD I2C bus id */
                    if (onlp_file_read_int(&value, VIM_PWR_LED_CTRL_PWR_CPLD_PATH, cpld_bus_id, vim_id) < 0)
                    {
                        AIM_LOG_ERROR("[CPU] Unable to read status from file "VIM_PWR_LED_CTRL_PWR_CPLD_PATH, cpld_bus_id, vim_id);
                        return ONLP_STATUS_E_INTERNAL;
                    }
                    printf("VIM#%d Power LED Control Register: %d \n", vim_id, value);

                    /* VIM port CPLD */
                    cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_PORT_CPLD_ID); /* Get Port CPLD I2C bus id */
                    if (onlp_file_read_int(&value, VIM_PORT_LED_CTRL_PORT_CPLD_PATH, cpld_bus_id, vim_id) < 0)
                    {
                        AIM_LOG_ERROR("[CPU] Unable to read status from file "VIM_PORT_LED_CTRL_PORT_CPLD_PATH, cpld_bus_id, vim_id);
                        return ONLP_STATUS_E_INTERNAL;
                    }
                    printf("VIM#%d Port LED on-off control: %d \n", vim_id, value);
                    break;

                default:
                    printf("board_id: 0x%x [%s]\n", board_id, "None");
                    break;
            }
        }
    }

    /* Disable BMC Test Mode */
    system("ipmitool raw 0x34 0xf1 0x00");

    return 0;
}

/* MGMT Primary Port, link status and speed */
int onlp_sysi_debug_diag_mgmt_info(void)
{
    int value = 0;
    int link_status = 0;
    int speed = 0;

    /* MGMT Primary Port */
    if (onlp_file_read_int(&value, MODULE_MGMT_PRIMARY_INTF_PATH) < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "MODULE_MGMT_PRIMARY_INTF_PATH);
        return ONLP_STATUS_E_INTERNAL;
    }
    printf("MGMT Primary Port: %s \n", intf_to_str(value));

    /* SFP+	Link status and speed */
    if (onlp_file_read_int(&link_status, MODULE_MGMT_SFP_PATH, "link_status") < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "MODULE_MGMT_SFP_PATH, "link_status");
        return ONLP_STATUS_E_INTERNAL;
    }

    if (onlp_file_read_int(&speed, MODULE_MGMT_SFP_PATH, "speed") < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "MODULE_MGMT_SFP_PATH, "speed");
        return ONLP_STATUS_E_INTERNAL;
    }
    printf("SFP+ Link status: %s, speed: %s \n", link_status_to_str(link_status), speed_to_str(speed));

    /* QSFP28 Link status and speed */
    if (onlp_file_read_int(&link_status, MODULE_MGMT_QSFP_PATH, "link_status") < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "MODULE_MGMT_QSFP_PATH, "link_status");
        return ONLP_STATUS_E_INTERNAL;
    }

    if (onlp_file_read_int(&speed, MODULE_MGMT_QSFP_PATH, "speed") < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "MODULE_MGMT_QSFP_PATH, "speed");
        return ONLP_STATUS_E_INTERNAL;
    }
    printf("QSFP28 Link status: %s, speed: %s \n", link_status_to_str(link_status), speed_to_str(speed));

    /* Copper Link status and speed */
    if (onlp_file_read_int(&link_status, MODULE_MGMT_COPPER_PATH, "link_status") < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "MODULE_MGMT_COPPER_PATH, "link_status");
        return ONLP_STATUS_E_INTERNAL;
    }

    if (onlp_file_read_int(&speed, MODULE_MGMT_COPPER_PATH, "speed") < 0)
    {
        AIM_LOG_ERROR("[CPU] Unable to read status from file "MODULE_MGMT_COPPER_PATH, "speed");
        return ONLP_STATUS_E_INTERNAL;
    }
    printf("Copper Link status: %s, speed: %s \n", link_status_to_str(link_status), speed_to_str(speed));
    return 0;
}


/* For VIM card debug fumction
 *      - onlp_sysi_debug_diag_vim_status
 *      - onlp_sysi_debug_diag_vim_sfp
 */
int onlp_sysi_debug_diag_vim_status(void)
{
    int i = 0;
    int present = 0, board_id = 0;
    for (i = 0; i < VIM_ID_MAX; i++)
    {
        present = onlp_vimi_present_get(i+1);
        board_id = onlp_vimi_board_id_get(i+1);

        /* Get Present */
        if ( present < 0 )
        {
            printf("Get VIM#%d present failed, err=%d\n", i+1, present);
        }
        else
        {
            printf("VIM#%d \n", i+1);
            printf("Present: 0x%x [%s]\n", present,
                (present) ? "NOT PRESENT" : "PRESENT");
        }

        /* Get Board_id */
        if ( board_id < 0 )
        {
            if (present == 1)
                printf("VIM#%d not present\n", i+1);
            else
                printf("Get VIM#%d Board ID failed, err=%d\n", i+1, board_id);
        }
        else
        {
            switch (board_id)
            {
                case VIM_8DE:
                    printf("Board ID: 0x%x [%s]\n", board_id, "8DE");
                    break;

                case VIM_16CE:
                    printf("Board ID: 0x%x [%s]\n", board_id, "16CE");
                    break;

                case VIM_24CE:
                    printf("Board ID: 0x%x [%s]\n", board_id, "24CE");
                    break;

                case VIM_24YE:
                    printf("Board ID: 0x%x [%s]\n", board_id, "24YE");
                    break;

                default:
                    printf("Board ID: 0x%x [%s]\n", board_id, "None");
                    break;
            }
        }
        printf("\n");
    }
    return 0;
}

int onlp_sysi_debug_diag_vim_sfp(int index, int offset, int page, int page_select)
{
    uint8_t *data = NULL;
    int rv = 0;

    uint8_t org_b = 0;
    uint16_t org_w = 0;
    uint8_t temp_b = 0;
    uint16_t temp_w = 0;
	uint8_t page_org_b = 0;

    int addr = 0;
    int board_id;
    int is_qsfp_port = 0;
    int vim_id;

    data = aim_zmalloc(256);
    if ((rv = onlp_sfpi_eeprom_read(index, data)) < 0)
    {
        aim_printf(&aim_pvs_stdout, "Error reading eeprom: %{onlp_status}\n");
    }
    else
    {
        aim_printf(&aim_pvs_stdout, "dump eeprom:\n%{data}\n", data, 256);
    }
    aim_free(data);
    data = NULL;

    /* Confirm which board_id the port index belongs to */
    vim_id = onlp_vimi_index_map_to_vim_id(index);
    if (vim_id == ONLP_STATUS_E_INTERNAL)
    {
        /* Port index out of range */
        return ONLP_STATUS_E_INVALID;
    }
    board_id = onlp_vimi_board_id_get(vim_id);
    if (board_id == ONLP_STATUS_E_INTERNAL)
    {
        /* Get board id failed */
        return ONLP_STATUS_E_INVALID;
    }
    addr = SFP_PLUS_EEPROM_I2C_ADDR;

    switch (board_id)
    {
        case VIM_8DE:
            /* QSFP-DD 400G */
            page_select = QSFPDD_DIAG_PAGE_SELECT;
            offset = QSFPDD_DIAG_OFFSET;
            page = QSFPDD_DIAG_PAGE;
            is_qsfp_port = 1;
            break;
        case VIM_16CE:
            /* QSFP28 100G */
            page_select = QSFP28_DIAG_PAGE_SELECT;
            offset = QSFP28_DIAG_OFFSET;
            page = QSFP28_DIAG_PAGE;
            is_qsfp_port = 1;
            break;
        case VIM_24CE:
            /* SFP-DD */
            offset = SFP_DIAG_OFFSET;
            break;
        case VIM_24YE:
            /* SFP28 (SFF-8472) */
            addr = SFP_DOM_EEPROM_I2C_ADDR;
            offset = SFP28_DIAG_OFFSET;
            break;
        default:
            break;
    }

    /* BYTE */
    printf("Read/Write byte test...\n");

	if (is_qsfp_port)
    {
        page_org_b = onlp_vim_sfpi_dev_readb(index, addr, page_select);
    	if (page_org_b < 0)
    	{
        	printf("Error, read failed!\n");
        	goto DONE;
    	}

		rv = onlp_vim_sfpi_dev_writeb(index, addr, page_select, page);
    	if (rv < 0)
    	{
        	printf("Error, write failed!\n");
        	goto RESTORE_PAGE;
    	}
    	sleep(2);
    	temp_b = onlp_vim_sfpi_dev_readb(index, addr, page_select);
    	if (temp_b < 0)
    	{
        	printf("Error, read failed!\n");
        	goto RESTORE_PAGE;
    	}
    	if (temp_b != page)
    	{
        	printf("Error, can not change page!\n");
        	goto RESTORE_PAGE;
    	}
    }

    org_b = onlp_vim_sfpi_dev_readb(index, addr, offset);
    if (org_b < 0)
    {
        printf("Error, read failed!\n");
        if (is_qsfp_port)
        {
            goto RESTORE_PAGE;
        }
		else
        {
            goto DONE;
        }
    }

    rv = onlp_vim_sfpi_dev_writeb(index, addr, offset, SFP_DIAG_PATTEN_B);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto RESTORE_EEPROM_B;
    }
    sleep(2);
    temp_b = onlp_vim_sfpi_dev_readb(index, addr, offset);
    if (temp_b < 0)
    {
        printf("Error, read failed!\n");
        goto RESTORE_EEPROM_B;
    }
    if (temp_b != SFP_DIAG_PATTEN_B)
    {
        printf("Error, mismatch!\n");
        goto RESTORE_EEPROM_B;
    }
    rv = onlp_vim_sfpi_dev_writeb(index, addr, offset, org_b);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto RESTORE_EEPROM_B;
    }
    sleep(2);

    /* WORD */
    printf("Read/Write word test...\n");
    org_w = onlp_vim_sfpi_dev_readw(index, addr, offset);
    if (org_w < 0)
    {
        printf("Error, read failed!\n");
        if (is_qsfp_port)
        {
            goto RESTORE_PAGE;
        }
		else
        {
            goto DONE;
        }
    }
    rv = onlp_vim_sfpi_dev_writew(index, addr, offset, SFP_DIAG_PATTEN_W);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto RESTORE_EEPROM_W;
    }
    sleep(2);
    temp_w = onlp_vim_sfpi_dev_readw(index, addr, offset);
    if (temp_w < 0)
    {
        printf("Error, read failed!\n");
        goto RESTORE_EEPROM_W;
    }
    if (temp_w != SFP_DIAG_PATTEN_W)
    {
        printf("Error, mismatch!\n");
        goto RESTORE_EEPROM_W;
    }
    rv = onlp_vim_sfpi_dev_writew(index, addr, offset, org_w);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto RESTORE_EEPROM_W;
    }

	if (is_qsfp_port)
    {
		goto RESTORE_PAGE;
    }

DONE:
    return 0;
RESTORE_PAGE:
    rv = onlp_vim_sfpi_dev_writeb(index, addr, page_select, page_org_b);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
    }
    goto DONE;
RESTORE_EEPROM_B:
    rv = onlp_vim_sfpi_dev_writeb(index, addr, offset, org_b);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
    }
    if (is_qsfp_port)
    {
        goto RESTORE_PAGE;
    }
	else
    {
        goto DONE;
    }
RESTORE_EEPROM_W:
    rv = onlp_vim_sfpi_dev_writew(index, addr, offset, org_w);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
    }
    if (is_qsfp_port)
    {
        goto RESTORE_PAGE;
    }
	else
    {
        goto DONE;
    }
}

int
onlp_sysi_show_cpld_version()
{

    DIAG_PRINT("%s", __FUNCTION__);

	int i, ret, vim_id = 0, index = 0;
    int sys_cpld_version = 0;
    int vim1_present = VIM_NOT_PRESENT;
    int vim2_present = VIM_NOT_PRESENT;
    int vim1_board_id = VIM_NONE;
    int vim2_board_id = VIM_NONE;
    int vim1_power_cpld_i2c_bus_id = 0;
    int vim1_port_cpld_i2c_bus_id = 0;
    int vim2_power_cpld_i2c_bus_id = 0;
    int vim2_port_cpld_i2c_bus_id = 0;

	vim_cpld_version_t cplds[] = { {},
                                   {},
                                   {},
                                   {} };

    for (vim_id = 1; vim_id <= VIM_ID_MAX; vim_id++)
    {
        switch (vim_id)
        {
            case VIM1_ID:
                vim1_present = onlp_vimi_present_get(VIM1_ID);
                if (vim1_present == VIM_PRESENT)
                {
                    vim1_board_id = onlp_vimi_board_id_get(VIM1_ID);
                    vim1_power_cpld_i2c_bus_id = onlp_vimi_cpld_bus_id_get(VIM1_ID, VIM_POWER_CPLD_ID);
                    cplds[index] = (vim_cpld_version_t){ VIM_POWER_CPLD_REVISION_PATH, vim1_power_cpld_i2c_bus_id, VIM1_ID, 0, "VIM1-Power-CPLD"};
                    index++;
                    if (vim1_board_id == VIM_24CE || vim1_board_id == VIM_24YE)
                    {
                        vim1_port_cpld_i2c_bus_id = onlp_vimi_cpld_bus_id_get(VIM1_ID, VIM_PORT_CPLD_ID);
                        cplds[index] = (vim_cpld_version_t){ VIM_PORT_CPLD_REVISION_PATH, vim1_port_cpld_i2c_bus_id, VIM1_ID, 0, "VIM1-Port-CPLD"};
                        index++;
                    }
                }
                break;
            case VIM2_ID:
                vim2_present = onlp_vimi_present_get(VIM2_ID);
                if (vim2_present == VIM_PRESENT)
                {
                    vim2_board_id = onlp_vimi_board_id_get(VIM2_ID);
                    vim2_power_cpld_i2c_bus_id = onlp_vimi_cpld_bus_id_get(VIM2_ID, VIM_POWER_CPLD_ID);
                    cplds[index] = (vim_cpld_version_t){ VIM_POWER_CPLD_REVISION_PATH, vim2_power_cpld_i2c_bus_id, VIM2_ID, 0, "VIM2-Power-CPLD"};
                    index++;
                    if (vim2_board_id == VIM_24CE || vim2_board_id == VIM_24YE)
                    {
                        vim2_port_cpld_i2c_bus_id = onlp_vimi_cpld_bus_id_get(VIM2_ID, VIM_PORT_CPLD_ID);
                        cplds[index] = (vim_cpld_version_t){ VIM_PORT_CPLD_REVISION_PATH, vim2_port_cpld_i2c_bus_id, VIM2_ID, 0, "VIM2-Port-CPLD"};
                        index++;
                    }
                }
                break;

            default:
                break;
        }
    }

    /* Read SYS CPLD version */
    ret = onlp_file_read_int(&sys_cpld_version, SYS_CPLD_REVISION_PATH);

	if (ret < 0) {
		AIM_LOG_ERROR("Unable to read version from CPLD(%s)\r\n", "System-CPLD");
		return ONLP_STATUS_E_INTERNAL;
    }

	/* Read VIM CPLD version */
	for (i = 0; i < index; i++) {
		ret = onlp_file_read_int(&cplds[i].version,
					 			cplds[i].cpld_path_format,
					 			cplds[i].i2c_bus_id,
                                cplds[i].vim_id);

		if (ret < 0) {
			AIM_LOG_ERROR("Unable to read version from CPLD(%s)\r\n", cplds[i].description);
			return ONLP_STATUS_E_INTERNAL;
		}
	}

    /* Print cpld version */
    printf("%s:%02x\n", "System CPLD", sys_cpld_version);
    for (i = 0; i < index; i++) {
        printf("%s:%02x\n", cplds[i].description, cplds[i].version);
    }

    return ONLP_STATUS_OK;
}

int
onlp_sysi_debug(aim_pvs_t *pvs, int argc, char *argv[])
{
    int ret = 0;

    /* ONLPI driver APIs debug */

    if (argc > 0 && !strcmp(argv[0], "sys"))
    {
        diag_flag_set(DIAG_FLAG_ON);
        printf("DIAG for SYS: \n");
        printf("Platform : %s\n", onlp_sysi_platform_get());
        onlp_sysi_init();
        onlp_sysi_platform_manage_init();
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "cpld_version"))
    {
        printf("DIAG for cpld_version: \n");
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sysi_show_cpld_version();
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "fan"))
    {
        printf("DIAG for FAN: \n");
        diag_flag_set(DIAG_FLAG_ON);
        onlp_fani_init();
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "fan_status"))
    {
        diag_flag_set(DIAG_FLAG_ON);
        printf("DIAG for FAN status: \n");
        onlp_sysi_debug_diag_fan_status();
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "psu"))
    {
        printf("DIAG for PSU: \n");
        diag_flag_set(DIAG_FLAG_ON);
        onlp_psui_init();
        diag_flag_set(DIAG_FLAG_OFF);

    }
    else if (argc > 0 && !strcmp(argv[0], "led"))
    {
        printf("DIAG for LED: \n");
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sysi_debug_diag_led();
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "sfp_status"))
    {
        printf("DIAG for SFP status: \n");
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sysi_debug_diag_sfp_status();
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "sfp_dom"))
    {
        int port_index = 0;
        int vim_end_index = onlp_vimi_get_vim_end_index();

        if (argc != 2)
        {
            printf("Parameter error, format: onlpdump debugi sfp_dom [PORT]\n");
            return -1;
        }
        port_index = atoi(argv[1]);
        if (port_index <= SFP_START_INDEX || port_index > vim_end_index)
        {
            printf("Parameter error, PORT out of range.\n");
            return -1;
        }
        if (IS_VIM_PORT((port_index - 1), vim_end_index))
        {
            printf("DIAG for VIM SFP DOM #%d: \n", port_index - 1);
        }
        else
        {
            printf("DIAG for SFP DOM #%d: \n", port_index - 1);
        }
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sysi_debug_diag_sfp_dom(port_index - 1);
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "sfp_ctrl_set"))
    {
        int port_index = 0, ctrl = 0, val = 0;
        int vim_end_index = onlp_vimi_get_vim_end_index();

        if (argc != 4)
        {
            printf("Parameter error, format: onlpdump debugi sfp_ctrl_set [PORT] [CTRL] [VALUE]\n");
            return -1;
        }
        port_index = atoi(argv[1]);
        if (port_index <= SFP_START_INDEX || port_index > vim_end_index)
        {
            printf("Parameter error, PORT out of range.\n");
            return -1;
        }
        ctrl = atoi(argv[2]);
        val = atoi(argv[3]);
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sfpi_control_set(port_index - 1, ctrl, val);
        diag_flag_set(DIAG_FLAG_OFF);

    }
    else if (argc > 0 && !strcmp(argv[0], "sfp_ctrl_get"))
    {
        int port_index = 0, ctrl = 0, val = 0;
        int vim_end_index = onlp_vimi_get_vim_end_index();

        if (argc != 3)
        {
            printf("Parameter error, format: onlpdump debugi sfp_ctrl_get [PORT] [CTRL] \n");
            return -1;
        }
        port_index = atoi(argv[1]);
        if (port_index <= SFP_START_INDEX || port_index > vim_end_index)
        {
            printf("Parameter error, PORT out of range.\n");
            return -1;
        }
        ctrl = atoi(argv[2]);
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sfpi_control_get(port_index - 1, ctrl, &val);
        printf("Value = %d(0x%X)\n", val, val);
        diag_flag_set(DIAG_FLAG_OFF);

    }
    else if (argc > 0 && !strcmp(argv[0], "sfp_ctrl"))
    {
        int port_index = 0;
        int vim_end_index = onlp_vimi_get_vim_end_index();

        if (argc != 2)
        {
            printf("Parameter error, format: onlpdump debugi sfp_ctrl [PORT]\n");
            return -1;
        }
        port_index = atoi(argv[1]);
        if (port_index <= SFP_START_INDEX || port_index > vim_end_index)
        {
            printf("Parameter error, PORT out of range.\n");
            return -1;
        }
        if (IS_VIM_PORT((port_index - 1), vim_end_index))
        {
            printf("DIAG for VIM SFP Control #%d: \n", port_index - 1);
        }
        else
        {
            printf("DIAG for SFP Control #%d: \n", port_index - 1);
        }
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sysi_debug_diag_sfp_ctrl(port_index - 1);
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "sfp"))
    {
        int vim_end_index = onlp_vimi_get_vim_end_index();

        if (argc > 1)
        {
            int sfp_offset = -1, sfp_page = -1, sfp_page_sel = -1;
            int port_index = atoi(argv[1]);
            if (port_index <= SFP_START_INDEX || port_index > (SFP_START_INDEX + NUM_OF_SFP_PORT))
            {
                if (!(IS_VIM_PORT((port_index - 1), vim_end_index)))
                {
                    printf("Parameter error, PORT out of range.\n");
                    return -1;
                }
            }
            if (argc > 2)
            {
                if (argc == 5)
                {
                    sfp_offset = atoi(argv[2]);
                    sfp_page = atoi(argv[3]);
                    sfp_page_sel = atoi(argv[4]);
                }
                else
                {
                    printf("Parameter error, The command format is \"onlpdump debugi sfp [PORT] [REG_ADDR] [PAGE] [PAGE_SEL]\".\n");
                    return -1;
                }
            }

            if (IS_VIM_PORT((port_index - 1), vim_end_index))
            {
                printf("DIAG for VIM SFP#%d: \n", port_index - 1);
                diag_flag_set(DIAG_FLAG_ON);
                onlp_sysi_debug_diag_vim_sfp(port_index - 1, sfp_offset, sfp_page, sfp_page_sel);
                diag_flag_set(DIAG_FLAG_OFF);
            }
            else
            {
                printf("DIAG for SFP#%d: \n", port_index - 1);
                diag_flag_set(DIAG_FLAG_ON);
                onlp_sysi_debug_diag_sfp(port_index - 1, sfp_offset, sfp_page, sfp_page_sel);
                diag_flag_set(DIAG_FLAG_OFF);
            }
        }
        else
        {
            printf("DIAG for SFP: \n");
            onlp_sfp_bitmap_t bmap;
            diag_flag_set(DIAG_FLAG_ON);

            onlp_sfpi_denit();
            onlp_sfpi_init();

            onlp_sfp_bitmap_t_init(&bmap);
            ret = onlp_sfpi_bitmap_get(&bmap);
            if (ret < 0)
            {
                printf("Error, onlp_sfpi_bitmap_get failed!\n");
            }
            else
            {
                aim_printf(&aim_pvs_stdout, "sfp_bitmap:\n  %{aim_bitmap}\n", &bmap);
            }
            diag_flag_set(DIAG_FLAG_OFF);

            return 0;
        }
    }
    else if (argc > 0 && !strcmp(argv[0], "sfpwb")) /* write byte */
    {
        int port;
        uint8_t addr, value;
        int vim_end_index = onlp_vimi_get_vim_end_index();

        if (argc == 4)
        {
            port = atoi(argv[1]);
            addr = (uint8_t)atoi(argv[2]);
            value = (uint8_t)atoi(argv[3]);

            if (port <= SFP_START_INDEX || port > (SFP_START_INDEX + NUM_OF_SFP_PORT))
            {
                if (!(IS_VIM_PORT((port - 1), vim_end_index)))
                {
                    printf("Parameter error, PORT out of range.\n");
                    return -1;
                }
            }

            diag_flag_set(DIAG_FLAG_ON);
            if (IS_VIM_PORT((port - 1), vim_end_index))
            {
                onlp_vim_sfpi_dev_writeb(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr, value);
            }
            else
            {
                onlp_sfpi_dev_writeb(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr, value);
            }
            diag_flag_set(DIAG_FLAG_OFF);
        }
        else
        {
            printf("Parameter error, format: onlpdump debugi sfpwb [PORT] [ADDR] [VALUE]\n");
            return -1;
        }

    }
    else if (argc > 0 && !strcmp(argv[0], "sfprb")) /* read byte */
    {
        int port;
        uint8_t addr;
        int vim_end_index = onlp_vimi_get_vim_end_index();

        if (argc == 3)
        {
            port = atoi(argv[1]);
            addr = (uint8_t)atoi(argv[2]);

            if (port <= SFP_START_INDEX || port > (SFP_START_INDEX + NUM_OF_SFP_PORT))
            {
                if (!(IS_VIM_PORT((port - 1), vim_end_index)))
                {
                    printf("Parameter error, PORT out of range.\n");
                    return -1;
                }
            }

            diag_flag_set(DIAG_FLAG_ON);
            if (IS_VIM_PORT((port - 1), vim_end_index))
            {
                onlp_vim_sfpi_dev_readb(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr);
            }
            else
            {
                onlp_sfpi_dev_readb(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr);
            }
            diag_flag_set(DIAG_FLAG_OFF);
        }
        else
        {
            printf("Parameter error, format: onlpdump debugi sfprb [PORT] [ADDR]\n");
            return -1;
        }
    }
    else if (argc > 0 && !strcmp(argv[0], "sfpww")) /* write word */
    {
        int port;
        uint16_t value;
        uint8_t addr;
        int vim_end_index = onlp_vimi_get_vim_end_index();

        if (argc == 4)
        {
            port = atoi(argv[1]);
            addr = (uint8_t)atoi(argv[2]);
            value = (uint16_t)atoi(argv[3]);

            if (port <= SFP_START_INDEX || port > (SFP_START_INDEX + NUM_OF_SFP_PORT))
            {
                if (!(IS_VIM_PORT((port - 1), vim_end_index)))
                {
                    printf("Parameter error, PORT out of range.\n");
                    return -1;
                }
            }

            diag_flag_set(DIAG_FLAG_ON);
            if (IS_VIM_PORT((port - 1), vim_end_index))
            {
                onlp_vim_sfpi_dev_writew(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr, value);
            }
            else
            {
                onlp_sfpi_dev_writew(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr, value);
            }
            diag_flag_set(DIAG_FLAG_OFF);
        }
        else
        {
            printf("Parameter error, format: onlpdump debugi sfpwb [PORT] [ADDR] [VALUE]\n");
            return -1;
        }
    }
    else if (argc > 0 && !strcmp(argv[0], "sfprw")) /* read word */
    {
        int port;
        uint8_t addr;
        int vim_end_index = onlp_vimi_get_vim_end_index();

        if (argc == 3)
        {
            port = atoi(argv[1]);
            addr = (uint8_t)atoi(argv[2]);

            if (port <= SFP_START_INDEX || port > (SFP_START_INDEX + NUM_OF_SFP_PORT))
            {
                if (!(IS_VIM_PORT((port - 1), vim_end_index)))
                {
                    printf("Parameter error, PORT out of range.\n");
                    return -1;
                }
            }

            diag_flag_set(DIAG_FLAG_ON);
            if (IS_VIM_PORT((port - 1), vim_end_index))
            {
                onlp_vim_sfpi_dev_readw(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr);
            }
            else
            {
                onlp_sfpi_dev_readw(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr);
            }
            diag_flag_set(DIAG_FLAG_OFF);
        }
        else
        {
            printf("Parameter error, format: onlpdump debugi sfprb [PORT] [ADDR]\n");
            return -1;
        }
    }
    else if (argc > 0 && !strcmp(argv[0], "thermal"))
    {
        printf("DIAG for Thermal: \n");
        diag_flag_set(DIAG_FLAG_ON);
        onlp_thermali_init();
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "trace_on"))
    {
        diag_debug_trace_on();
        DIAG_PRINT("%s, ONLPI TRACE: ON", __FUNCTION__);
    }
    else if (argc > 0 && !strcmp(argv[0], "trace_off"))
    {
        diag_debug_trace_off();
        DIAG_PRINT("%s, ONLPI TRACE: OFF", __FUNCTION__);
    }
    else if (argc > 0 && !strcmp(argv[0], "vim_status"))
    {
        printf("DIAG for VIM: \n");
        diag_flag_set(DIAG_FLAG_ON);
        onlp_vimi_init();
        onlp_sysi_debug_diag_vim_status();
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "led_control_bit"))
    {
        printf("DIAG for LED control bit: \n");
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sysi_debug_diag_led_control_bit();
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "mgmt"))
    {
        printf("DIAG for MGMT information: \n");
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sysi_debug_diag_mgmt_info();
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "get_i2c_tree_db"))
    {
        int fd;
        char data[STORAGE_SIZE];
        void *addr;
        int count;

        /* Get shared memory file descriptor (not a file) */
        fd = shm_open(STORAGE_ID, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
        if (fd == -1)
        {
            AIM_LOG_ERROR("[get_i2c_tree_db] shm_open failed\r\n");
            return 10;
        }

        /* Map shared memory to process address space */
        addr = mmap(NULL, STORAGE_SIZE, PROT_READ, MAP_SHARED, fd, 0);
        if (addr == MAP_FAILED)
        {
            AIM_LOG_ERROR("[get_i2c_tree_db] mmap failed\r\n");
            return 30;
        }

        /* Copy memory from addr to data */
        memcpy(data, addr, STORAGE_SIZE);

        AIM_SYSLOG_INFO("[DEBUG]",
                        "[DEBUG]",
                        "[get_i2c_tree_db] Read from shared memory: addr--(%s)", addr);

        AIM_SYSLOG_INFO("[DEBUG]",
                        "[DEBUG]",
                        "[get_i2c_tree_db] Read from shared memory: data--(%s)", data);

        /* Data processing */
        count = 0;
        char *split_date = strtok(data, " ");
        if(split_date)
        {
            printf("[%d] atoi(%s) is %d \n", count, split_date, atoi(split_date));
            count ++;
        }

        while((split_date=strtok(NULL, " ")))
        {
            /* Use the first parameter as NULL to extract a substring */
            printf("[%d] atoi(%s) is %d\n", count, split_date, atoi(split_date));
            count ++;
        }
    }
    else if (argc > 0 && !strcmp(argv[0], "get_vim_cpld_bus_id"))
    {
        int vim_id = atoi(argv[1]);
        int cpld_bus_id = 0;
        int vim_board_id = 0;
        int vim_present = -1;
        vim_present = onlp_vimi_present_get(vim_id);

        if (vim_present == VIM_PRESENT)
        {
            cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_POWER_CPLD_ID);
            if (cpld_bus_id < 58)
                printf("onlp_vimi_cpld_bus_id_get failed\n");
            else
                printf("VIM %d Power CPLD I2C bus id is %d\n", vim_id, cpld_bus_id);

            vim_board_id = onlp_vimi_board_id_get(vim_id);
            if ((vim_board_id == VIM_24CE) || (vim_board_id == VIM_24YE))
            {
                cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_PORT_CPLD_ID);
                if (cpld_bus_id < 58)
                    printf("onlp_vimi_cpld_bus_id_get failed\n");
                else
                    printf("VIM %d Port CPLD I2C bus id is %d\n", vim_id, cpld_bus_id);
            }
        }
        else
        {
            printf("VIM %d is not present\n", vim_id);
        }

    }
    else if (argc > 0 && !strcmp(argv[0], "get_vim_port_information"))
    {
        int vim1_start_port, vim2_start_port;
        int end_port = onlp_vimi_get_vim_end_index();
        int vim_id, list_index;
        int *optoe_bus_id;

        vim1_start_port = onlp_vimi_optoe_start_port_get(VIM1_ID);
        vim2_start_port = onlp_vimi_optoe_start_port_get(VIM2_ID);

        if ((vim1_start_port == -1) && (vim2_start_port == -1))
        {
            printf("VIM 1 not present\n");
            printf("VIM 2 not present\n");
        }
        else if (vim2_start_port == -1)
        {
            printf("VIM 1 (port %d ~ port %d)\n", vim1_start_port, end_port-1);
            printf("VIM 2 not present\n");

            printf("\n");

            printf("SYS_PORT_INDEX    VIM_ID    VIM_PORT_INDEX    I2C_BUS_ID\n");
            printf("--------------    ------    --------------    ----------\n");
            for (int port = vim1_start_port; port < end_port; port++)
            {
                vim_id = onlp_vimi_index_map_to_vim_id(port);
                optoe_bus_id = onlp_vimi_get_optoe_bus_id_list(vim_id);
                list_index = onlp_vimi_get_list_index(vim_id, port);
                printf("%14d    VIM %2d    %14d    %10d\n", port, vim_id, list_index, optoe_bus_id[list_index-1]);
            }
        }
        else if (vim1_start_port == -1)
        {
            printf("VIM 1 not present\n");
            printf("VIM 2 (port %d ~ port %d)\n", vim2_start_port, end_port-1);

            printf("\n");

            printf("SYS_PORT_INDEX    VIM_ID    VIM_PORT_INDEX    I2C_BUS_ID\n");
            printf("--------------    ------    --------------    ----------\n");
            for (int port = vim2_start_port; port < end_port; port++)
            {
                vim_id = onlp_vimi_index_map_to_vim_id(port);
                optoe_bus_id = onlp_vimi_get_optoe_bus_id_list(vim_id);
                list_index = onlp_vimi_get_list_index(vim_id, port);
                printf("%14d    VIM %2d    %14d    %10d\n", port, vim_id, list_index, optoe_bus_id[list_index-1]);
            }
        }
        else
        {
            printf("VIM 1 (port %d ~ port %d)\n", vim1_start_port, vim2_start_port-1);
            printf("VIM 2 (port %d ~ port %d)\n", vim2_start_port, end_port-1);

            printf("\n");

            printf("SYS_PORT_INDEX    VIM_ID    VIM_PORT_INDEX    I2C_BUS_ID\n");
            printf("--------------    ------    --------------    ----------\n");
            for (int port = vim1_start_port; port < end_port; port++)
            {
                vim_id = onlp_vimi_index_map_to_vim_id(port);
                optoe_bus_id = onlp_vimi_get_optoe_bus_id_list(vim_id);
                list_index = onlp_vimi_get_list_index(vim_id, port);
                printf("%14d    VIM %2d    %14d    %10d\n", port, vim_id, list_index, optoe_bus_id[list_index-1]);
            }
        }

        if (!((vim1_start_port == -1) && (vim2_start_port == -1)))
        {
            printf("\n");
            printf("Note1: SYS_PORT_INDEX start from 0, and 0~41 is front port index.\n");
            printf("Note2: VIM_PORT_INDEX means the port index for each VIM.\n");
            printf("Note3: I2C_BUS_ID means the i2c bus id for each port on VIM.\n");
        }
    }
    else if (argc > 0 && !strcmp(argv[0], "get_vim_sfp_ctl_support"))
    {
        int vim1_start_port, vim2_start_port;
        int end_port = onlp_vimi_get_vim_end_index();
        int vim_id, list_index;
        int reset_sup = 0;
        int lp_mode_sup = 0;
        int tx_fault_sup = 0;
        int tx_dis_sup = 0;
        int rx_loss_sup = 0;
        int tx_fault_b_sup = 0;
        int tx_dis_b_sup = 0;
        int rx_loss_b_sup = 0;

        vim1_start_port = onlp_vimi_optoe_start_port_get(VIM1_ID);
        vim2_start_port = onlp_vimi_optoe_start_port_get(VIM2_ID);

        if ((vim1_start_port == -1) && (vim2_start_port == -1))
        {
            printf("VIM 1 not present\n");
            printf("VIM 2 not present\n");
        }
        else if (vim2_start_port == -1)
        {
            printf("VIM 1 (port %d ~ port %d)\n", vim1_start_port, end_port-1);
            printf("VIM 2 not present\n");

            printf("\n");

            printf("SYS_PORT_INDEX    VIM_ID    VIM_PORT_INDEX    RESET    LP_MODE    TX_FAULT    TX_DIS    RX_LOSS    TX_FAULT_B    TX_DIS_B    RX_LOSS_B\n");
            printf("--------------    ------    --------------    -----    -------    --------    ------    -------    ----------    --------    ---------\n");
            for (int port = vim1_start_port; port < end_port; port++)
            {
                vim_id = onlp_vimi_index_map_to_vim_id(port);
                list_index = onlp_vimi_get_list_index(vim_id, port);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_RESET, &reset_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_LP_MODE, &lp_mode_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_TX_FAULT, &tx_fault_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_TX_DISABLE, &tx_dis_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_RX_LOS, &rx_loss_sup);
                onlp_sfpi_control_supported_for_b_attr(port, ONLP_SFP_CONTROL_TX_FAULT_B, &tx_fault_b_sup);
                onlp_sfpi_control_supported_for_b_attr(port, ONLP_SFP_CONTROL_TX_DISABLE_B, &tx_dis_b_sup);
                onlp_sfpi_control_supported_for_b_attr(port, ONLP_SFP_CONTROL_RX_LOS_B, &rx_loss_b_sup);

                printf("%14d    VIM %2d    %14d    %5d    %7d    %8d    %6d    %7d    %10d    %8d    %9d\n", \
                port, vim_id, list_index, reset_sup, lp_mode_sup, tx_fault_sup, tx_dis_sup, rx_loss_sup, tx_fault_b_sup, tx_dis_b_sup, rx_loss_b_sup);
            }
        }
        else if (vim1_start_port == -1)
        {
            printf("VIM 1 not present\n");
            printf("VIM 2 (port %d ~ port %d)\n", vim2_start_port, end_port-1);

            printf("\n");

            printf("SYS_PORT_INDEX    VIM_ID    VIM_PORT_INDEX    RESET    LP_MODE    TX_FAULT    TX_DIS    RX_LOSS    TX_FAULT_B    TX_DIS_B    RX_LOSS_B\n");
            printf("--------------    ------    --------------    -----    -------    --------    ------    -------    ----------    --------    ---------\n");
            for (int port = vim2_start_port; port < end_port; port++)
            {
                vim_id = onlp_vimi_index_map_to_vim_id(port);
                list_index = onlp_vimi_get_list_index(vim_id, port);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_RESET, &reset_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_LP_MODE, &lp_mode_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_TX_FAULT, &tx_fault_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_TX_DISABLE, &tx_dis_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_RX_LOS, &rx_loss_sup);
                onlp_sfpi_control_supported_for_b_attr(port, ONLP_SFP_CONTROL_TX_FAULT_B, &tx_fault_b_sup);
                onlp_sfpi_control_supported_for_b_attr(port, ONLP_SFP_CONTROL_TX_DISABLE_B, &tx_dis_b_sup);
                onlp_sfpi_control_supported_for_b_attr(port, ONLP_SFP_CONTROL_RX_LOS_B, &rx_loss_b_sup);

                printf("%14d    VIM %2d    %14d    %5d    %7d    %8d    %6d    %7d    %10d    %8d    %9d\n", \
                port, vim_id, list_index, reset_sup, lp_mode_sup, tx_fault_sup, tx_dis_sup, rx_loss_sup, tx_fault_b_sup, tx_dis_b_sup, rx_loss_b_sup);
            }
        }
        else
        {
            printf("VIM 1 (port %d ~ port %d)\n", vim1_start_port, vim2_start_port-1);
            printf("VIM 2 (port %d ~ port %d)\n", vim2_start_port, end_port-1);

            printf("\n");

            printf("SYS_PORT_INDEX    VIM_ID    VIM_PORT_INDEX    RESET    LP_MODE    TX_FAULT    TX_DIS    RX_LOSS    TX_FAULT_B    TX_DIS_B    RX_LOSS_B\n");
            printf("--------------    ------    --------------    -----    -------    --------    ------    -------    ----------    --------    ---------\n");
            for (int port = vim1_start_port; port < end_port; port++)
            {
                vim_id = onlp_vimi_index_map_to_vim_id(port);
                list_index = onlp_vimi_get_list_index(vim_id, port);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_RESET, &reset_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_LP_MODE, &lp_mode_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_TX_FAULT, &tx_fault_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_TX_DISABLE, &tx_dis_sup);
                onlp_sfpi_control_supported(port, ONLP_SFP_CONTROL_RX_LOS, &rx_loss_sup);
                onlp_sfpi_control_supported_for_b_attr(port, ONLP_SFP_CONTROL_TX_FAULT_B, &tx_fault_b_sup);
                onlp_sfpi_control_supported_for_b_attr(port, ONLP_SFP_CONTROL_TX_DISABLE_B, &tx_dis_b_sup);
                onlp_sfpi_control_supported_for_b_attr(port, ONLP_SFP_CONTROL_RX_LOS_B, &rx_loss_b_sup);

                printf("%14d    VIM %2d    %14d    %5d    %7d    %8d    %6d    %7d    %10d    %8d    %9d\n", \
                port, vim_id, list_index, reset_sup, lp_mode_sup, tx_fault_sup, tx_dis_sup, rx_loss_sup, tx_fault_b_sup, tx_dis_b_sup, rx_loss_b_sup);
            }

        }

        if (!((vim1_start_port == -1) && (vim2_start_port == -1)))
        {
            printf("\n");
            printf("Note1: 1 means it support control\n");
            printf("       0 means it not support control\n");
            printf("Note2: SYS_PORT_INDEX start from 0, and 0~41 is front port index.\n");
            printf("Note3: VIM_PORT_INDEX means the port index for each VIM.\n");
        }

    }
    else if (argc > 0 && !strcmp(argv[0], "get_vim_eeprom"))
    {
        int vim_id = atoi(argv[1]);
        int rv;

        rv = onlp_sysi_debug_vim_eeprom(vim_id);

        printf("rv = %d\n", rv);
    }
    else if (argc > 0 && !strcmp(argv[0], "vim_power_on"))
    {
        int vim_id = atoi(argv[1]);
        int rv;

        rv = onlp_vimi_power_control(ON, vim_id);

        printf("rv = %d\n", rv);
    }
    else if (argc > 0 && !strcmp(argv[0], "vim_power_off"))
    {
        int vim_id = atoi(argv[1]);
        int rv;

        rv = onlp_vimi_power_control(OFF, vim_id);

        printf("rv = %d\n", rv);
    }
    else if (argc > 0 && !strcmp(argv[0], "help"))
    {
        printf("\nUsage: onlpdump debugi [OPTION]\n");
        printf("    help                : this message.\n");
        printf("    cpld_version        : show cpld version.\n");
        printf("    vim_status          : show vim status (present, board id).\n");
        printf("    trace_on            : turn on ONLPI debug trace message output on screen.\n");
        printf("    trace_off           : turn off ONLPI debug trace message output on screen.\n");
        printf("    sys                 : run system ONLPI diagnostic function.\n");
        printf("    fan                 : run fan ONLPI diagnostic function.\n");
        printf("    fan_status          : run fan status ONLPI diagnostic function.\n");
        printf("    led                 : run LED ONLPI diagnostic function.\n");
        printf("    psu                 : run psu ONLPI diagnostic function.\n");
        printf("    thermal             : run thermal ONLPI diagnostic function.\n");
        printf("    sfp                 : run sfp ONLPI diagnostic function.\n");
        printf("    sfp [PORT] [REG_ADDR] [PAGE] [PAGE_SEL] : run sfp ONLPI diagnostic function.\n");
        printf("    sfp_dom [PORT]      : run sfp dom ONLPI diagnostic function.\n");
        printf("    sfp_ctrl [PORT]     : run sfp control ONLPI diagnostic function.\n");
        printf("    led_control_bit                         : show LED control bit status.\n");
        printf("    mgmt                                    : show mgmt status (Link status and speed).\n");
        printf("    get_i2c_tree_db                         : show i2c tree db information.\n");
        printf("    get_vim_cpld_bus_id [VIM_ID]            : show VIM cpld i2c bus id.\n");
        printf("    get_vim_port_information                : show sfp information.\n");
        printf("    get_vim_sfp_ctl_support                 : show sfp control support list.\n");
        printf("    get_vim_eeprom [VIM_ID]                 : show VIM EEPROM.\n");
        printf("    vim_power_on [VIM_ID]                   : enable VIM power.\n");
        printf("    vim_power_off [VIM_ID]                  : disable VIM power.\n");

        printf("    (Warning! Please be careful to write a value to SFP,\n");
        printf("     you should keep the original value to prevent lose it forever.)\n");
        printf("    sfprb [PORT] [ADDR] : read a byte from sfp transeciver.\n");
        printf("    sfprw [PORT] [ADDR] : read a word from sfp transeciver.\n");
        printf("    sfpwb [PORT] [ADDR] [VALUE] : write a byte to sfp transeciver.\n");
        printf("    sfpww [PORT] [ADDR] [VALUE] : write a word to sfp transeciver.\n");

        printf("                        [PORT] is the port index start from 0.\n");
        printf("                        [ADDR] is the address to read/write.\n");
        printf("                        [VALUE] is the value to read/write.\n");


    }
    else if (argc > 0 && !strcmp(argv[0], "test")) /* for RD debug test */
    {
        diag_flag_set(DIAG_FLAG_ON);
        onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_BLINKING);
    }
    else
    {}

    return 0;
}





