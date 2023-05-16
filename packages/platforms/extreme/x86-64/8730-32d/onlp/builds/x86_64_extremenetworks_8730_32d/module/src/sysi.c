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

#include "x86_64_extremenetworks_8730_32d_int.h"
#include "x86_64_extremenetworks_8730_32d_log.h"
#include "platform_lib.h"
#include <fcntl.h>
#include <unistd.h>

#define DEBUG                               0
#define SYSTEM_CPLD_MAX_STRLEN              8

#define PORT_CPLD_REVISION_FORMAT		   	"/sys/bus/i2c/devices/%s"
#define POWER_CPLD_REVISION_FORMAT		    "/sys/bus/platform/devices/8730_pwr_cpld/%s"

#define PLATFORM_STRING "x86-64-extremenetworks-8730-32d-r0"

typedef struct cpld_version {
	char *cpld_path_format;
	char *attr_name;
	int   version;
	char *description;
} cpld_version_t;


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
	cpld_version_t cplds[] = { { PORT_CPLD_REVISION_FORMAT, "4-0057/version", 0, "Port-CPLD#0"},
				   			   { PORT_CPLD_REVISION_FORMAT, "7-0057/version", 0, "Port-CPLD#1"},
				   			   { POWER_CPLD_REVISION_FORMAT, "pwr_cpld_ver", 0, "Power-CPLD"} };

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
onlp_sysi_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);
    return ONLP_STATUS_OK;
}

int onlp_sysi_debug_diag_sfp_status(void)
{
    int i = 0;
    int status = 0;
    for (i = 0; i < NUM_OF_SFP_PORT; i++)
    {
        status = onlp_sfpi_is_present(i);
        printf("SFP#%d \n", i+1);
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


    printf("POWER o     STATUS  o     FAN o     PSU o     SECURITY o   \n");
    printf("\n");

    printf("[Stop platform manage ...]\n");

    diag_debug_pause_platform_manage_on();

    sleep(1);

    printf("[Set All LED to OFF ...]\n");
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_PWR), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_FAN), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_SEC), ONLP_LED_MODE_OFF);	
    printf("<Press Any Key to Continue>\n");
    getchar();

    /* POWER LED */
    printf("[Set POWER LED to ONLP_LED_MODE_GREEN ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PWR), ONLP_LED_MODE_GREEN);
    printf("<Press Any Key to Continue>\n");
    getchar();

    /* STATUS LED */
    printf("[Set STATUS LED to ONLP_LED_MODE_GREEN ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_GREEN);
    printf("<Press Any Key to Continue>\n");
    getchar();
	
    printf("[Set STATUS LED to ONLP_LED_MODE_ORANGE ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_ORANGE);
    printf("<Press Any Key to Continue>\n");
    getchar();
	
	printf("[Set STATUS LED to ONLP_LED_MODE_BLINKING(Blinking Amber-Green) ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_BLINKING);
    printf("<Press Any Key to Continue>\n");
    getchar();

	/* FAN LED */
    printf("[Set FAN LED to ONLP_LED_MODE_GREEN ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_FAN), ONLP_LED_MODE_GREEN);
    printf("<Press Any Key to Continue>\n");
    getchar();
	
    printf("[Set FAN LED to ONLP_LED_MODE_ORANGE ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_FAN), ONLP_LED_MODE_ORANGE);
    printf("<Press Any Key to Continue>\n");
    getchar();
	
	printf("[Set FAN LED to ONLP_LED_MODE_BLINKING(Blinking Amber-Green) ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_FAN), ONLP_LED_MODE_BLINKING);
    printf("<Press Any Key to Continue>\n");
    getchar();

    /* PSU LED */
    printf("[Set PSU LED to ONLP_LED_MODE_GREEN ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_GREEN);
    printf("<Press Any Key to Continue>\n");
    getchar();
    printf("[Set PSU LED to ONLP_LED_MODE_GREEN_BLINKING ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_GREEN_BLINKING);
    printf("<Press Any Key to Continue>\n");
    getchar();
    printf("[Set PSU LED to ONLP_LED_MODE_ORANGE ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_ORANGE);
    printf("<Press Any Key to Continue>\n");
    getchar();
    printf("[Set PSU LED to ONLP_LED_MODE_ORANGE_BLINKING ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_ORANGE_BLINKING);
    printf("<Press Any Key to Continue>\n");
    getchar();
	printf("[Set PSU LED to ONLP_LED_MODE_BLINKING(Blinking Amber-Green) ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_BLINKING);
    printf("<Press Any Key to Continue>\n");
    getchar();
    
    /* SECURITY LED */
    printf("[Set SEC LED to ONLP_LED_MODE_BLUE ...]\n");
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_SEC), ONLP_LED_MODE_BLUE);
    printf("<Press Any Key to Continue>\n");
    getchar();

    printf("[Set All LED to OFF ...]\n");
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_PWR), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_FAN), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_PSU), ONLP_LED_MODE_OFF);
    onlp_ledi_set(ONLP_LED_ID_CREATE(LED_SEC), ONLP_LED_MODE_OFF);

    printf("[Restart platform manage ...]\n");
    onlp_ledi_init();

    diag_debug_pause_platform_manage_off();

    return 0;
}

#define SFP_DIAG_OFFSET 118     // EXTENDED MODULE CONTROL/STATUS BYTES  in SFF-8472 standard
#define QSFP_DIAG_PAGE_SELECT 127 //page select byte in QSFP-DD-CMIS standard
#define QSFP_DIAG_OFFSET 222      //the reserved bytes 214~223 of page 13h support r/w in QSFP-DD-CMIS standard
#define QSFP_DIAG_PAGE	 0x13	  //the reserved bytes 214~223 of page 13h support r/w in QSFP-DD-CMIS standard
#define SFP_DIAG_PATTEN_B 0xAA
#define SFP_DIAG_PATTEN_W 0xABCD

int onlp_sysi_debug_diag_sfp(int index)
{
    uint8_t *data = NULL;
    int rv = 0;

    uint8_t org_b = 0;    
    uint16_t org_w = 0;
    uint8_t temp_b = 0;
    uint16_t temp_w = 0;
	uint8_t page_org_b = 0;

    int offset = 0, addr = 0;
	int page_offset = 0;

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

    if (index >= SFP_START_INDEX && index <= (SFP_START_INDEX+NUM_OF_SFP_PORT))
    {
		addr = SFP_PLUS_EEPROM_I2C_ADDR;
	
        if (IS_QSFP_PORT(index))
        {
        	page_offset = QSFP_DIAG_PAGE_SELECT;
            offset = QSFP_DIAG_OFFSET;
        }
        else
        {
            goto DONE;
        }

    }

    //BYTE
    printf("Read/Write byte test...\n");

	if (IS_QSFP_PORT(index))
    {
        page_org_b = onlp_sfpi_dev_readb(index, addr, page_offset);
    	if (page_org_b < 0)
    	{
        	printf("Error, read failed!\n");
        	goto DONE;
    	}

		rv = onlp_sfpi_dev_writeb(index, addr, page_offset, QSFP_DIAG_PAGE);
    	if (rv < 0)
    	{
        	printf("Error, write failed!\n");
        	goto DONE;
    	}
    	sleep(2);
    	temp_b = onlp_sfpi_dev_readb(index, addr, page_offset);
    	if (temp_b < 0)
    	{
        	printf("Error, read failed!\n");
        	goto DONE;
    	}
    	if (temp_b != QSFP_DIAG_PAGE)
    	{
        	printf("Error, can not change page!\n");
        	goto DONE;
    	}
    }
	
    org_b = onlp_sfpi_dev_readb(index, addr, offset);
    if (org_b < 0)
    {
        printf("Error, read failed!\n");
        goto DONE;
    }

    rv = onlp_sfpi_dev_writeb(index, addr, offset, SFP_DIAG_PATTEN_B);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto DONE;
    }
    sleep(2);
    temp_b = onlp_sfpi_dev_readb(index, addr, offset);
    if (temp_b < 0)
    {
        printf("Error, read failed!\n");
        goto DONE;
    }
    if (temp_b != SFP_DIAG_PATTEN_B)
    {
        printf("Error, mismatch!\n");
        goto DONE;
    }
    rv = onlp_sfpi_dev_writeb(index, addr, offset, org_b);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto DONE;
    }
    sleep(2);
    //WORD
    printf("Read/Write word test...\n");
    org_w = onlp_sfpi_dev_readw(index, addr, offset);
    if (org_w < 0)
    {
        printf("Error, read failed!\n");
        goto DONE;
    }
    rv = onlp_sfpi_dev_writew(index, addr, offset, SFP_DIAG_PATTEN_W);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto DONE;
    }
    sleep(2);
    temp_w = onlp_sfpi_dev_readw(index, addr, offset);
    if (temp_w < 0)
    {
        printf("Error, read failed!\n");
        goto DONE;
    }
    if (temp_w != SFP_DIAG_PATTEN_W)
    {
        printf("Error, mismatch!\n");
        goto DONE;
    }
    rv = onlp_sfpi_dev_writew(index, addr, offset, org_w);
    if (rv < 0)
    {
        printf("Error, write failed!\n");
        goto DONE;
    }

	if (IS_QSFP_PORT(index))
    {
		rv = onlp_sfpi_dev_writeb(index, addr, page_offset, page_org_b);
    	if (rv < 0)
    	{
        	printf("Error, write failed!\n");
        	goto DONE;
    	}
    }

DONE:
    return 0;
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
    printf("[Set %s... to 1]\n", sfp_control_to_str(ONLP_SFP_CONTROL_RESET));
    onlp_sfpi_control_set(index, ONLP_SFP_CONTROL_RESET, 1);
    sleep(1);
    printf("[Get %s... ]\n", sfp_control_to_str(ONLP_SFP_CONTROL_RESET));
    onlp_sfpi_control_get(index, ONLP_SFP_CONTROL_RESET, &val);
    printf("<Press Any Key to Continue>\n");
    getchar();

    printf("[Set %s... to 0]\n", sfp_control_to_str(ONLP_SFP_CONTROL_RESET));
    onlp_sfpi_control_set(index, ONLP_SFP_CONTROL_RESET, 0);
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

    return 0;
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
        if (argc != 2)
        {
            printf("Parameter error, format: onlpdump debugi sfp_dom [PORT]\n");
            return -1;
        }
        port_index = atoi(argv[1]);
        if (port_index <= SFP_START_INDEX || port_index > (SFP_START_INDEX+NUM_OF_SFP_PORT))
        {
            printf("Parameter error, PORT out of range.\n");
            return -1;
        }
        printf("DIAG for SFP DOM #%d: \n", port_index - 1);
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sysi_debug_diag_sfp_dom(port_index - 1);
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "sfp_ctrl_set"))
    {
        int port_index = 0, ctrl = 0, val = 0;
        if (argc != 4)
        {
            printf("Parameter error, format: onlpdump debugi sfp_ctrl_set [PORT] [CTRL] [VALUE]\n");
            return -1;
        }
        port_index = atoi(argv[1]);
        if (port_index <= SFP_START_INDEX || port_index > (SFP_START_INDEX+NUM_OF_SFP_PORT))
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
        if (argc != 3)
        {
            printf("Parameter error, format: onlpdump debugi sfp_ctrl_get [PORT] [CTRL] \n");
            return -1;
        }
        port_index = atoi(argv[1]);
        if (port_index <= SFP_START_INDEX || port_index > (SFP_START_INDEX+NUM_OF_SFP_PORT))
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
        if (argc != 2)
        {
            printf("Parameter error, format: onlpdump debugi sfp_ctrl [PORT]\n");
            return -1;
        }
        port_index = atoi(argv[1]);
        if (port_index <= SFP_START_INDEX || port_index > (SFP_START_INDEX+NUM_OF_SFP_PORT))
        {
            printf("Parameter error, PORT out of range.\n");
            return -1;
        }

        printf("DIAG for SFP Control #%d: \n", port_index - 1);
        diag_flag_set(DIAG_FLAG_ON);
        onlp_sysi_debug_diag_sfp_ctrl(port_index - 1);
        diag_flag_set(DIAG_FLAG_OFF);
    }
    else if (argc > 0 && !strcmp(argv[0], "sfp"))
    {
        if (argc > 1)
        {
            int port_index = atoi(argv[1]);
            if (port_index <= SFP_START_INDEX || port_index > (SFP_START_INDEX+NUM_OF_SFP_PORT))
            {
                printf("Parameter error, PORT out of range.\n");
                return -1;
            }
            printf("DIAG for SFP#%d: \n", port_index - 1);
            diag_flag_set(DIAG_FLAG_ON);
            onlp_sysi_debug_diag_sfp(port_index - 1);
            diag_flag_set(DIAG_FLAG_OFF);
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
    else if (argc > 0 && !strcmp(argv[0], "sfpwb")) //write byte
    {
        int port;
        uint8_t addr, value;

        if (argc == 4)
        {
            port = atoi(argv[1]);
            addr = (uint8_t)atoi(argv[2]);
            value = (uint8_t)atoi(argv[3]);

            if (port <= SFP_START_INDEX || port > (SFP_START_INDEX+NUM_OF_SFP_PORT))
            {
                printf("Parameter error, PORT out of range.\n");
                return -1;
            }

            diag_flag_set(DIAG_FLAG_ON);
            onlp_sfpi_dev_writeb(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr, value);
            diag_flag_set(DIAG_FLAG_OFF);
        }
        else
        {
            printf("Parameter error, format: onlpdump debugi sfpwb [PORT] [ADDR] [VALUE]\n");
            return -1;
        }

    }
    else if (argc > 0 && !strcmp(argv[0], "sfprb")) //read byte
    {
        int port;
        uint8_t addr;
        if (argc == 3)
        {
            port = atoi(argv[1]);
            addr = (uint8_t)atoi(argv[2]);

            if (port <= SFP_START_INDEX || port > (SFP_START_INDEX+NUM_OF_SFP_PORT))
            {
                printf("Parameter error, PORT out of range.\n");
                return -1;
            }

            diag_flag_set(DIAG_FLAG_ON);
            onlp_sfpi_dev_readb(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr);
            diag_flag_set(DIAG_FLAG_OFF);
        }
        else
        {
            printf("Parameter error, format: onlpdump debugi sfprb [PORT] [ADDR]\n");
            return -1;
        }
    }
    else if (argc > 0 && !strcmp(argv[0], "sfpww")) //write word
    {
        int port;
        uint16_t value;
        uint8_t addr;

        if (argc == 4)
        {
            port = atoi(argv[1]);
            addr = (uint8_t)atoi(argv[2]);
            value = (uint16_t)atoi(argv[3]);

            if (port <= SFP_START_INDEX || port > (SFP_START_INDEX+NUM_OF_SFP_PORT))
            {
                printf("Parameter error, PORT out of range.\n");
                return -1;
            }

            diag_flag_set(DIAG_FLAG_ON);
            onlp_sfpi_dev_writew(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr, value);
            diag_flag_set(DIAG_FLAG_OFF);
        }
        else
        {
            printf("Parameter error, format: onlpdump debugi sfpwb [PORT] [ADDR] [VALUE]\n");
            return -1;
        }
    }
    else if (argc > 0 && !strcmp(argv[0], "sfprw")) //read word
    {
        int port;
        uint8_t addr;
        if (argc == 3)
        {
            port = atoi(argv[1]);
            addr = (uint8_t)atoi(argv[2]);

            if (port <= SFP_START_INDEX || port > (SFP_START_INDEX+NUM_OF_SFP_PORT))
            {
                printf("Parameter error, PORT out of range.\n");
                return -1;
            }

            diag_flag_set(DIAG_FLAG_ON);
            onlp_sfpi_dev_readw(port - 1, SFP_PLUS_EEPROM_I2C_ADDR, addr);
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
    else if (argc > 0 && !strcmp(argv[0], "help"))
    {
        printf("\nUsage: onlpdump debugi [OPTION]\n");
        printf("    help                : this message.\n");
        printf("    trace_on            : turn on ONLPI debug trace message output on screen.\n");
        printf("    trace_off           : turn off ONLPI debug trace message output on screen.\n");
        printf("    sys                 : run system ONLPI diagnostic function.\n");
        printf("    fan_status          : run fan status ONLPI diagnostic function.\n");
        printf("    led                 : run LED ONLPI diagnostic function.\n");
        printf("    psu                 : run psu ONLPI diagnostic function.\n");
        printf("    thermal             : run thermal ONLPI diagnostic function.\n");
        printf("    sfp                 : run sfp ONLPI diagnostic function.\n");
        printf("    sfp [PORT]          : run sfp ONLPI diagnostic function.\n");
        printf("    sfp_dom [PORT]      : run sfp dom ONLPI diagnostic function.\n");
        printf("    sfp_ctrl [PORT]     : run sfp control ONLPI diagnostic function.\n");

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
    else if (argc > 0 && !strcmp(argv[0], "test")) //for RD debug test
    {
        diag_flag_set(DIAG_FLAG_ON);
        onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_STAT), ONLP_LED_MODE_BLINKING);
    }
    else
    {}

    return 0;
}





