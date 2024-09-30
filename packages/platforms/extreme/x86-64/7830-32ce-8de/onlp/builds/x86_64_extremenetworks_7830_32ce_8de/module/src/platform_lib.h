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
#ifndef __PLATFORM_LIB_H__
#define __PLATFORM_LIB_H__

#include <onlplib/file.h>
#include <onlplib/shlocks.h>    /* onlp_shmem_create() */
#include <semaphore.h>          /* sem_init() */
#include "x86_64_extremenetworks_7830_32ce_8de_log.h"


#define PSU1_ID 1
#define PSU2_ID 2
#define PSU_FAN1_ID 1

#define CHASSIS_LED_COUNT       5
#define CHASSIS_PSU_COUNT       2
#define PER_PSU_FAN_COUNT   	1
#define PER_PSU_THERMAL_COUNT   3

#define CHASSIS_FAN_COUNT (ONLP_FAN_ID_MAX - (CHASSIS_PSU_COUNT * PER_PSU_FAN_COUNT) - 1)
#define CHASSIS_THERMAL_COUNT (ONLP_THERMAL_ID_MAX - (CHASSIS_PSU_COUNT * PER_PSU_THERMAL_COUNT) - 1)

#define PSU_HWMON_PREFIX            	"/sys/bus/platform/devices/7830_psu/"
#define PSU_EEPROM_PREFIX            	"/sys/bus/platform/devices/7830_psu/"

#define FAN_BOARD_PATH	                "/sys/bus/platform/devices/7830_fan/"

#define ONIE_EEPROM_PATH                "/sys/bus/i2c/devices/57-0055/eeprom"

#define NUM_OF_SFP_PORT 	                40      /* 32 * 100G + 8 * 400G*/
#define NUM_OF_IOBM_PORT 	       		 	2       /* 1 * 100G, 1 * 10G */
#define NUM_OF_IOBM_QSFP28_PORT 	        1       /* 1 * 100G */
#define NUM_OF_IOBM_SFP_PORT 	            1       /* 1 * 10G */
#define NUM_OF_QSFP_PORT_CPLD 	            2
#define NUM_OF_QSFP_PER_PORT_CPLD           2

#define SFP_START_INDEX                     0       /* Both QSFP and SFP */
#define SFP_PLUS_EEPROM_I2C_ADDR            0x50    /* SFP+ EEPROM Physical Address in the I2C */  
#define SFP_DOM_EEPROM_I2C_ADDR             0x51

#define QSFP_PORT_INDEX_START               0
#define QSFP_PORT_INDEX_END                 39
#define IOBM_QSFP28_PORT_INDEX              40
#define IOBM_SFP_PORT_INDEX                 41
#define QSFPDD_PORT_INDEX_START             32
#define QSFPDD_PORT_INDEX_END               39
#define QSFP28_PORT_INDEX_START             0
#define QSFP28_PORT_INDEX_END               31



#define IS_QSFP_PORT(_port) (_port >= QSFP_PORT_INDEX_START && _port <= QSFP_PORT_INDEX_END)
#define IS_IOBM_PORT(_port) (_port >= IOBM_QSFP28_PORT_INDEX && _port <= IOBM_SFP_PORT_INDEX)
#define IS_IOBM_QSFP28_PORT(_port) (_port == IOBM_QSFP28_PORT_INDEX)
#define IS_IOBM_SFP_PORT(_port) (_port == IOBM_SFP_PORT_INDEX)
#define IS_QSFPDD_PORT(_port) (_port >= QSFPDD_PORT_INDEX_START && _port <= QSFPDD_PORT_INDEX_END)
#define IS_QSFP28_PORT(_port) (_port >= QSFP28_PORT_INDEX_START && _port <= QSFP28_PORT_INDEX_END)

/* For share memory */
#define ONLP_OID_TYPE_VIM                   8
#define ONLP_VIMI_SHM_KEY                   (0xF001100 | ONLP_OID_TYPE_VIM)
#define STORAGE_SIZE                        32
#define STORAGE_ID                          "i2c_tree_db"
#define DEBUG_FLAG_SAMPLE                   0   /* 0: Execute Alpha's solution, only for development and testing. 
                                                 * 1: Execute sample hard code. Example only for this case: insert VIM 1 8DE, VIM 2 not present
                                                 */

/* For VIM card */
#define I2C_BUS_CHANNEL_COUNT               8
#define VIM1_ID                             1
#define VIM2_ID                             2
#define VIM_POWER_CPLD_ID                   1
#define VIM_PORT_CPLD_ID                    2
#define VIM_START_INDEX                     (NUM_OF_SFP_PORT + NUM_OF_IOBM_PORT)  
#define VIM_PRESENT                         0
#define VIM_NOT_PRESENT                     1
#define VIM_POWER_GOOD                      1
#define VIM_POWER_FAIL                      0
#define VIM1_MAX_PCA_COUNT                  4
#define VIM2_MAX_PCA_COUNT                  4
#define VIM_MAX_PCA_COUNT                   (VIM1_MAX_PCA_COUNT + VIM2_MAX_PCA_COUNT)
#define VIM_TYPE_COUNT                      5
#define PCA9548_NOT_USE                     0
#define IS_VIM_PORT(_port, _vim_end_index) (_port >= VIM_START_INDEX && _port < _vim_end_index)
#define VIM_PWR_CTRL_ENA_MP5990             0x02
#define VIM_PWR_CTRL_ENA_DCDC               0x03
#define VIM_PWR_CTRL_DIS_PWR                0x1c

/* VIM Power CPLD access from BMC (Board ID) */
#define VIM_BOARD_ID_PATH                   "/sys/bus/platform/devices/7830_bmc_vim_pwr_cpld/vim_%d_board_id"

/* VIM EEPROM access from BMC */
#define VIM_EEPROM_PATH                     "/sys/bus/platform/devices/7830_vim_eeprom/vim%d_eeprom"

/* VIM Power CPLD access from CPU (8DE: port 1~8. 16CE: port 1~16. 24CE,24YE: port 1~12) */
#define VIM_OPTOE_PRESENT_PWR_CPLD_PATH     "/sys/bus/i2c/devices/%d-005c/present_%d"
#define VIM_OPTOE_RESET_PWR_CPLD_PATH       "/sys/bus/i2c/devices/%d-005c/rst_mod_%d"
#define VIM_OPTOE_LP_MODE_PWR_CPLD_PATH     "/sys/bus/i2c/devices/%d-005c/lp_mode_%d"
#define VIM_OPTOE_MOD_SEL_PWR_CPLD_PATH     "/sys/bus/i2c/devices/%d-005c/mod_sel_%d"
#define VIM_OPTOE_TX_FAULT_PWR_CPLD_PATH    "/sys/bus/i2c/devices/%d-005c/tx_fault_%d"
#define VIM_OPTOE_TX_DIS_PWR_CPLD_PATH      "/sys/bus/i2c/devices/%d-005c/tx_dis_%d"
#define VIM_OPTOE_RX_LOSS_PWR_CPLD_PATH     "/sys/bus/i2c/devices/%d-005c/rx_los_%d"


/* System CPLD access from CPU */
#define VIM_PRESENT_PATH                    "/sys/bus/i2c/devices/0-006e/vim_%d_present"
#define VIM_RESET_PATH                      "/sys/bus/i2c/devices/0-006e/vim_%d_reset"
#define VIM_POWER_CONTROL_PATH              "/sys/bus/i2c/devices/0-006e/vim_%d_pwr_ctrl"
#define VIM_POWER_GOOD_PATH                 "/sys/bus/i2c/devices/0-006e/vim_%d_pwr_good"

/* VIM Port CPLD access from CPU (24CE,24YE: port 13~24) */
#define VIM_OPTOE_PRESENT_PORT_CPLD_PATH    "/sys/bus/i2c/devices/%d-0058/present_%d"
#define VIM_OPTOE_RESET_PORT_CPLD_PATH      "/sys/bus/i2c/devices/%d-0058/rst_mod_%d"
#define VIM_OPTOE_LP_MODE_PORT_CPLD_PATH    "/sys/bus/i2c/devices/%d-0058/lp_mode_%d"
#define VIM_OPTOE_MOD_SEL_PORT_CPLD_PATH    "/sys/bus/i2c/devices/%d-0058/mod_sel_%d"
#define VIM_OPTOE_TX_FAULT_PORT_CPLD_PATH   "/sys/bus/i2c/devices/%d-0058/tx_fault_%d"
#define VIM_OPTOE_TX_DIS_PORT_CPLD_PATH     "/sys/bus/i2c/devices/%d-0058/tx_dis_%d"
#define VIM_OPTOE_RX_LOSS_PORT_CPLD_PATH    "/sys/bus/i2c/devices/%d-0058/rx_los_%d"
#define VIM_OPTOE_EEPROM_PATH               "/sys/bus/i2c/devices/%d-0050/eeprom"
#define VIM_OPTOE_DOM_PATH                  "/sys/bus/i2c/devices/%d-0051/eeprom"

/* Create I2C tree echo command */
#define CMD_SIZE                            128
#define PCA9548_3_INDEX                     1
#define PCA9548_4_INDEX                     2
#define VIM_PWR_CPLD_INDEX                  3
#define VIM_PROT_CPLD_INDEX                 4
#define PCA9548_1_ADDR                      0x76
#define PCA9548_0_ADDR                      0x70
#define PCA9548_3_ADDR                      0x71
#define PCA9548_4_ADDR                      0x72
#define VIM1_PCA9548_1_BUS_ID               1
#define VIM2_PCA9548_1_BUS_ID               2
#define VIM_CPLD0_ADDR                      0x5C
#define VIM_CPLD1_ADDR                      0x58
#define VIM_SFP_EEPROM_ADDR                 0x50

enum vim_port {
    NOT_VIM_PORT, 
    VIM_PORT
};

enum vim_type_id {
    VIM_8DE, 
    VIM_16CE,
    VIM_24CE,
    VIM_24YE,
    VIM_NONE, 
    VIM_TYPE_ID_MAX
};

typedef enum psu_type {
    PSU_TYPE_UNKNOWN,
    PSU_TYPE_AC_F2B,
    PSU_TYPE_AC_B2F,
    PSU_TYPE_DC_48V_F2B,
    PSU_TYPE_DC_48V_B2F
} psu_type_t;


enum onlp_thermal_id
{
    THERMAL_RESERVED = 0,
    THERMAL_1_ON_MAINBOARD,  /* Main Board Bottom CPU Temp */
    THERMAL_2_ON_MAINBOARD,  /* Main Board Bottom TMP75_0 Temp (AFO)*/
    THERMAL_3_ON_MAINBOARD,  /* Main Board Bottom TMP75_1 Temp (HOT Spot) */
    THERMAL_4_ON_MAINBOARD,  /* Main Board Bottom TMP75_2 Temp (AFI) */
    THERMAL_5_ON_VIM1,       /* VIM1 TMP75 Temp (VIM1 TMP75) */
    THERMAL_6_ON_VIM2,       /* VIM1 TMP75 Temp (VIM2 TMP75) */
    THERMAL_1_ON_PSU1,
    THERMAL_2_ON_PSU1,
    THERMAL_3_ON_PSU1,
    THERMAL_1_ON_PSU2,
    THERMAL_2_ON_PSU2,
    THERMAL_3_ON_PSU2,
    ONLP_THERMAL_ID_MAX,
};


enum fan_id {
    FAN_1_ON_FAN_BOARD = 1,
    FAN_2_ON_FAN_BOARD,
    FAN_3_ON_FAN_BOARD,
    FAN_4_ON_FAN_BOARD,
    FAN_1_ON_PSU_1,
    FAN_1_ON_PSU_2,
    ONLP_FAN_ID_MAX,
};

/* FAN related data
 */
enum onlp_fan_id
{
    FAN_RESERVED = 0,
    FAN_1,
    FAN_2,
    FAN_3,
    FAN_4,
    FAN_1_ON_PSU1,
    FAN_1_ON_PSU2,
};

enum onlp_fan_fault_status
{
    FAN_FAULT,
    FAN_OK,
};


/* 
 * LED ID (need to sync with "enum onlp_led_id" defined in ledi.c)
 */

enum onlp_led_id
{
    LED_RESERVED = 0,
	LED_PWR,
    LED_STAT,
    LED_FAN,
    LED_PSU,
    LED_SEC  
};


#define DIAG_FLAG_ON 1
#define DIAG_FLAG_OFF 0
char diag_flag_set(char d);
char diag_flag_get(void);

char diag_debug_trace_on(void);
char diag_debug_trace_off(void);
char diag_debug_trace_check(void);

char diag_debug_pause_platform_manage_on(void);
char diag_debug_pause_platform_manage_off(void);
char diag_debug_pause_platform_manage_check(void);

#define DIAG_TRACE(fmt,args...) if(diag_debug_trace_check()) printf("\n[TRACE]"fmt"\n", args)
#define DIAG_PRINT(fmt,args...) DIAG_TRACE(fmt,args);else if(diag_flag_get()) printf("[DIAG]"fmt"\n", args) 

char* sfp_control_to_str(int value);
char* vim_sfp_control_to_str(int value);
char *rtrim(char *str);
char *ltrim(char *str);
char *trim(char *str);
psu_type_t psu_type_get(int id, char* modelname, int modelname_len);
uint32_t pltfm_create_sem (sem_t *mutex);

#endif  /* __PLATFORM_LIB_H__ */
