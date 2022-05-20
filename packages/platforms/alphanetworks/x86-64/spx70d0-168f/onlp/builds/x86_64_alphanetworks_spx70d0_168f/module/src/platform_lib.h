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
#ifndef __PLATFORM_LIB_H__
#define __PLATFORM_LIB_H__

#include "x86_64_alphanetworks_spx70d0_168f_log.h"

#define DEBUG_FLAG 0

#define ONIE_EEPROM_PATH                "/sys/bus/i2c/devices/33-0055/eeprom"

#define PSU1_ID 1
#define PSU2_ID 2

#define PSU_STATUS_PRESENT     1
#define PSU_STATUS_POWER_GOOD  1

#define PSU_HWMON_PATH  "/sys/devices/platform/spx70d0_pwr_cpld/"
#define PSU_PMBUS_PATH  "/sys/devices/platform/spx70d0_168f_psu/"

#define CHASSIS_LED_COUNT      6
#define CHASSIS_PSU_COUNT      2

#define FAN_BOARD_PATH	                "/sys/devices/platform/spx70d0_fan/"
#define FAN_NODE(node)	                FAN_BOARD_PATH#node

#define SFP_START_INDEX         0
#define NUM_OF_SFPP_PORT        16 /* 16 10G SFP+ Port 1-16  */
#define NUM_OF_SFP28_PORT       6  /* 6 25G SFP28 Port 1-6  */
#define NUM_OF_QSFP28_PORT      2  /* 2 100G QSFP28 Port */
#define NUM_OF_SFP_PORT (NUM_OF_SFPP_PORT + NUM_OF_SFP28_PORT + NUM_OF_QSFP28_PORT)  

#define SFP_PLUS_EEPROM_I2C_ADDR        0x50  /* SFP+ EEPROM Physical Address in the I2C */  
#define SFP_DOM_EEPROM_I2C_ADDR         0x51

enum sfp_port
{
	SFP_PORT_100G_QSFP28_1 = 0,
	SFP_PORT_100G_QSFP28_2 = 1,
	SFP_PORT_25G_SFP28_1 = 2,
	SFP_PORT_25G_SFP28_2 = 3,
	SFP_PORT_25G_SFP28_3 = 4,
	SFP_PORT_25G_SFP28_4 = 5,
	SFP_PORT_25G_SFP28_5 = 6,
	SFP_PORT_25G_SFP28_6 = 7,
	SFP_PORT_10G_SFPP_1 = 8,
	SFP_PORT_10G_SFPP_2 = 9,
	SFP_PORT_10G_SFPP_3 = 10,
	SFP_PORT_10G_SFPP_4 = 11,
	SFP_PORT_10G_SFPP_5 = 12,
	SFP_PORT_10G_SFPP_6 = 13,
	SFP_PORT_10G_SFPP_7 = 14,
	SFP_PORT_10G_SFPP_8 = 15,	
	SFP_PORT_10G_SFPP_9 = 16,
	SFP_PORT_10G_SFPP_10 = 17,
	SFP_PORT_10G_SFPP_11 = 18,
	SFP_PORT_10G_SFPP_12 = 19,
	SFP_PORT_10G_SFPP_13 = 20,
	SFP_PORT_10G_SFPP_14 = 21,
	SFP_PORT_10G_SFPP_15 = 22,
	SFP_PORT_10G_SFPP_16 = 23
};

#define SFPP_PORT_INDEX_START				SFP_PORT_10G_SFPP_1
#define SFPP_PORT_INDEX_END					SFP_PORT_10G_SFPP_16
#define SFP28_PORT_INDEX_START              SFP_PORT_25G_SFP28_1
#define SFP28_PORT_INDEX_END                SFP_PORT_25G_SFP28_6
#define QSFP_PORT_INDEX_START               SFP_PORT_100G_QSFP28_1
#define QSFP_PORT_INDEX_END                 SFP_PORT_100G_QSFP28_2

#define IS_SFP_PORT(_port)  (_port >= SFPP_PORT_INDEX_START && _port <= SFPP_PORT_INDEX_END)
#define IS_SFP28_PORT(_port)  (_port >= SFP28_PORT_INDEX_START && _port <= SFP28_PORT_INDEX_END)
#define IS_QSFP_PORT(_port) (_port >= QSFP_PORT_INDEX_START && _port <= QSFP_PORT_INDEX_END)

typedef long long               I64_T;      /* 64-bit signed   */
typedef unsigned long long      UI64_T;     /* 64-bit unsigned */
typedef long                    I32_T;      /* 32-bit signed   */
typedef unsigned long           UI32_T;     /* 32-bit unsigned */
typedef short int               I16_T;      /* 16-bit signed   */
typedef unsigned short int      UI16_T;     /* 16-bit unsigned */
typedef char                    I8_T;       /* 8-bit signed    */
typedef unsigned char           UI8_T;      /* 8-bit unsigned  */

/*----------------------------------------------------------*/
/*  BIT operation                                           */
/*----------------------------------------------------------*/
#define UTL_TEST_BITS(__type__,__var__,__pos__)                         \
        (__type__)((((__type__)(__var__)>>(__type__)(__pos__))&(__type__)1)?(__type__)1:(__type__)0)

#define UTL_LEFT_SHIFT_BITS(__type__,__range__,__var__,__sft_bits__)    \
        (((__type__)(__sft_bits__)>=(__range__))?0:((__type__)(__var__)<<(__type__)(__sft_bits__)))

#define UTL_RIGHT_SHIFT_BITS(__type__,__range__,__var__,__sft_bits__)   \
        (((__type__)__sft_bits__>=(__range__))?(__type__)(__var__):((__type__)(__var__)>>(__type__)(__sft_bits__)))

#define UTL_SET_BITS(__type__,__range__,__var__,__pos__)                \
        ((__type__)(__var__)|UTL_LEFT_SHIFT_BITS(__type__,__range__,(__type__)1U,(__type__)(__pos__)))

#define UTL_RESET_BITS(__type__,__range__,__var__,__pos__)              \
        ((__type__)(__var__)&~UTL_LEFT_SHIFT_BITS(__type__,__range__,(__type__)1U,(__type__)(__pos__)))

/*----------------------------------------------------------*/
/*  64 BIT operation                                        */
/*----------------------------------------------------------*/
#define UTL_TEST_BITS64(__var__,__pos__)     UTL_TEST_BITS(UI64_T,__var__,__pos__)

#define UTL_SET_BITS64(__var__,__pos__)      (__var__) = UTL_SET_BITS(UI64_T,64,__var__,__pos__)
#define UTL_RESET_BITS64(__var__,__pos__)    (__var__) = UTL_RESET_BITS(UI64_T,64,__var__,__pos__)

/*----------------------------------------------------------*/
/*  32 BIT operation                                        */
/*----------------------------------------------------------*/
/*   Usage: if( UTL_TEST_BITS32(val,2) )*/
/*   pos = 0~7, 0~15, 0~31              */
/*   0x1234 = 0001 0010 0011 0100       */
/*   UTL_TEST_BITS32( 0x1234, 2 ) ==> 1 */
/*   UTL_TEST_BITS32( 0x1234, 1 ) ==> 0 */
#define UTL_TEST_BITS32(__var__,__pos__)     UTL_TEST_BITS(UI32_T,__var__,__pos__)


/*   Usage: UTL_SET_BITS32( val, 6 )                    */
/*   pos = 0~7, 0~15, 0~31                                  */
/*   val = 0x0100 =               0000 0001 0000 0000       */
/*   UTL_SET_BITS32( val,6 )==>   0000 0001 0100 0000   */
/*   UTL_RESET_BITS32( val,8 )=>  0000 0000 0000 0000   */
#define UTL_SET_BITS32(__var__,__pos__)      (__var__) = UTL_SET_BITS(UI32_T,32,__var__,__pos__)
#define UTL_RESET_BITS32(__var__,__pos__)    (__var__) = UTL_RESET_BITS(UI32_T,32,__var__,__pos__)

/*----------------------------------------------------------*/
/*  16 BIT operation                                        */
/*----------------------------------------------------------*/
#define UTL_TEST_BITS16(__var__,__pos__)     UTL_TEST_BITS(UI16_T,__var__,__pos__)

#define UTL_SET_BITS16(__var__,__pos__)      (__var__) = UTL_SET_BITS(UI16_T,16,__var__,__pos__)
#define UTL_RESET_BITS16(__var__,__pos__)    (__var__) = UTL_RESET_BITS(UI16_T,16,__var__,__pos__)

/*----------------------------------------------------------*/
/*  8 BIT operation                                         */
/*----------------------------------------------------------*/
#define UTL_TEST_BITS8(__var__,__pos__)     UTL_TEST_BITS(UI8_T,__var__,__pos__)

#define UTL_SET_BITS8(__var__,__pos__)      (__var__) = UTL_SET_BITS(UI8_T,8,__var__,__pos__)
#define UTL_RESET_BITS8(__var__,__pos__)    (__var__) = UTL_RESET_BITS(UI8_T,16,__var__,__pos__)

typedef enum psu_type {
    PSU_TYPE_UNKNOWN,
    PSU_TYPE_AC_F2B,
    PSU_TYPE_AC_B2F,
    PSU_TYPE_DC_48V_F2B,
    PSU_TYPE_DC_48V_B2F
} psu_type_t;

/* THERMAL related data
 */
enum onlp_thermal_id
{
    THERMAL_RESERVED = 0,
	THERMAL_CPU_CORE,
    THERMAL_1_ON_MAIN_BROAD_AMBIENT,          /* 0x48 : The TMP1075 on Main Board for ambient */
    THERMAL_2_ON_MAIN_BROAD_100G_PORTS,       /* 0x49 : The TMP1075 on Main Board for 100G ports */
    THERMAL_3_ON_PON_BROAD_PSU_IN,            /* 0x4B : The TMP1075 on PON Board for PSU IN */
    THERMAL_4_ON_PON_BROAD_PON_MAC,           /* 0x4E : The TMP1075 on PON Board for PON MAC */
    THERMAL_5_ON_PON_BROAD_PON_PORTS,         /* 0x4C : The TMP1075 on PON Board for PON Ports */
    THERMAL_6_ON_MAIN_BROAD_TMP435_local,     /* 0x4D : The TMP435 on Main Board */
    THERMAL_7_ON_MAIN_BROAD_TMP435_remote,    /* 0x4D : The TMP435 on Main Board */
    THERMAL_8_psu1_temp1,
    THERMAL_9_psu2_temp1,
	ONLP_THERMAL_ID_MAX,
};
		
#define CHASSIS_THERMAL_COUNT (ONLP_THERMAL_ID_MAX - CHASSIS_PSU_COUNT - 1)



/* FAN related data
 */
enum onlp_fan_id
{
	FAN_1 = 1,	
    FAN_2,
    FAN_3,
    FAN_PSU1_0,
    FAN_PSU2_0,
	ONLP_FAN_ID_MAX,
};
		
#define CHASSIS_FAN_COUNT (ONLP_FAN_ID_MAX - CHASSIS_PSU_COUNT - 1)


/* 
 * LED ID (need to sync with "enum onlp_led_id" defined in ledi.c)
 */
enum onlp_led_id
{
    LED_RESERVED = 0,
    LED_POWER,
    LED_PSU1,
    LED_PSU2,
    LED_SYSTEM,
    LED_FAN,
    LED_LOC
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
psu_type_t psu_type_get(int id, char* modelname, int modelname_len);
int psu_serial_number_get(int id, char *serial, int serial_len);
int psu_pmbus_info_get(int id, char *node, int *value);
int psu_pmbus_info_set(int id, char *node, int value);

#endif  /* __PLATFORM_LIB_H__ */
