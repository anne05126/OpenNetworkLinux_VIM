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
#include <onlplib/file.h>
#include <onlplib/onie.h>
//#include <onlplib/i2c.h>
#include "platform_lib.h"

#include <onlplib/mmap.h>

#include <pthread.h>

#define LED_FORMAT "/sys/class/gpio/gpio%d/value"

/* Power LED and PSU LED are controlled by BMC */
#define CPLD_PSU_LED_ADDRESS_OFFSET        0x0A //PSU0 [1:0], PSU1 [3:2]
#define CPLD_POWER_LED_ADDRESS_OFFSET      0x0B //power [1:0]

/* 
  * reference DCGS_TYPE1_ Power_CPLD_Spec_v05_20190820, chap 3.13. 
  * 1. Bit[0] status is got from signal  V3P3_SB_PG and V2P5_SB_PG and V1P2_SB_PG and V1P15_SB_PG and V1P8_SB_PG. 
  * 2. Bit[1] when set to 1 ,PWR_LED_Y is blinking;
  * 3. Bit[1] when set to 0 ,PWR_LED_Y is depended by Bit[0] -- > Bit[0] = 0, PWR_LED_Y is blinking
  *                                                                                             -- > Bit[0] = 1, PWR_LED_Y is off
  */
#define POWER_LED_OFF                      0x00
#define POWER_LED_GREEN_SOLID              0x01
#define POWER_LED_AMBER_BLINKING_BIT0_L    0x02
#define POWER_LED_AMBER_BLINKING_BIT0_H    0x03 

/*
  * Bit[0] status is got from (PSU0_PRESENT_L = '0') and (PSU0_PWOK_L = '0'). 
  * Bit[1] when set to 1 , PSU0_LED_Y is blinking;
  * Bit[1] when set to 0 , PSU0_LED_Y is depended by Bit[0] -- > Bit[0] = 0, PSU0_LED_Y is blinking
  *                                                                                            -- > Bit[0] = 1, PSU0_LED_Y is off
  */
#define PSU_LED_OFF                        0x00
#define PSU_LED_GREEN_SOLID                0x01
#define PSU_LED_AMBER_BLINKING_BIT0_L      0x02
#define PSU_LED_AMBER_BLINKING_BIT0_H      0x03

#define SYSTEM_LED_OFF                     0x00
#define SYSTEM_LED_AMBER_SOLID             0x01
#define SYSTEM_LED_GREEN_SOLID             0x02
#define SYSTEM_LED_GREEN_BLINKING          0x03 

#define FAN_LED_OFF                        0x00
#define FAN_LED_AMBER_SOLID                0x01
#define FAN_LED_GREEN_SOLID                0x02

#define LOC_LED_OFF                        0x00
#define LOC_LED_BLUE_BLINKING              0x01

#define VALIDATE(_id)                           \
    do {                                        \
        if(!ONLP_OID_IS_LED(_id)) {             \
            return ONLP_STATUS_E_INVALID;       \
        }                                       \
    } while(0)

struct led_id_mode
{
    enum onlp_led_id led_id;
    onlp_led_mode_t mode;
    int hw_led_light_mode;
};

static struct led_id_mode led_id_mode_data[] = {
    { LED_POWER,   ONLP_LED_MODE_OFF,              POWER_LED_OFF },
    { LED_POWER,   ONLP_LED_MODE_GREEN,			   POWER_LED_GREEN_SOLID },
    { LED_POWER,   ONLP_LED_MODE_ORANGE_BLINKING,  POWER_LED_AMBER_BLINKING_BIT0_L},
    { LED_POWER,   ONLP_LED_MODE_ORANGE_BLINKING,  POWER_LED_AMBER_BLINKING_BIT0_H},    
    { LED_POWER,   ONLP_LED_MODE_ON,               POWER_LED_GREEN_SOLID },

    { LED_PSU,     ONLP_LED_MODE_OFF,              PSU_LED_OFF },
    { LED_PSU,     ONLP_LED_MODE_GREEN,            PSU_LED_GREEN_SOLID },
    { LED_PSU,     ONLP_LED_MODE_ORANGE_BLINKING,  PSU_LED_AMBER_BLINKING_BIT0_L},
    { LED_PSU,     ONLP_LED_MODE_ORANGE_BLINKING,  PSU_LED_AMBER_BLINKING_BIT0_H},    
    { LED_PSU,     ONLP_LED_MODE_ON,               PSU_LED_GREEN_SOLID },

    { LED_SYSTEM,  ONLP_LED_MODE_OFF,              SYSTEM_LED_OFF },
    { LED_SYSTEM,  ONLP_LED_MODE_GREEN,            SYSTEM_LED_GREEN_SOLID },
    { LED_SYSTEM,  ONLP_LED_MODE_GREEN_BLINKING,   SYSTEM_LED_GREEN_BLINKING },
    { LED_SYSTEM,  ONLP_LED_MODE_ORANGE,           SYSTEM_LED_AMBER_SOLID },
    { LED_SYSTEM,  ONLP_LED_MODE_ON,               SYSTEM_LED_GREEN_SOLID },

    { LED_FAN,     ONLP_LED_MODE_OFF,              FAN_LED_OFF },
    { LED_FAN,     ONLP_LED_MODE_GREEN,            FAN_LED_GREEN_SOLID },
    { LED_FAN,     ONLP_LED_MODE_ORANGE,           FAN_LED_AMBER_SOLID },
    { LED_FAN,     ONLP_LED_MODE_ON,               FAN_LED_GREEN_SOLID },   

    { LED_LOC,     ONLP_LED_MODE_OFF,              LOC_LED_OFF },
    { LED_LOC,     ONLP_LED_MODE_BLUE_BLINKING,    LOC_LED_BLUE_BLINKING},
    { LED_LOC,     ONLP_LED_MODE_ON,               LOC_LED_BLUE_BLINKING },   
};

typedef union
{
    unsigned char val;
    struct
    {
        unsigned power :2;
        unsigned char :6;  /* reserved */
    }bit;

}_CPLD_POWER_LED_REG_T;

typedef union
{
    unsigned char val;
    struct
    {
        unsigned char psu0 :2;
        unsigned char psu1 :2;
        unsigned char :4;  /* reserved */
    }bit;

}_CPLD_PSU_LED_REG_T;

typedef union
{
    unsigned char val;
    struct
    {
        unsigned char system :2;
        unsigned char fan :2;
        unsigned char locator :1;
        unsigned char :3;  /* reserved */
    }bit;

}_PCA9539_NUM2_SYS_FAN_LED_REG_T;


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
        { ONLP_LED_ID_CREATE(LED_PSU), "Chassis LED 2 (PSU LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_ORANGE_BLINKING,
    },
    {
        { ONLP_LED_ID_CREATE(LED_SYSTEM), "Chassis LED 3 (SYSTEM LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_GREEN_BLINKING | ONLP_LED_CAPS_ORANGE,
    },
    {
        { ONLP_LED_ID_CREATE(LED_FAN), "Chassis LED 4 (FAN LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_ORANGE,
    },
    {
        { ONLP_LED_ID_CREATE(LED_LOC), "Chassis LED 5 (LOC LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_BLUE_BLINKING,
    },
};

struct led_gpio_id
{
    uint8_t ledid;
    uint16_t gpioid_color1;
	uint16_t gpioid_color2;
};

struct led_gpio_id leds[] = {
    {  },  /* Not used */
    {  },  /* Not used */
    {  },  /* Not used */
    { LED_SYSTEM, 496, 497 },  /* 496:AMBER. 497:GREEN */
    { LED_FAN,    498, 499 },  /* 498:AMBER. 499:GREEN */
    { LED_LOC,    500 }   	   /* 500:BLUE */
};


int hw_version = 1; //0:Proto 1:Beta

int
device_hw_version_get()
{
	int ret = 0;
	uint8_t* ma = NULL;
	int size;
	onlp_onie_info_t rv;

	ret = onlp_sysi_onie_data_get(&ma, &size);
	if(ret < 0)
	{
		AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ret);
        return ret;
	}
	else
	{
    	uint8_t* onie_data = ma;

		#if 0 //debug
        int i = 0;
        for (i=0;
             i<256;
             i++)
        {
            if ( i%8 == 0)
            {
                AIM_LOG_INFO("\n",onie_data[i]);
            }
            AIM_LOG_INFO("0x%2X [%c]",onie_data[i],(onie_data[i]<=122 && onie_data[i] >=45)?onie_data[i]:' ');

        }
        AIM_LOG_INFO("\n",onie_data[i]);
		#endif

		if(onie_data)
		{
			if(onlp_onie_decode(&rv, onie_data, -1) < 0)
			{
				onlp_sysi_onie_data_free(onie_data);

				hw_version = 1;
    			system("mkdir -p /tmp/");
				system("echo setenv PROTO_TYPE 0 > /tmp/hw_type.soc");
				
				return -1;
			}
			
            onlp_sysi_onie_data_free(onie_data);

			if((strcmp( rv.serial_number, "082F200800001") == 0) || (strcmp( rv.serial_number, "082F200800002") == 0))
        	{
        		hw_version = 0;
				
    			system("mkdir -p /tmp/");
				system("echo setenv PROTO_TYPE 1 > /tmp/hw_type.soc");
        	}
        	else
        	{
				hw_version = 1;
				
    			system("mkdir -p /tmp/");
				system("echo setenv PROTO_TYPE 0 > /tmp/hw_type.soc");
			}
			return 0;
    	}			
	}

	return -1;
}

static int convert_hw_led_light_mode_to_onlp(enum onlp_led_id id, int hw_mode)
{
    int i, nsize = sizeof(led_id_mode_data) / sizeof(led_id_mode_data[0]);
	
	if(hw_version == 1)
	{
		switch (id)
    	{
			case LED_POWER:
			case LED_PSU:     
				break;
			
       		case LED_SYSTEM:
			case LED_FAN:
				hw_mode = hw_mode ^ 3 ;
		 		break;
			
			case LED_LOC:    
        		hw_mode = hw_mode ^ 1 ;
        		break;
            
		 	default :
       			AIM_LOG_INFO("%s:%d %d is a wrong ID\n", __FUNCTION__, __LINE__, id);
		 		return ONLP_STATUS_E_PARAM;
   		 }
	}
	
    for (i = 0; i < nsize; i++)
    {	
        if ((led_id_mode_data[i].led_id == id) &&
            (led_id_mode_data[i].hw_led_light_mode == hw_mode))
        {
            DIAG_PRINT("%s, id:%d, hw_mode:%d mode:%d", __FUNCTION__, id, hw_mode, led_id_mode_data[i].mode);
            return (int)led_id_mode_data[i].mode;
        }
    }

    return -1;
}

static int convert_onlp_led_light_mode_to_hw(enum onlp_led_id id, onlp_led_mode_t mode)
{
    int i, nsize = sizeof(led_id_mode_data) / sizeof(led_id_mode_data[0]);
	char hw_mode = 0 ;
	
    for (i = 0; i < nsize; i++)
    {
        if ((led_id_mode_data[i].led_id == id) &&
            (led_id_mode_data[i].mode == mode))
        {
            DIAG_PRINT("%s, id:%d, mode:%d hw_mode:%d", __FUNCTION__, id, mode, led_id_mode_data[i].hw_led_light_mode);
			
			if(hw_version == 0)
			{	
				return led_id_mode_data[i].hw_led_light_mode;	
			}
			else		
			{	
        		switch (id)
    			{
      		  		case LED_POWER:
      		  		case LED_PSU:   
						hw_mode = led_id_mode_data[i].hw_led_light_mode;
	         		   	break;
					
    	  		 	case LED_SYSTEM:
      			  	case LED_FAN:
						hw_mode = led_id_mode_data[i].hw_led_light_mode ^ 3 ;
         		   		break;
			
	        		case LED_LOC:    
    	        		hw_mode = led_id_mode_data[i].hw_led_light_mode ^ 1 ;
           			 	break;
            
        			default :
            			AIM_LOG_INFO("%s:%d %d is a wrong ID\n", __FUNCTION__, __LINE__, id);
	            		return ONLP_STATUS_E_PARAM;
  				}

				return hw_mode;
       		}
        }
    }
	
    return -1;
}

pthread_t system_led_blinking_thread;
int system_led_thread_is_running = 0;

static void *set_system_led_green_blinking(void *arg)
{
    int sys_amber = 0;
	int sys_green = 0;
	char system_led_off = 0;
	char system_led_green_solid = 0;

	if(hw_version == 0)
	{
		system_led_off = 0;
	  	system_led_green_solid = 1;
	}
	else
	{
		system_led_off = 1;
	  	system_led_green_solid = 0;
	}

    do 
	{
		
		/* Get System LED Amber value */
    	if (onlp_file_read_int(&sys_amber, LED_FORMAT, leds[LED_SYSTEM].gpioid_color1) < 0) 
    	{
        	AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[LED_SYSTEM].gpioid_color1);
			system_led_thread_is_running = 0;
       		return NULL;
    	}

		/* Get System LED Green value */
    	if (onlp_file_read_int(&sys_green, LED_FORMAT, leds[LED_SYSTEM].gpioid_color2) < 0) 
    	{
        	AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[LED_SYSTEM].gpioid_color2);
			system_led_thread_is_running = 0;
       		return NULL;
    	}
	
		if ( (sys_amber == system_led_off) && (sys_green == system_led_off) )
		{
			if (onlp_file_write_int(system_led_off, LED_FORMAT, leds[LED_SYSTEM].gpioid_color1) < 0) 
        		return NULL;
			
        	if (onlp_file_write_int(system_led_green_solid, LED_FORMAT, leds[LED_SYSTEM].gpioid_color2) < 0) 
    			return NULL;
		}
     	else
		{
			if (onlp_file_write_int(system_led_off, LED_FORMAT, leds[LED_SYSTEM].gpioid_color1) < 0) 
    			return NULL;
    		
        	if (onlp_file_write_int(system_led_off, LED_FORMAT, leds[LED_SYSTEM].gpioid_color2) < 0) 
    			return NULL;
		}	

      	usleep (500000); //change led status for each 500ms
    } while(1);
    
    //pthread_exit(NULL);
}

pthread_t locator_led_blinking_thread;
int locator_led_thread_is_running = 0;

static void *set_locator_led_blue_blinking(void *arg)
{
    int data = 0;
    _PCA9539_NUM2_SYS_FAN_LED_REG_T locator_led_reg;

    do {

		/* Get Locator LED value */
    	if (onlp_file_read_int(&data, LED_FORMAT, leds[LED_LOC].gpioid_color1) < 0) 
    	{
        	AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[LED_LOC].gpioid_color1);
			locator_led_thread_is_running = 0;
       		return NULL;
    	}

		locator_led_reg.bit.locator = (unsigned char)data;

      	if (locator_led_reg.bit.locator == 0)
	{
          	locator_led_reg.bit.locator = 1;
	}
      	else
	{
          	locator_led_reg.bit.locator = 0;
	}

		if (onlp_file_write_int(locator_led_reg.bit.locator, LED_FORMAT, leds[LED_LOC].gpioid_color1) < 0) 
		{
			AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[LED_LOC].gpioid_color1);
			locator_led_thread_is_running = 0;
			return NULL;
		}

      	usleep (500000); //change led status for each 500ms
    } while(1);
    
    //pthread_exit(NULL);
}

/*
 * This function will be called prior to any other onlp_ledi_* functions.
 */
int
onlp_ledi_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);

    int ret = 0;
	
	/* Get device hw version Proto or Beta */
	ret = device_hw_version_get();
	if (ret < 0)
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ret);
        return ret;
    }
	
	/* Set Locator LED to off */
	/* (Need to check hw version first because led behavior is different between proto and beta) */
    onlp_ledi_mode_set(ONLP_LED_ID_CREATE(LED_LOC), ONLP_LED_MODE_OFF);
	
    return ONLP_STATUS_OK;
}

int
onlp_ledi_info_get(onlp_oid_t id, onlp_led_info_t *info)
{
    DIAG_PRINT("%s, id=%d", __FUNCTION__, id);
    _CPLD_POWER_LED_REG_T power_led_reg;
    _CPLD_PSU_LED_REG_T psu_led_reg;
    _PCA9539_NUM2_SYS_FAN_LED_REG_T sys_fan_led_reg;
    char data = 0;
    int ret = 0, local_id = 0;
	int led_amber = 0;
	int led_green = 0;

    VALIDATE(id);

    /* Set the onlp_oid_hdr_t and capabilities */
    *info = linfo[ONLP_OID_ID_GET(id)];

    local_id = ONLP_OID_ID_GET(id);

    switch (local_id)
    {
        case LED_POWER:
            ret = bmc_i2c_read_byte(BMC_CPLD_I2C_BUS_ID, BMC_CPLD_I2C_ADDR, CPLD_POWER_LED_ADDRESS_OFFSET, &data);
            if (ret < 0)
            {
                AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ret);
                return ret;
            }
            power_led_reg.val = data;
            break;

        case LED_PSU:
            ret = bmc_i2c_read_byte(BMC_CPLD_I2C_BUS_ID, BMC_CPLD_I2C_ADDR, CPLD_PSU_LED_ADDRESS_OFFSET, &data);
            if (ret < 0)
            {
                AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ret);
                return ret;
            }
            psu_led_reg.val = data;
            break;         

        case LED_SYSTEM:
        case LED_FAN:
			if (onlp_file_read_int(&led_amber, LED_FORMAT, leds[local_id].gpioid_color1) < 0) 
    		{
        		AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[local_id].gpioid_color1);
       			return ONLP_STATUS_E_INTERNAL;
    		}

			if (onlp_file_read_int(&led_green, LED_FORMAT, leds[local_id].gpioid_color2) < 0) 
    		{
        		AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[local_id].gpioid_color2);
       			return ONLP_STATUS_E_INTERNAL;
    		}

			ret = led_amber | (led_green << 1);

			if(local_id == LED_SYSTEM)
				sys_fan_led_reg.bit.system = (unsigned char)ret;
			else
				sys_fan_led_reg.bit.fan = (unsigned char)ret;

			break;
			
        case LED_LOC:   
            if (onlp_file_read_int(&ret, LED_FORMAT, leds[local_id].gpioid_color1) < 0) 
    		{
        		AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[local_id].gpioid_color1);
       			return ONLP_STATUS_E_INTERNAL;
    		}

			sys_fan_led_reg.bit.locator = (unsigned char)ret;
            break;
            
        default :
            AIM_LOG_INFO("%s:%d %d is a wrong ID\n", __FUNCTION__, __LINE__, local_id);
            return ONLP_STATUS_E_PARAM;
    }

    /* Get LED status */
    switch (local_id)
    {
        case LED_POWER:
            info->mode = convert_hw_led_light_mode_to_onlp(local_id, power_led_reg.bit.power);
            break;
        case LED_PSU:
            info->mode = convert_hw_led_light_mode_to_onlp(local_id, psu_led_reg.bit.psu0);
            break;
        case LED_SYSTEM:
            info->mode = convert_hw_led_light_mode_to_onlp(local_id, sys_fan_led_reg.bit.system);
            break;
        case LED_FAN:
            info->mode = convert_hw_led_light_mode_to_onlp(local_id, sys_fan_led_reg.bit.fan);
            break;
        case LED_LOC:
            info->mode = convert_hw_led_light_mode_to_onlp(local_id, sys_fan_led_reg.bit.locator);
            break;
        default:
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
onlp_ledi_mode_set(onlp_oid_t id, onlp_led_mode_t mode)
{
    DIAG_PRINT("%s, id=%d, mode=%d", __FUNCTION__, id, mode);
    _CPLD_POWER_LED_REG_T power_led_reg;
    _CPLD_PSU_LED_REG_T psu_led_reg;
    _PCA9539_NUM2_SYS_FAN_LED_REG_T sys_fan_led_reg;
    char data = 0;
    int ret = 0, local_id = 0, hw_led_mode = 0;
	char loc_led_blue_blinking = 0, system_led_green_blinking = 0;
	int led_amber=0, led_green=0;

    VALIDATE(id);

    local_id = ONLP_OID_ID_GET(id);

    switch (local_id)
    {
        case LED_POWER:
            ret = bmc_i2c_read_byte(BMC_CPLD_I2C_BUS_ID, BMC_CPLD_I2C_ADDR, CPLD_POWER_LED_ADDRESS_OFFSET, &data);
            if (ret < 0)
            {
                AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ret);
                return ret;
            }
            power_led_reg.val = data;
            break;

        case LED_PSU:
            ret = bmc_i2c_read_byte(BMC_CPLD_I2C_BUS_ID, BMC_CPLD_I2C_ADDR, CPLD_PSU_LED_ADDRESS_OFFSET, &data);
            if (ret < 0)
            {
                AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ret);
                return ret;
            }
            psu_led_reg.val = data;
            break;

        case LED_SYSTEM:
        case LED_FAN:
        case LED_LOC:
            break;

        default :
            AIM_LOG_INFO("%s:%d %d is a wrong ID\n", __FUNCTION__, __LINE__, local_id);
            return ONLP_STATUS_E_PARAM;
    }

    hw_led_mode = convert_onlp_led_light_mode_to_hw(local_id, mode);
    if (hw_led_mode < 0)
        return ONLP_STATUS_E_PARAM;

    /* Set LED light mode */
    switch (local_id)
    {
        case LED_POWER:
            power_led_reg.bit.power = hw_led_mode;
            break;
        case LED_PSU:
            psu_led_reg.bit.psu0 = hw_led_mode;
            break;
        case LED_SYSTEM:
            sys_fan_led_reg.bit.system = hw_led_mode;
            break;
        case LED_FAN:
            sys_fan_led_reg.bit.fan = hw_led_mode;
            break;
        case LED_LOC:
            sys_fan_led_reg.bit.locator = hw_led_mode;
            break;
        default:
            AIM_LOG_INFO("%s:%d %d is a wrong ID\n", __FUNCTION__, __LINE__, local_id);
            return ONLP_STATUS_E_PARAM;
    }

    switch (local_id)
    {
        case LED_POWER:
            //printf("[debug]service_led_reg.val:0x%x\n", service_led_reg.val);
            ret = bmc_i2c_write_byte(BMC_CPLD_I2C_BUS_ID, BMC_CPLD_I2C_ADDR, CPLD_POWER_LED_ADDRESS_OFFSET, power_led_reg.val);
            if (ret < 0)
            {
                AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ret);
                return ret;
            }
            break;

        case LED_PSU:
            ret = bmc_i2c_write_byte(BMC_CPLD_I2C_BUS_ID, BMC_CPLD_I2C_ADDR, CPLD_PSU_LED_ADDRESS_OFFSET, psu_led_reg.val);
            if (ret < 0)
            {
                AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ret);
                return ret;
            }
            break;

        case LED_SYSTEM:
			if(hw_version == 0)
				system_led_green_blinking = SYSTEM_LED_GREEN_BLINKING;
			else
				system_led_green_blinking = SYSTEM_LED_GREEN_BLINKING ^ 3;	
			
            if (hw_led_mode == system_led_green_blinking && system_led_thread_is_running == 0)
            {
                pthread_create(&system_led_blinking_thread, NULL, &set_system_led_green_blinking, NULL);
                system_led_thread_is_running = 1;
            }
            else
            {
                if (system_led_thread_is_running)
                {
                    pthread_cancel(system_led_blinking_thread);
                    system_led_thread_is_running = 0;
                }

				led_amber = sys_fan_led_reg.bit.system & 1;
				led_green = (sys_fan_led_reg.bit.system & 2) >> 1;

				if (onlp_file_write_int(led_amber, LED_FORMAT, leds[local_id].gpioid_color1) < 0)
    			{
        			AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[local_id].gpioid_color1);
					return ONLP_STATUS_E_INTERNAL;
    			}

				if (onlp_file_write_int(led_green, LED_FORMAT, leds[local_id].gpioid_color2) < 0)
    			{
        			AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[local_id].gpioid_color2);
					return ONLP_STATUS_E_INTERNAL;
    			}
            }
            break;

        case LED_FAN:
			led_amber = sys_fan_led_reg.bit.fan & 1;
			led_green = (sys_fan_led_reg.bit.fan & 2) >> 1;
				
			if (onlp_file_write_int(led_amber, LED_FORMAT, leds[local_id].gpioid_color1) < 0)
    		{
        		AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[local_id].gpioid_color1);
				return ONLP_STATUS_E_INTERNAL;
    		}

			if (onlp_file_write_int(led_green, LED_FORMAT, leds[local_id].gpioid_color2) < 0)
    		{
        		AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[local_id].gpioid_color2);
				return ONLP_STATUS_E_INTERNAL;
    		}
            break;

        case LED_LOC:
			if(hw_version == 0)
				loc_led_blue_blinking = LOC_LED_BLUE_BLINKING;
			else
				loc_led_blue_blinking = LOC_LED_BLUE_BLINKING ^ 1;	

            if (hw_led_mode == loc_led_blue_blinking && locator_led_thread_is_running == 0)
            {
                pthread_create(&locator_led_blinking_thread, NULL, &set_locator_led_blue_blinking, NULL);
                locator_led_thread_is_running = 1;
            }
            else
            {
                if (locator_led_thread_is_running)
                {
                    pthread_cancel(locator_led_blinking_thread);
                    locator_led_thread_is_running = 0;
                }

				if (onlp_file_write_int(sys_fan_led_reg.bit.locator, LED_FORMAT, leds[local_id].gpioid_color1) < 0)
    			{
        			AIM_LOG_ERROR("Unable to read status from file "LED_FORMAT, leds[local_id].gpioid_color1);
					return ONLP_STATUS_E_INTERNAL;
    			}
            }
            break;

        default :
            AIM_LOG_INFO("%s:%d %d is a wrong ID\n", __FUNCTION__, __LINE__, local_id);
            return ONLP_STATUS_E_PARAM;
    }

    return ONLP_STATUS_OK;
}

/*
 * Generic LED ioctl interface.
 */
int
onlp_ledi_ioctl(onlp_oid_t id, va_list vargs)
{
    return ONLP_STATUS_E_UNSUPPORTED;
}

