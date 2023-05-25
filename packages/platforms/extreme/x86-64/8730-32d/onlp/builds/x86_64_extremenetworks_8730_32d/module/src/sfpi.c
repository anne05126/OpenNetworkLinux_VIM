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
 *
 *
 ***********************************************************/
#include <onlp/platformi/sfpi.h>
#include <onlplib/i2c.h>
#include <onlplib/file.h>
#include "platform_lib.h"

#define CPLD_MAX_PORT_NUM       16

static const int port_cpld_bus_index[NUM_OF_SFP_PORT] = 
{
    4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4,
    7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7
};

static const int port_bus_index[NUM_OF_SFP_PORT] = 
{
     9, 10, 11, 12, 13, 14, 15, 16,
    17, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30, 31, 32,
    33, 34, 35, 36, 37, 38, 39, 40
};

static const int qsfp_port_cpld_bus_index[NUM_OF_QSFP_PORT_CPLD] = 
{
    4, 7
};

#define PORT_CPLD_BUS_INDEX(port)           (port_cpld_bus_index[port])
#define PORT_BUS_INDEX(port)                (port_bus_index[port])
#define QSFP_PORT_CPLD_BUS_INDEX(port_cpld) (qsfp_port_cpld_bus_index[port_cpld])

/* QSFP */
#define MODULE_EEPROM_QSFP_FORMAT           "/sys/bus/i2c/devices/%d-0050/eeprom"
#define MODULE_PRESENT_QSFP_CPLD_FORMAT     "/sys/bus/i2c/devices/%d-0057/module_present_%d"
#define MODULE_RESET_QSFP_CPLD_FORMAT       "/sys/bus/i2c/devices/%d-0057/module_reset_%d"
#define MODULE_LPMODE_QSFP_CPLD_FORMAT      "/sys/bus/i2c/devices/%d-0057/module_lp_mode_%d"
#define MODULE_PRESENT_ALL_QSFP_CPLD_FORMAT "/sys/bus/i2c/devices/%d-0057/module_present_all"

/* IOBM */
#define MODULE_PRESENT_IOBM_SFP_FORMAT     	"/sys/bus/platform/devices/8730_iobm_io_eeprom/sfp%d_present"
#define MODULE_TXFAULT_IOBM_SFP_FORMAT     	"/sys/bus/platform/devices/8730_iobm_io_eeprom/sfp%d_txfault"
#define MODULE_RXLOS_IOBM_SFP_FORMAT     	"/sys/bus/platform/devices/8730_iobm_io_eeprom/sfp%d_rxlos"
#define MODULE_TXDIS_IOBM_SFP_FORMAT     	"/sys/bus/platform/devices/8730_iobm_io_eeprom/sfp%d_txdis"
#define MODULE_EEPROM_IOBM_SFP_FORMAT     	"/sys/bus/platform/devices/8730_iobm_io_eeprom/sfp%d_eeprom"
#define MODULE_DOM_IOBM_SFP_FORMAT     		"/sys/bus/platform/devices/8730_iobm_io_eeprom/sfp%d_dom"

#define MODULE_PRESENT_IOBM_QSFP28_FORMAT   "/sys/bus/platform/devices/8730_iobm_io_eeprom/qsfp%d_mod_present"
#define MODULE_RESET_IOBM_QSFP28_FORMAT   	"/sys/bus/platform/devices/8730_iobm_io_eeprom/qsfp%d_rst_mod"
#define MODULE_LPMODE_IOBM_QSFP28_FORMAT   	"/sys/bus/platform/devices/8730_iobm_io_eeprom/qsfp%d_lp_mode"
#define MODULE_EEPROM_IOBM_QSFP28_FORMAT   	"/sys/bus/platform/devices/8730_iobm_io_eeprom/qsfp%d_eeprom"
#define MODULE_DOM_IOBM_QSFP28_FORMAT   	"/sys/bus/platform/devices/8730_iobm_io_eeprom/qsfp%d_dom"


/************************************************************
 *
 * SFPI Entry Points
 *
 ***********************************************************/
int
onlp_sfpi_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);
    /* Called at initialization time */
    return ONLP_STATUS_OK;
}

int
onlp_sfpi_bitmap_get(onlp_sfp_bitmap_t* bmap)
{
	/*
     * Front QSFPDD Ports {0, 31}
     * IOBM QSFP28 Port {32}
     * IOBM SFP+ Ports {33, 34}
     */
    int p;
    AIM_BITMAP_CLR_ALL(bmap);

    for(p = 0; p < (NUM_OF_SFP_PORT + NUM_OF_IOBM_PORT); p++) {
        AIM_BITMAP_SET(bmap, p);
    }

    DIAG_PRINT("%s", __FUNCTION__);
    
    return ONLP_STATUS_OK;
}

int onlp_sfpi_is_present(int port)
{
    /*
     * Return 1 if present.
     * Return 0 if not present.
     * Return < 0 if error.
     */
    int present;

	if (IS_QSFP_PORT(port))
    {
        if (onlp_file_read_int(&present, MODULE_PRESENT_QSFP_CPLD_FORMAT, 
                PORT_CPLD_BUS_INDEX(port), (port+1)) < 0) 
        {
            AIM_LOG_ERROR("Unable to read present status from port(%d)\r\n", port);
            return ONLP_STATUS_E_INTERNAL;
        }
        DIAG_PRINT("%s, QSFP Port:%d Present value:0x%x, ", __FUNCTION__, port, present);
    }
	else if (IS_IOBM_PORT(port))
    {
    	if(port == IOBM_QSFP28_PORT_INDEX)
    	{
    		if (onlp_file_read_int(&present, MODULE_PRESENT_IOBM_QSFP28_FORMAT, 
                	(port - QSFP_PORT_INDEX_END)) < 0) 
        	{
        		char file[64] = {0};
        		sprintf(file, MODULE_PRESENT_IOBM_QSFP28_FORMAT, (port - QSFP_PORT_INDEX_END)); 

            	AIM_LOG_ERROR("Unable to read present status from port(%d). path: %s\r\n", port, file);
            	return ONLP_STATUS_E_INTERNAL;
        	}
    	}
		else
		{
        	if (onlp_file_read_int(&present, MODULE_PRESENT_IOBM_SFP_FORMAT, 
                	(port - IOBM_QSFP28_PORT_INDEX)) < 0) 
        	{
        		char file[64] = {0};
        		sprintf(file, MODULE_PRESENT_IOBM_QSFP28_FORMAT, (port - QSFP_PORT_INDEX_END)); 
				
            	AIM_LOG_ERROR("Unable to read present status from port(%d)\r\n", port);
            	return ONLP_STATUS_E_INTERNAL;
        	}
		}
        DIAG_PRINT("%s, IOBM Port:%d Present value:0x%x, ", __FUNCTION__, port, present);
    }
    else
    {
        return ONLP_STATUS_E_INVALID;
    }
    
    return present;
}

int
onlp_sfpi_presence_bitmap_get(onlp_sfp_bitmap_t* dst)
{
	/*
     * Front QSFPDD Ports {0, 31}
     * IOBM QSFP28 Port {32}
     * IOBM SFP+ Ports {33, 34}
     */
    DIAG_PRINT("%s", __FUNCTION__);
    uint32_t bytes[(NUM_OF_QSFP_PORT_CPLD*NUM_OF_QSFP_PER_PORT_CPLD)] = { 0 };
    int i = 0;
	int j = 0;
    FILE* fp;

    /* Get 4 QSFP PORT CPLD present_all status */
    for (i = 0; i < NUM_OF_QSFP_PORT_CPLD; i++)
    {
        int count = 0;
        char file[64] = {0};

        sprintf(file, MODULE_PRESENT_ALL_QSFP_CPLD_FORMAT, QSFP_PORT_CPLD_BUS_INDEX(i));      
        fp = fopen(file, "r");

		DIAG_PRINT("%s, QSFP PRESENT ALL path: %s", __FUNCTION__, file);

        if(fp == NULL) 
        {
            AIM_LOG_ERROR("Unable to open the module_present_all device file from port_cpld(%d)\r\n", i+1);
            return ONLP_STATUS_E_INTERNAL;
        }
        count = fscanf(fp, "%x %x", bytes+i+j, bytes+i+(j+1));
        fclose(fp);
        
        if(count != 2) 
        {
            /* Likely a CPLD read timeout. */
            AIM_LOG_ERROR("Unable to read module_present_all status from port_cpld(%d)\r\n", i+1);
            return ONLP_STATUS_E_INTERNAL;
        }

        DIAG_PRINT("%s, port_cpld:%d, present_all:0x%x 0x%x", __FUNCTION__, i+1, bytes[i+j], bytes[i+(j+1)]);

		j++;
    }

    for (i = 0; i < (NUM_OF_QSFP_PORT_CPLD*NUM_OF_QSFP_PER_PORT_CPLD); i++)
    {
        DIAG_PRINT("%s, bytes[%d]:0x%x", __FUNCTION__, i, bytes[i]);
    }

    /* Convert to 64 bit integer in port order */
    uint64_t presence_all = 0 ;
    for (i = AIM_ARRAYSIZE(bytes) - 1; i >= 0; i--)
    {
        presence_all <<= 8;
        presence_all |= bytes[i];
    }

    /* Populate bitmap */
    for (i = 0; presence_all; i++)
    {
        AIM_BITMAP_MOD(dst, i, (presence_all & 1));
        presence_all >>= 1;
    }

	/* Get IOBM present bitmap */
	for (i = NUM_OF_SFP_PORT; i <= IOBM_SFP_PORT_INDEX_END; i++)
    {
        if (onlp_sfpi_is_present(i))
        {
            AIM_BITMAP_SET(dst, i);
        }
    }

    return ONLP_STATUS_OK;
}

int
onlp_sfpi_rx_los_bitmap_get(onlp_sfp_bitmap_t* dst)
{
	/*
     * Front QSFPDD Ports {0, 31}
     * IOBM QSFP28 Port {32}
     * IOBM SFP+ Ports {33, 34}
     */
    int p = 0;
	int supported;

    AIM_BITMAP_CLR_ALL(dst);

    for (p = 0; p < (NUM_OF_SFP_PORT + NUM_OF_IOBM_PORT); p++)
    {
		if(onlp_sfpi_is_present(p) == 0) 
		{
        	continue;
    	}

    	supported = 0;
    	if( (onlp_sfpi_control_supported(p, ONLP_SFP_CONTROL_RX_LOS, &supported) >= 0) &&
        	!supported) {
        	continue;
    	}

        AIM_BITMAP_SET(dst, p);
    }
    DIAG_PRINT("%s", __FUNCTION__);

    return ONLP_STATUS_OK;
}

int
onlp_sfpi_eeprom_read(int port, uint8_t data[256])
{
    /*
     * Read the SFP eeprom into data[]
     *
     * Return MISSING if SFP is missing.
     * Return OK if eeprom is read
     */
    int size = 0;
    if(port < 0 || port >= (NUM_OF_SFP_PORT + NUM_OF_IOBM_PORT))
        return ONLP_STATUS_E_INTERNAL;

    if (IS_QSFP_PORT(port))
    {
    	DIAG_PRINT("%s, port:%d, busid:%d", __FUNCTION__, port, PORT_BUS_INDEX(port));
		
        if(onlp_file_read(data, 256, &size, MODULE_EEPROM_QSFP_FORMAT, 
                PORT_BUS_INDEX(port)) != ONLP_STATUS_OK) 
        {
            AIM_LOG_ERROR("Unable to read eeprom from port(%d)\r\n", port);
            return ONLP_STATUS_E_INTERNAL;
        }
    }
	else if (IS_IOBM_PORT(port))
    {
    	DIAG_PRINT("%s, port:%d", __FUNCTION__, port);
		
    	if(port == IOBM_QSFP28_PORT_INDEX)
    	{
    		if(onlp_file_read(data, 256, &size, MODULE_EEPROM_IOBM_QSFP28_FORMAT, 
                	(port - QSFP_PORT_INDEX_END)) != ONLP_STATUS_OK) 
        	{
            	AIM_LOG_ERROR("Unable to read eeprom from port(%d)\r\n", port);
            	return ONLP_STATUS_E_INTERNAL;
        	}
    	}
		else
		{
        	if (onlp_file_read(data, 256, &size, MODULE_EEPROM_IOBM_SFP_FORMAT, 
                	(port - IOBM_QSFP28_PORT_INDEX)) < 0) 
        	{
            	AIM_LOG_ERROR("Unable to read eeprom from port(%d)\r\n", port);
            	return ONLP_STATUS_E_INTERNAL;
        	}
		}
    }
    else
    {
        return ONLP_STATUS_E_INVALID;
    }

    if(size != 256) 
    {
        return ONLP_STATUS_E_INTERNAL;
    }

    return ONLP_STATUS_OK;
}

int
onlp_sfpi_dom_read(int port, uint8_t data[256])
{
	/*
     * Read the IOBM dom(A2h) into data[]
     *
     * Return OK if dom is read
     */     
    int size = 0;
    if(port < 0 || port >= (NUM_OF_SFP_PORT + NUM_OF_IOBM_PORT))
        return ONLP_STATUS_E_INTERNAL;

    if (IS_QSFP_PORT(port))
    {
    	DIAG_PRINT("%s, port:%d, busid:%d", __FUNCTION__, port, PORT_BUS_INDEX(port));
		
        return ONLP_STATUS_E_UNSUPPORTED;
    }
	else if (IS_IOBM_PORT(port))
    {
    	DIAG_PRINT("%s, port:%d", __FUNCTION__, port);
		
    	if(port == IOBM_QSFP28_PORT_INDEX)
    	{
        	if (onlp_file_read(data, 256, &size, MODULE_DOM_IOBM_QSFP28_FORMAT, 
                	(port - QSFP_PORT_INDEX_END)) < 0) 
        	{
            	return ONLP_STATUS_E_UNSUPPORTED;
        	}
    	}
		else
		{
        	if (onlp_file_read(data, 256, &size, MODULE_DOM_IOBM_SFP_FORMAT, 
                	(port - IOBM_QSFP28_PORT_INDEX)) < 0) 
        	{
            	return ONLP_STATUS_E_UNSUPPORTED;
        	}
		}
    }
    else
    {
        return ONLP_STATUS_E_INVALID;
    }

    if(size != 256) 
    {
        return ONLP_STATUS_E_INTERNAL;
    }

    return ONLP_STATUS_OK;
}  

int
onlp_sfpi_dev_readb(int port, uint8_t devaddr, uint8_t addr)
{
    int ret = 0;
    int bus = PORT_BUS_INDEX(port);

    ret = onlp_i2c_readb(bus, devaddr, addr, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, ret:%d(0x%02X)", __FUNCTION__, port, devaddr, addr, ret, ret);
    return ret; 
}

int
onlp_sfpi_dev_writeb(int port, uint8_t devaddr, uint8_t addr, uint8_t value)
{
    int ret = 0;
    int bus = PORT_BUS_INDEX(port);

    ret = onlp_i2c_writeb(bus, devaddr, addr, value, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, value:%d(0x%02X), ret:%d", __FUNCTION__, port, devaddr, addr, value, value, ret);
    return ret;
}

int
onlp_sfpi_dev_readw(int port, uint8_t devaddr, uint8_t addr)
{
    int ret = 0;
    int bus = PORT_BUS_INDEX(port);

    ret = onlp_i2c_readw(bus, devaddr, addr, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, ret:%d(0x%04X)", __FUNCTION__, port, devaddr, addr, ret, ret);
    return ret;
}

int
onlp_sfpi_dev_writew(int port, uint8_t devaddr, uint8_t addr, uint16_t value)
{
    int ret = 0;
    int bus = PORT_BUS_INDEX(port);

    ret = onlp_i2c_writew(bus, devaddr, addr, value, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, value:%d(0x%04X), ret:%d", __FUNCTION__, port, devaddr, addr, value, value, ret);
    return ret;
}

int onlp_sfpi_control_supported(int port, onlp_sfp_control_t control, int *supported)
{
    if (supported == NULL)
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_PARAM);
        return ONLP_STATUS_E_PARAM;
    }

    *supported = 0;
    switch (control)
    {
        case ONLP_SFP_CONTROL_RESET:
        case ONLP_SFP_CONTROL_LP_MODE:
        {
            if(IS_QSFP_PORT(port) || IS_IOBM_QSFP28_PORT(port))
            {
                *supported = 1;
            }
            else
            {
                *supported = 0;
            }
        }
            break;
        case ONLP_SFP_CONTROL_RX_LOS:
        case ONLP_SFP_CONTROL_TX_FAULT:
        case ONLP_SFP_CONTROL_TX_DISABLE:
        {
            if(IS_IOBM_SFP_PORT(port))
            {
                *supported = 1;
            }
            else
            {
                *supported = 0;
            }
        }
            break;

        default:
            *supported = 0;
            break;
    }

    DIAG_PRINT("%s, port:%d, control:%d(%s), supported:%d", __FUNCTION__, port, control, sfp_control_to_str(control), *supported);
    return ONLP_STATUS_OK;
}


int
onlp_sfpi_control_set(int port, onlp_sfp_control_t control, int value)
{
    int rv;  
    int supported = 0;

    if ((onlp_sfpi_control_supported(port, control, &supported) == ONLP_STATUS_OK) && 
        (supported == 0))
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
        return ONLP_STATUS_E_UNSUPPORTED;
    }
    
    DIAG_PRINT("%s, port:%d, control:%d(%s), value:0x%X", __FUNCTION__, port, control, sfp_control_to_str(control), value);
    
    /* ONLP_SFP_CONTROL_RESET: write-only. */
    switch(control)
    {
        case ONLP_SFP_CONTROL_RESET:
        {
            if(IS_QSFP_PORT(port)) 
            {
                if (onlp_file_write_int(value, MODULE_RESET_QSFP_CPLD_FORMAT, 
                        PORT_CPLD_BUS_INDEX(port), (port+1)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to set reset status to port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else 
                {
                    rv = ONLP_STATUS_OK;
                }
                DIAG_PRINT("%s, Write QSFP port:%d RESET value:0x%x, ", __FUNCTION__, port, value);
            }
			else if(IS_IOBM_QSFP28_PORT(port))
			{
				if(onlp_file_write_int(value, MODULE_RESET_IOBM_QSFP28_FORMAT, 
                		(port - QSFP_PORT_INDEX_END)) < 0) 
        		{
            		AIM_LOG_ERROR("Unable to set reset status to port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
        		}
				else 
                {
                    rv = ONLP_STATUS_OK;
                }
                DIAG_PRINT("%s, Write IOBM QSFP28 port:%d RESET value:0x%x, ", __FUNCTION__, port, value);
			}
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }
        case ONLP_SFP_CONTROL_LP_MODE:
        {
            if(IS_QSFP_PORT(port)) 
            {
                if (onlp_file_write_int(value, MODULE_LPMODE_QSFP_CPLD_FORMAT, 
                        PORT_CPLD_BUS_INDEX(port), (port+1)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to set lp_mode status to port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else 
                {
                    rv = ONLP_STATUS_OK;
                }
                DIAG_PRINT("%s, Write QSFP port:%d LPMODE value:0x%x, ", __FUNCTION__, port, value);
            }
			else if(IS_IOBM_QSFP28_PORT(port)) 
            {
                if (onlp_file_write_int(value, MODULE_LPMODE_IOBM_QSFP28_FORMAT, 
                        (port - QSFP_PORT_INDEX_END)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to set lp_mode status to port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else 
                {
                    rv = ONLP_STATUS_OK;
                }
                DIAG_PRINT("%s, Write IOBM QSFP28 port:%d LPMODE value:0x%x, ", __FUNCTION__, port, value);
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }
        case ONLP_SFP_CONTROL_TX_DISABLE:
        {			
            if(IS_IOBM_SFP_PORT(port)) 
            {
                if (onlp_file_write_int(value, MODULE_TXDIS_IOBM_SFP_FORMAT, 
                        (port - IOBM_QSFP28_PORT_INDEX)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to set tx_disable status to port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else 
                {
                    rv = ONLP_STATUS_OK;
                }
                DIAG_PRINT("%s, Write IOBM SFP port:%d TXDIS value:0x%x, ", __FUNCTION__, port, value);
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }

        default:
            rv = ONLP_STATUS_E_UNSUPPORTED;
            break;
    }

    if (rv == ONLP_STATUS_E_UNSUPPORTED)
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
    }

    return rv;
}

int
onlp_sfpi_control_get(int port, onlp_sfp_control_t control, int* value)
{
    int rv;
    int val;
    int supported = 0;

    if (value == NULL)
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_PARAM);
        return ONLP_STATUS_E_PARAM;
    }

    if ((onlp_sfpi_control_supported(port, control, &supported) == ONLP_STATUS_OK) && 
        (supported == 0))
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
        return ONLP_STATUS_E_UNSUPPORTED;
    }
    *value = 0;
    
    /* ONLP_SFP_CONTROL_RX_LOS , ONLP_SFP_CONTROL_TX_FAULT are read-only. */
    switch(control)
    {
        case ONLP_SFP_CONTROL_RESET:
        {
            if(IS_QSFP_PORT(port))
            {
                if (onlp_file_read_int(&val,  MODULE_RESET_QSFP_CPLD_FORMAT, 
                        PORT_CPLD_BUS_INDEX(port), (port+1)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to read reset status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else
                {
                    rv = ONLP_STATUS_OK;
                }

                DIAG_PRINT("%s, Read Current CPLD QSFP RESET value:0x%x, ", __FUNCTION__, val);
                *value = val;
            }
			else if(IS_IOBM_QSFP28_PORT(port))
            {
                if (onlp_file_read_int(&val,  MODULE_RESET_IOBM_QSFP28_FORMAT, 
                        (port - QSFP_PORT_INDEX_END)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to read reset status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else
                {
                    rv = ONLP_STATUS_OK;
                }

                DIAG_PRINT("%s, Read Current IOBM QSFP28 RESET value:0x%x, ", __FUNCTION__, val);
                *value = val;
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }
        case ONLP_SFP_CONTROL_LP_MODE:
        {
            if(IS_QSFP_PORT(port))
            {
                if (onlp_file_read_int(&val,  MODULE_LPMODE_QSFP_CPLD_FORMAT, 
                        PORT_CPLD_BUS_INDEX(port), (port+1)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to read lp_mode status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else
                {
                    rv = ONLP_STATUS_OK;
                }

                DIAG_PRINT("%s, Read Current CPLD QSFP LPMODE value:0x%x, ", __FUNCTION__, val);
                *value = val;
            }
			else if(IS_IOBM_QSFP28_PORT(port))
            {
                if (onlp_file_read_int(&val,  MODULE_LPMODE_IOBM_QSFP28_FORMAT, 
                        (port - QSFP_PORT_INDEX_END)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to read lp_mode status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else
                {
                    rv = ONLP_STATUS_OK;
                }

                DIAG_PRINT("%s, Read Current IOBM QSFP28 LPMODE value:0x%x, ", __FUNCTION__, val);
                *value = val;
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }
        case ONLP_SFP_CONTROL_RX_LOS: 
		{
			if(IS_IOBM_SFP_PORT(port))
            {
                if (onlp_file_read_int(&val,  MODULE_RXLOS_IOBM_SFP_FORMAT, 
                        (port - IOBM_QSFP28_PORT_INDEX)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to read rx_los status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else
                {
                    rv = ONLP_STATUS_OK;
                }

                DIAG_PRINT("%s, Read Current IOBM SFP RXLOS value:0x%x, ", __FUNCTION__, val);
                *value = val;
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
			break;
        }
        case ONLP_SFP_CONTROL_TX_FAULT:
		{
			if(IS_IOBM_SFP_PORT(port))
            {
                if (onlp_file_read_int(&val,  MODULE_TXFAULT_IOBM_SFP_FORMAT, 
                        (port - IOBM_QSFP28_PORT_INDEX)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to read tx_fault status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else
                {
                    rv = ONLP_STATUS_OK;
                }

                DIAG_PRINT("%s, Read Current IOBM SFP TXFAULT value:0x%x, ", __FUNCTION__, val);
                *value = val;
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
			break;
        }
        case ONLP_SFP_CONTROL_TX_DISABLE:
        {
            if(IS_IOBM_SFP_PORT(port))
            {
                if (onlp_file_read_int(&val,  MODULE_TXDIS_IOBM_SFP_FORMAT, 
                        (port - IOBM_QSFP28_PORT_INDEX)) < 0) 
                {
                    AIM_LOG_ERROR("Unable to read tx_disable status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
                else
                {
                    rv = ONLP_STATUS_OK;
                }

                DIAG_PRINT("%s, Read Current IOBM SFP TXDIS value:0x%x, ", __FUNCTION__, val);
                *value = val;
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }

        default:
            rv = ONLP_STATUS_E_UNSUPPORTED;
    }

    DIAG_PRINT("%s, port:%d, control:%d(%s), value:0x%X", __FUNCTION__, port, control, sfp_control_to_str(control), *value);

    if (rv == ONLP_STATUS_E_UNSUPPORTED)
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
    }

    return rv;
}

int
onlp_sfpi_denit(void)
{
    DIAG_PRINT("%s", __FUNCTION__);
    return ONLP_STATUS_OK;
}

