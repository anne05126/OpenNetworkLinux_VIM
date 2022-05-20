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
#include <onlp/platformi/sfpi.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <onlplib/i2c.h>
#include <onlplib/file.h>
#include <onlplib/gpio.h>
#include <x86_64_alphanetworks_spx70d0_168f/x86_64_alphanetworks_spx70d0_168f_gpio_table.h>
#include "platform_lib.h"

/**
 * This table maps the presence gpio, tx_disable gpio, tx_fault gpio, lp_mode gpio, and reset gpio
 * for each SFP port.
 */
typedef struct sfpmap_s {
    int port;
    int present_gpio;
    int tx_dis_gpio;
    int tx_fault_gpio;
    int rx_los_gpio;
	int mode_sel_gpio;
    int lp_mode_gpio;
    int reset_gpio;
} sfpmap_t;

static sfpmap_t sfpmap__[] =
    {
        {  0, SPX70D0_168F_PCA9539_GPIO_QSFP28_1_PRSNT_N, 0, 0, 0, 0, SPX70D0_168F_PCA9539_GPIO_QSFP28_2_LPMODE_N, SPX70D0_168F_PCA9539_GPIO_QSFP28_1_RESET_N },
        {  1, SPX70D0_168F_PCA9539_GPIO_QSFP28_2_PRSNT_N, 0, 0, 0, 0, SPX70D0_168F_PCA9539_GPIO_QSFP28_2_LPMODE_N, SPX70D0_168F_PCA9539_GPIO_QSFP28_2_RESET_N },
        {  2, SPX70D0_168F_PCA9539_GPIO_SFP28_1_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_1_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFP28_1_TX_FAULT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_1_RX_LOS_N, 0, 0, 0 },
        {  3, SPX70D0_168F_PCA9539_GPIO_SFP28_2_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_2_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFP28_2_TX_FAULT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_2_RX_LOS_N, 0, 0, 0 },
        {  4, SPX70D0_168F_PCA9539_GPIO_SFP28_3_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_3_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFP28_3_TX_FAULT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_3_RX_LOS_N, 0, 0, 0 },
        {  5, SPX70D0_168F_PCA9539_GPIO_SFP28_4_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_4_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFP28_4_TX_FAULT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_4_RX_LOS_N, 0, 0, 0 },
        {  6, SPX70D0_168F_PCA9539_GPIO_SFP28_5_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_5_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFP28_5_TX_FAULT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_5_RX_LOS_N, 0, 0, 0 },
        {  7, SPX70D0_168F_PCA9539_GPIO_SFP28_6_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_6_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFP28_6_TX_FAULT_N, SPX70D0_168F_PCA9539_GPIO_SFP28_6_RX_LOS_N, 0, 0, 0 },
        {  8, SPX70D0_168F_PCA9539_GPIO_SFPP_1_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_1_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_1_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_1_MODE_SEL_N, 0, 0 },
        {  9, SPX70D0_168F_PCA9539_GPIO_SFPP_2_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_2_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_2_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_2_MODE_SEL_N, 0, 0 },
        {  10, SPX70D0_168F_PCA9539_GPIO_SFPP_3_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_3_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_3_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_3_MODE_SEL_N, 0, 0 },
        {  11, SPX70D0_168F_PCA9539_GPIO_SFPP_4_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_4_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_4_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_4_MODE_SEL_N, 0, 0 },
        {  12, SPX70D0_168F_PCA9539_GPIO_SFPP_5_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_5_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_5_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_5_MODE_SEL_N, 0, 0 },
        {  13, SPX70D0_168F_PCA9539_GPIO_SFPP_6_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_6_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_6_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_6_MODE_SEL_N, 0, 0 },
        {  14, SPX70D0_168F_PCA9539_GPIO_SFPP_7_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_7_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_7_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_7_MODE_SEL_N, 0, 0 },
        {  15, SPX70D0_168F_PCA9539_GPIO_SFPP_8_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_8_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_8_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_8_MODE_SEL_N, 0, 0 },
        {  16, SPX70D0_168F_PCA9539_GPIO_SFPP_9_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_9_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_9_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_9_MODE_SEL_N, 0, 0 },
        {  17, SPX70D0_168F_PCA9539_GPIO_SFPP_10_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_10_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_10_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_10_MODE_SEL_N, 0, 0 },
        {  18, SPX70D0_168F_PCA9539_GPIO_SFPP_11_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_11_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_11_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_11_MODE_SEL_N, 0, 0 },
        {  19, SPX70D0_168F_PCA9539_GPIO_SFPP_12_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_12_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_12_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_12_MODE_SEL_N, 0, 0 },
        {  20, SPX70D0_168F_PCA9539_GPIO_SFPP_13_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_13_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_13_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_13_MODE_SEL_N, 0, 0 },
        {  21, SPX70D0_168F_PCA9539_GPIO_SFPP_14_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_14_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_14_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_14_MODE_SEL_N, 0, 0 },
        {  22, SPX70D0_168F_PCA9539_GPIO_SFPP_15_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_15_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_15_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_15_MODE_SEL_N, 0, 0 },
        {  23, SPX70D0_168F_PCA9539_GPIO_SFPP_16_PRSNT_N, SPX70D0_168F_PCA9539_GPIO_SFPP_16_TX_DIS_N, SPX70D0_168F_PCA9539_GPIO_SFPP_16_TX_FAULT_N, 0, SPX70D0_168F_PCA9539_GPIO_SFPP_16_MODE_SEL_N, 0, 0 },
  };

#define SFP_GET(_port) (sfpmap__ + _port)

#define PORT_BUS_INDEX(port) (port_to_dev_busid(port))

/* SFP */
#define MODULE_EEPROM_SFP_FORMAT            "/sys/bus/i2c/devices/%d-0050/eeprom"
#define MODULE_EEPROM_DOM_SFP_FORMAT        "/sys/bus/i2c/devices/%d-0051/eeprom"

/* bus id base */
#define PON_PORT_BUSID_BASE					1
#define NNI_PORT_BUSID_BASE					25

static int
port_to_dev_busid(int port)
{
    int busid = 0;

	if(!(port > SFP_PORT_10G_SFPP_16))
	{
		if(port >= SFP_PORT_10G_SFPP_1)
			busid = port + PON_PORT_BUSID_BASE; /* port8(busid=9), port9(busid=10)... */
		else
			busid = port + NNI_PORT_BUSID_BASE; /* port0(busid=25), port1(busid=26)... */
	}
	
    return busid;
}

/************************************************************
 *
 * SFPI Entry Points
 *
 ***********************************************************/
int
onlp_sfpi_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);

    return ONLP_STATUS_OK;
}

int
onlp_sfpi_bitmap_get(onlp_sfp_bitmap_t *bmap)
{
    /*
     * Ports {0, 23}
     */
    int p = 0;
    AIM_BITMAP_CLR_ALL(bmap);

    for (p = 0; p < NUM_OF_SFP_PORT; p++)
    {
        AIM_BITMAP_SET(bmap, p);
    }
    DIAG_PRINT("%s", __FUNCTION__);

    return ONLP_STATUS_OK;
}

int
onlp_sfpi_is_present(int port)
{
    /*
     * Return 1 if present.
     * Return 0 if not present.
     * Return < 0 if error.
     */

	int value = 0;
	
	if (IS_SFP_PORT(port) || IS_SFP28_PORT(port) || IS_QSFP_PORT(port))
	{
    	sfpmap_t* sfp = SFP_GET(port);
    	if(sfp->present_gpio > 0) 
    	{
        	DIAG_PRINT("%s, port %d, sfp->present_gpio %d\r\n", __FUNCTION__, port, sfp->present_gpio);

        	if(onlp_gpio_get(sfp->present_gpio, &value) == ONLP_STATUS_OK)
            	return (value == 0);
        	else
           		return ONLP_STATUS_E_MISSING;		
    	}
    	else
    	{
        	return ONLP_STATUS_E_INVALID;
    	}
	}
	else
    {
        return ONLP_STATUS_E_INVALID;
    }
}

int
onlp_sfpi_presence_bitmap_get(onlp_sfp_bitmap_t *dst)
{
    DIAG_PRINT("%s", __FUNCTION__);

	/* auto generate from sfp.c */
    return ONLP_STATUS_E_UNSUPPORTED;
}

int
onlp_sfpi_rx_los_bitmap_get(onlp_sfp_bitmap_t* dst)
{
	/*
     * Ports {0, 23}
     */
    int p = 0;
	int supported;

    AIM_BITMAP_CLR_ALL(dst);

    for (p = 0; p < NUM_OF_SFP_PORT; p++)
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
    if(port < 0 || port >= NUM_OF_SFP_PORT)
        return ONLP_STATUS_E_INTERNAL;

    DIAG_PRINT("%s, port:%d, busid:%d", __FUNCTION__, port, PORT_BUS_INDEX(port));

    if (IS_SFP_PORT(port) || IS_SFP28_PORT(port) || IS_QSFP_PORT(port)) 
    {
        if(onlp_file_read(data, 256, &size, MODULE_EEPROM_SFP_FORMAT, 
                PORT_BUS_INDEX(port)) != ONLP_STATUS_OK) 
        {
            AIM_LOG_ERROR("Unable to read eeprom from port(%d)\r\n", port);
            return ONLP_STATUS_E_INTERNAL;
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

int onlp_sfpi_dom_read(int port, uint8_t data[256])
{
	FILE* fp;
    char file[64] = {0};

    DIAG_PRINT("%s, port:%d, busid:%d", __FUNCTION__, port, PORT_BUS_INDEX(port));

    if (IS_SFP_PORT(port) || IS_SFP28_PORT(port))
    {
        sprintf(file, MODULE_EEPROM_SFP_FORMAT, PORT_BUS_INDEX(port));
        fp = fopen(file, "r");
        if(fp == NULL) {
            AIM_LOG_ERROR("Unable to open the eeprom device file of port(%d)", port);
            return ONLP_STATUS_E_INTERNAL;
        }

        if (fseek(fp, 256, SEEK_CUR) != 0) {
            fclose(fp);
            AIM_LOG_ERROR("Unable to set the file position indicator of port(%d)", port);
            return ONLP_STATUS_E_INTERNAL;
        }

        int ret = fread(data, 1, 256, fp);
        fclose(fp);
        if (ret != 256) {
            AIM_LOG_ERROR("Unable to read the module_eeprom device file of port(%d)", port);
            return ONLP_STATUS_E_INTERNAL;
        }
    }
    else
    {
        return ONLP_STATUS_E_INVALID;
    }

    return ONLP_STATUS_OK;
}

int
onlp_sfpi_dev_readb(int port, uint8_t devaddr, uint8_t addr)
{
    int ret = 0;
    int bus = port_to_dev_busid(port);

    ret = onlp_i2c_readb(bus, devaddr, addr, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, ret:%d(0x%02X)", __FUNCTION__, port, devaddr, addr, ret, ret);

    return ret;
}

int
onlp_sfpi_dev_writeb(int port, uint8_t devaddr, uint8_t addr, uint8_t value)
{
    int ret = 0;
    int bus = port_to_dev_busid(port);

    ret = onlp_i2c_writeb(bus, devaddr, addr, value, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, value:%d(0x%02X), ret:%d", __FUNCTION__, port, devaddr, addr, value, value, ret);

    return ret;
}

int
onlp_sfpi_dev_readw(int port, uint8_t devaddr, uint8_t addr)
{
    int ret = 0;
    int bus = port_to_dev_busid(port);

    ret = onlp_i2c_readw(bus, devaddr, addr, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, ret:%d(0x%04X)", __FUNCTION__, port, devaddr, addr, ret, ret);

    return ret;
}

int
onlp_sfpi_dev_writew(int port, uint8_t devaddr, uint8_t addr, uint16_t value)
{
    int ret = 0;
    int bus = port_to_dev_busid(port);

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
            if(IS_QSFP_PORT(port))
            {
                *supported = 1;
            }
            else
            {
                *supported = 0;
            }
            break;            
        }

        case ONLP_SFP_CONTROL_TX_FAULT:
        case ONLP_SFP_CONTROL_TX_DISABLE:
        {
            if(IS_SFP_PORT(port) || IS_SFP28_PORT(port))
            {
                *supported = 1;
            }
            else
            {
                *supported = 0;
            }
            break;            
        }

		case ONLP_SFP_CONTROL_RX_LOS:
		{
			if(IS_SFP28_PORT(port))
            {
                *supported = 1;
            }
            else
            {
                *supported = 0;
            }
            break; 
		}

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
    switch (control)
    {
        case ONLP_SFP_CONTROL_TX_DISABLE:
        {
            if(IS_SFP_PORT(port) || IS_SFP28_PORT(port)) 
            {
            	sfpmap_t* sfp = SFP_GET(port);
                if(onlp_gpio_set(sfp->tx_dis_gpio, value) == ONLP_STATUS_OK)
                    rv = ONLP_STATUS_OK;
                else
                {
                    AIM_LOG_ERROR("Unable to set tx_disable status to port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }            
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
            	sfpmap_t* sfp = SFP_GET(port);
                if(onlp_gpio_set(sfp->lp_mode_gpio, value) == ONLP_STATUS_OK)
                    rv = ONLP_STATUS_OK;
                else
                {
                    AIM_LOG_ERROR("Unable to set lp_mode status to port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }            
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }

        case ONLP_SFP_CONTROL_RESET:
        {
            if(IS_QSFP_PORT(port)) 
            {
            	sfpmap_t* sfp = SFP_GET(port);
                if(onlp_gpio_set(sfp->reset_gpio, value) == ONLP_STATUS_OK)
                    rv = ONLP_STATUS_OK;
                else
                {
                    AIM_LOG_ERROR("Unable to set reset_gpio status to port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }            
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
onlp_sfpi_control_get(int port, onlp_sfp_control_t control, int *value)
{
	int rv;
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

    switch (control)
    {
    	case ONLP_SFP_CONTROL_RX_LOS:
		{
            if(IS_SFP28_PORT(port))
            {
            	sfpmap_t* sfp = SFP_GET(port);
                if(onlp_gpio_get(sfp->tx_dis_gpio, value) == ONLP_STATUS_OK)
				{                    
                    rv = ONLP_STATUS_OK;
                }
                else
				{
                    AIM_LOG_ERROR("Unable to read rx_los status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
				
				DIAG_PRINT("%s, Read Current SFP port:%d RXLOS GPIO value:0x%x, ", __FUNCTION__, port, *value);
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }
		
        case ONLP_SFP_CONTROL_TX_FAULT:
        {
            if(IS_SFP_PORT(port) || IS_SFP28_PORT(port))
            {
            	sfpmap_t* sfp = SFP_GET(port);
                if(onlp_gpio_get(sfp->tx_fault_gpio, value) == ONLP_STATUS_OK)
				{
                    rv = ONLP_STATUS_OK;
                }
                else
				{
                    AIM_LOG_ERROR("Unable to read tx_fault status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }    

				DIAG_PRINT("%s, Read Current SFP port:%d TXFAULT GPIO value:0x%x, ", __FUNCTION__, port, *value);
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }
            
        case ONLP_SFP_CONTROL_TX_DISABLE:
        {
            if(IS_SFP_PORT(port) || IS_SFP28_PORT(port))
            {
            	sfpmap_t* sfp = SFP_GET(port);
                if(onlp_gpio_get(sfp->tx_dis_gpio, value) == ONLP_STATUS_OK)
				{                    
                    rv = ONLP_STATUS_OK;
                }
                else
				{
                    AIM_LOG_ERROR("Unable to read tx_disable status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }

				DIAG_PRINT("%s, Read Current SFP port:%d TXDISABLE GPIO value:0x%x, ", __FUNCTION__, port, *value);
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
            	sfpmap_t* sfp = SFP_GET(port);
                if(onlp_gpio_get(sfp->lp_mode_gpio, value) == ONLP_STATUS_OK)
				{
                    rv = ONLP_STATUS_OK;
                }
                else
				{
                    AIM_LOG_ERROR("Unable to read lp_mode status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }     

				DIAG_PRINT("%s, Read Current SFP port:%d LPMODE GPIO value:0x%x, ", __FUNCTION__, port, *value);
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }

        case ONLP_SFP_CONTROL_RESET:
        {
            if(IS_QSFP_PORT(port))
            {
            	sfpmap_t* sfp = SFP_GET(port);
                if(onlp_gpio_get(sfp->reset_gpio, value) == ONLP_STATUS_OK)
				{
                    rv = ONLP_STATUS_OK;
                }
                else
				{
                    AIM_LOG_ERROR("Unable to read reset status from port(%d)\r\n", port);
                    rv = ONLP_STATUS_E_INTERNAL;
                }
				
				DIAG_PRINT("%s, Read Current SFP port:%d CONTRORESET GPIO value:0x%x, ", __FUNCTION__, port, *value);
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

