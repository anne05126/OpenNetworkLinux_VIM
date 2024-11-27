/*
 * An I2C multiplexer dirver for extremenetworks 7830-32ce-8de VIM CPLD
 *
 * Copyright (C) 2023 Alphanetworks Technology Corporation.
 * Ruby Wang <Ru-xin_Wang@alphanetworks.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/version.h>
#include <linux/stat.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/umh.h>

#define DRIVER_NAME                             "7830_i2c_vim_port_cpld"
#define I2C_RW_RETRY_COUNT                      10
#define I2C_RW_RETRY_INTERVAL                   60 /* ms */

#define DEBUG_MODE                              0

/* VIM */
#define VIM_CPLD1_ADDRESS                       0x5C
#define VIM_CPLD2_ADDRESS                       0x58
#define VIM_CPLD_REG_ADDR_BOARD_ID         	    0x01
#define VIM_CPLD_REG_ADDR_REVISION              0x00

#define PCA9548_8_ADDRESS                       0x77
#define PCA9548_1_ADDRESS                       0x76
#define CHANNEL_0_ADDRESS  				        0x01	/* CH0 */
#define CHANNEL_1_ADDRESS  				        0x02	/* CH1 */
#define CHANNEL_3_ADDRESS  				        0x08	/* CH3 */
#define CHANNEL_4_ADDRESS  				        0x10	/* CH4 */

/* VIM CPLD2 0x58 */
#define VIM_SFP_PRESENT_P13_P16                 0x11 
#define VIM_SFP_PRESENT_P17_P24                 0x13 
#define VIM_SFP_RST_MOD_P13_P16                 0x16 
#define VIM_SFP_RST_MOD_P17_P24                 0x17 
#define VIM_SFP_LP_MODE_P13_P16                 0x19 
#define VIM_SFP_LP_MODE_P17_P24                 0x1A 
#define VIM_SFP_TXFAULT_P13_P16                 0x1C
#define VIM_SFP_TXFAULT_P17_P24                 0x1D
#define VIM_SFP_TXFAULT_B_P13_P16               0x1E
#define VIM_SFP_TXFAULT_B_P17_P24               0x1F
#define VIM_SFP_TXDIS_P13_P16                   0x21
#define VIM_SFP_TXDIS_P17_P24                   0x22
#define VIM_SFP_TXDIS_B_P13_P16                 0x23
#define VIM_SFP_TXDIS_B_P17_P24                 0x24
#define VIM_SFP_RXLOS_P13_P16                   0x26
#define VIM_SFP_RXLOS_P17_P24                   0x27
#define VIM_SFP_RXLOS_B_P13_P16                 0x29
#define VIM_SFP_RXLOS_B_P17_P24                 0x2A

#define NUM_OF_SFP_QSFP_ATTR                    24
#define NOT_SUPPORT                             0xFF
#define VIM_CPLD_VERSION_BITS_MASK    			0x0F

static unsigned int debug = DEBUG_MODE;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");


#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)

static LIST_HEAD(cpld_client_list);
static struct mutex     list_lock;

struct cpld_client_node {
    struct i2c_client *client;
    struct list_head   list;
};

enum vim_id
{
    VIM_1, 
    VIM_2,
    VIM_ID_MAX
};

enum vim_type_id
{
    VIM_8DE, 
    VIM_16CE,
    VIM_24CE,
    VIM_24YE
};

enum cpld_type {
    extreme7830_32ce_8de_vim1_cpld2,
    extreme7830_32ce_8de_vim2_cpld2
};

struct extreme7830_32ce_8de_vim_port_cpld_data {
    enum cpld_type type;
    struct device      *hwmon_dev;
    struct mutex        update_lock;
    enum  vim_type_id board_id;
};

struct chip_desc {
    u8   nchans;
    u8   deselectChan;
};

static const struct i2c_device_id extreme7830_32ce_8de_vim_port_cpld_id[] = {
    { "VIM1_CPLD1", extreme7830_32ce_8de_vim1_cpld2 },
    { "VIM2_CPLD1", extreme7830_32ce_8de_vim2_cpld2 },
    {}
};
MODULE_DEVICE_TABLE(i2c, extreme7830_32ce_8de_vim_port_cpld_id);


#define VIM_CPLD_VERSION_ATTR_ID(index)				VIM_##index##_CPLD_VERSION
#define VIM_BOARD_ID_ATTR_ID(index)					VIM_##index##_BOARD_ID
#define PRESENT_ATTR_ID(index)				        PRESENT_##index
#define RST_MOD_ATTR_ID(index)				        RST_MOD_##index
#define LP_MODE_ATTR_ID(index)				        LP_MODE_##index
#define MOD_SEL_ATTR_ID(index)				        MOD_SEL_##index
#define TXFAULT_ATTR_ID(index)				        TXFAULT_##index
#define TXDIS_ATTR_ID(index)				        TXDIS_##index
#define RXLOS_ATTR_ID(index)				        RXLOS_##index
#define TXFAULT_B_ATTR_ID(index)				    TXFAULT_B_##index
#define TXDIS_B_ATTR_ID(index)				        TXDIS_B_##index
#define RXLOS_B_ATTR_ID(index)				        RXLOS_B_##index

enum extreme7830_32ce_8de_sys_cpld_sysfs_attributes {
    VIM_CPLD_VERSION_ATTR_ID(1), 
    VIM_CPLD_VERSION_ATTR_ID(2),
    VIM_BOARD_ID_ATTR_ID(1), 
    VIM_BOARD_ID_ATTR_ID(2),

    PRESENT_ATTR_ID(13), 
    PRESENT_ATTR_ID(14),
    PRESENT_ATTR_ID(15),
    PRESENT_ATTR_ID(16),
    PRESENT_ATTR_ID(17),
    PRESENT_ATTR_ID(18),
    PRESENT_ATTR_ID(19), 
    PRESENT_ATTR_ID(20),
    PRESENT_ATTR_ID(21),
    PRESENT_ATTR_ID(22),
    PRESENT_ATTR_ID(23),
    PRESENT_ATTR_ID(24),

    RST_MOD_ATTR_ID(13), 
    RST_MOD_ATTR_ID(14),
    RST_MOD_ATTR_ID(15),
    RST_MOD_ATTR_ID(16),
    RST_MOD_ATTR_ID(17),
    RST_MOD_ATTR_ID(18),
    RST_MOD_ATTR_ID(19), 
    RST_MOD_ATTR_ID(20),
    RST_MOD_ATTR_ID(21),
    RST_MOD_ATTR_ID(22),
    RST_MOD_ATTR_ID(23),
    RST_MOD_ATTR_ID(24),

    LP_MODE_ATTR_ID(13), 
    LP_MODE_ATTR_ID(14),
    LP_MODE_ATTR_ID(15),
    LP_MODE_ATTR_ID(16),
    LP_MODE_ATTR_ID(17),
    LP_MODE_ATTR_ID(18),
    LP_MODE_ATTR_ID(19), 
    LP_MODE_ATTR_ID(20),
    LP_MODE_ATTR_ID(21),
    LP_MODE_ATTR_ID(22),
    LP_MODE_ATTR_ID(23),
    LP_MODE_ATTR_ID(24),

    MOD_SEL_ATTR_ID(13), 
    MOD_SEL_ATTR_ID(14),
    MOD_SEL_ATTR_ID(15),
    MOD_SEL_ATTR_ID(16),
    MOD_SEL_ATTR_ID(17),
    MOD_SEL_ATTR_ID(18),
    MOD_SEL_ATTR_ID(19), 
    MOD_SEL_ATTR_ID(20),
    MOD_SEL_ATTR_ID(21),
    MOD_SEL_ATTR_ID(22),
    MOD_SEL_ATTR_ID(23),
    MOD_SEL_ATTR_ID(24),

    TXFAULT_ATTR_ID(13), 
    TXFAULT_ATTR_ID(14),
    TXFAULT_ATTR_ID(15),
    TXFAULT_ATTR_ID(16),
    TXFAULT_ATTR_ID(17),
    TXFAULT_ATTR_ID(18),
    TXFAULT_ATTR_ID(19), 
    TXFAULT_ATTR_ID(20),
    TXFAULT_ATTR_ID(21),
    TXFAULT_ATTR_ID(22),
    TXFAULT_ATTR_ID(23),
    TXFAULT_ATTR_ID(24),

    TXFAULT_B_ATTR_ID(13), 
    TXFAULT_B_ATTR_ID(14),
    TXFAULT_B_ATTR_ID(15),
    TXFAULT_B_ATTR_ID(16),
    TXFAULT_B_ATTR_ID(17),
    TXFAULT_B_ATTR_ID(18),
    TXFAULT_B_ATTR_ID(19), 
    TXFAULT_B_ATTR_ID(20),
    TXFAULT_B_ATTR_ID(21),
    TXFAULT_B_ATTR_ID(22),
    TXFAULT_B_ATTR_ID(23),
    TXFAULT_B_ATTR_ID(24),

    TXDIS_ATTR_ID(13), 
    TXDIS_ATTR_ID(14),
    TXDIS_ATTR_ID(15),
    TXDIS_ATTR_ID(16),
    TXDIS_ATTR_ID(17),
    TXDIS_ATTR_ID(18),
    TXDIS_ATTR_ID(19), 
    TXDIS_ATTR_ID(20),
    TXDIS_ATTR_ID(21),
    TXDIS_ATTR_ID(22),
    TXDIS_ATTR_ID(23),
    TXDIS_ATTR_ID(24),

    TXDIS_B_ATTR_ID(13), 
    TXDIS_B_ATTR_ID(14),
    TXDIS_B_ATTR_ID(15),
    TXDIS_B_ATTR_ID(16),
    TXDIS_B_ATTR_ID(17),
    TXDIS_B_ATTR_ID(18),
    TXDIS_B_ATTR_ID(19), 
    TXDIS_B_ATTR_ID(20),
    TXDIS_B_ATTR_ID(21),
    TXDIS_B_ATTR_ID(22),
    TXDIS_B_ATTR_ID(23),
    TXDIS_B_ATTR_ID(24),

    RXLOS_ATTR_ID(13), 
    RXLOS_ATTR_ID(14),
    RXLOS_ATTR_ID(15),
    RXLOS_ATTR_ID(16),
    RXLOS_ATTR_ID(17),
    RXLOS_ATTR_ID(18),
    RXLOS_ATTR_ID(19), 
    RXLOS_ATTR_ID(20),
    RXLOS_ATTR_ID(21),
    RXLOS_ATTR_ID(22),
    RXLOS_ATTR_ID(23),
    RXLOS_ATTR_ID(24),

    RXLOS_B_ATTR_ID(13), 
    RXLOS_B_ATTR_ID(14),
    RXLOS_B_ATTR_ID(15),
    RXLOS_B_ATTR_ID(16),
    RXLOS_B_ATTR_ID(17),
    RXLOS_B_ATTR_ID(18),
    RXLOS_B_ATTR_ID(19), 
    RXLOS_B_ATTR_ID(20),
    RXLOS_B_ATTR_ID(21),
    RXLOS_B_ATTR_ID(22),
    RXLOS_B_ATTR_ID(23),
    RXLOS_B_ATTR_ID(24),

};


/* sysfs attributes for hwmon */
static ssize_t show_vim_sfp_status(struct device *dev, struct device_attribute *da,
                           char *buf);
static ssize_t set_vim_sfp_status(struct device *dev, struct device_attribute *da, 
                      const char *buf, size_t count);
static ssize_t show_vim_board_id(struct device *dev, struct device_attribute *da,
                           char *buf);
static ssize_t set_vim_board_id(struct device *dev, struct device_attribute *da, 
                      const char *buf, size_t count);
static ssize_t show_vim_cpld_version(struct device *dev, struct device_attribute *da,
                            char *buf);
static int extreme7830_32ce_8de_vim_cpld_read_internal(struct i2c_client *client, u8 reg);
static int extreme7830_32ce_8de_vim_cpld_write_internal(struct i2c_client *client, u8 reg, u8 value);

/* VIM SFP/QSFP attributes */
#define DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(index) \
	static SENSOR_DEVICE_ATTR(present_##index, S_IRUGO, show_vim_sfp_status, NULL, \
                  PRESENT_##index); \
    static SENSOR_DEVICE_ATTR(rst_mod_##index, S_IWUSR | S_IRUGO, show_vim_sfp_status, set_vim_sfp_status, \
				  RST_MOD_##index); \
    static SENSOR_DEVICE_ATTR(lp_mode_##index, S_IWUSR | S_IRUGO, show_vim_sfp_status, set_vim_sfp_status, \
                  LP_MODE_##index); \
    static SENSOR_DEVICE_ATTR(mod_sel_##index, S_IWUSR | S_IRUGO, show_vim_sfp_status, NULL, \
				  MOD_SEL_##index); \
    static SENSOR_DEVICE_ATTR(tx_fault_##index, S_IRUGO, show_vim_sfp_status, NULL, \
                  TXFAULT_##index); \
    static SENSOR_DEVICE_ATTR(tx_dis_##index, S_IWUSR | S_IRUGO, show_vim_sfp_status, set_vim_sfp_status, \
				  TXDIS_##index); \
    static SENSOR_DEVICE_ATTR(rx_los_##index, S_IRUGO, show_vim_sfp_status, NULL, \
				  RXLOS_##index); \
    static SENSOR_DEVICE_ATTR(tx_fault_b_##index, S_IRUGO, show_vim_sfp_status, NULL, \
                  TXFAULT_##index); \
    static SENSOR_DEVICE_ATTR(tx_dis_b_##index, S_IWUSR | S_IRUGO, show_vim_sfp_status, set_vim_sfp_status, \
				  TXDIS_##index); \
    static SENSOR_DEVICE_ATTR(rx_los_b_##index, S_IRUGO, show_vim_sfp_status, NULL, \
				  RXLOS_##index);

#define DECLARE_VIM_SFP_ATTR(index) \
	&sensor_dev_attr_present_##index.dev_attr.attr, \
	&sensor_dev_attr_rst_mod_##index.dev_attr.attr, \
	&sensor_dev_attr_lp_mode_##index.dev_attr.attr, \
	&sensor_dev_attr_mod_sel_##index.dev_attr.attr, \
	&sensor_dev_attr_tx_fault_##index.dev_attr.attr, \
	&sensor_dev_attr_tx_dis_##index.dev_attr.attr, \
	&sensor_dev_attr_rx_los_##index.dev_attr.attr, \
    &sensor_dev_attr_tx_fault_b_##index.dev_attr.attr, \
	&sensor_dev_attr_tx_dis_b_##index.dev_attr.attr, \
	&sensor_dev_attr_rx_los_b_##index.dev_attr.attr

/* VIM_BOARD_ID attributes */
#define DECLARE_VIM_BOARD_ID_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(vim_##index##_board_id, S_IWUSR | S_IRUGO, show_vim_board_id, set_vim_board_id, \
                  VIM_##index##_BOARD_ID);
#define DECLARE_VIM_BOARD_ID_ATTR(index)  &sensor_dev_attr_vim_##index##_board_id.dev_attr.attr

/* VIM_CPLD_VERSION attributes */
#define DECLARE_VIM_CPLD_VERSION_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(vim##index##_cpld_version, S_IRUGO, show_vim_cpld_version, NULL, \
                  VIM_##index##_CPLD_VERSION);
#define DECLARE_VIM_CPLD_VERSION_ATTR(index)  &sensor_dev_attr_vim##index##_cpld_version.dev_attr.attr


DECLARE_VIM_CPLD_VERSION_SENSOR_DEVICE_ATTR(1);
DECLARE_VIM_CPLD_VERSION_SENSOR_DEVICE_ATTR(2);
DECLARE_VIM_BOARD_ID_SENSOR_DEVICE_ATTR(1);
DECLARE_VIM_BOARD_ID_SENSOR_DEVICE_ATTR(2);

DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(13);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(14);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(15);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(16);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(17);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(18);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(19);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(20);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(21);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(22);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(23);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(24);


/* VIM#1 CPLD2 */
static struct attribute *extreme7830_32ce_8de_vim1_cpld2_attributes[] = {
    DECLARE_VIM_CPLD_VERSION_ATTR(1),
    DECLARE_VIM_BOARD_ID_ATTR(1), 

    /* transceiver attributes */
    DECLARE_VIM_SFP_ATTR(13),
    DECLARE_VIM_SFP_ATTR(14),
    DECLARE_VIM_SFP_ATTR(15),
    DECLARE_VIM_SFP_ATTR(16),
    DECLARE_VIM_SFP_ATTR(17),
    DECLARE_VIM_SFP_ATTR(18),
    DECLARE_VIM_SFP_ATTR(19),
    DECLARE_VIM_SFP_ATTR(20),
    DECLARE_VIM_SFP_ATTR(21),
    DECLARE_VIM_SFP_ATTR(22),
    DECLARE_VIM_SFP_ATTR(23),
    DECLARE_VIM_SFP_ATTR(24), 

    NULL
};
static const struct attribute_group extreme7830_32ce_8de_vim1_cpld2_group = {
    .attrs = extreme7830_32ce_8de_vim1_cpld2_attributes,
};


/* VIM#2 CPLD2 */
static struct attribute *extreme7830_32ce_8de_vim2_cpld2_attributes[] = {
    DECLARE_VIM_CPLD_VERSION_ATTR(2),
    DECLARE_VIM_BOARD_ID_ATTR(2), 

    /* transceiver attributes */
    DECLARE_VIM_SFP_ATTR(13),
    DECLARE_VIM_SFP_ATTR(14),
    DECLARE_VIM_SFP_ATTR(15),
    DECLARE_VIM_SFP_ATTR(16),
    DECLARE_VIM_SFP_ATTR(17),
    DECLARE_VIM_SFP_ATTR(18),
    DECLARE_VIM_SFP_ATTR(19),
    DECLARE_VIM_SFP_ATTR(20),
    DECLARE_VIM_SFP_ATTR(21),
    DECLARE_VIM_SFP_ATTR(22),
    DECLARE_VIM_SFP_ATTR(23),
    DECLARE_VIM_SFP_ATTR(24), 

    NULL
};
static const struct attribute_group extreme7830_32ce_8de_vim2_cpld2_group = {
    .attrs = extreme7830_32ce_8de_vim2_cpld2_attributes,
};

/*
 * Attributes function
 *      - set_vim_sfp_status    : Set VIM SFP attribute
 *      - show_vim_sfp_status   : Set VIM SFP attribute
 *      - set_vim_board_id      : 
 *      - show_vim_board_id     : 
 */
static ssize_t set_vim_sfp_status(struct device *dev, struct device_attribute *da, 
                      const char *buf, size_t count)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_port_cpld_data *data = i2c_get_clientdata(client);

    long on;
    int status= -ENOENT;
    u8 reg = 0, mask = 0;
    int board_id = 0, err;    

    status = kstrtol(buf, 10, &on);
    if (status) {
        return status;
    }

	if ((on != 1) && (on != 0))
        return -EINVAL;

    /* Get VIM board ID */
    mutex_lock(&data->update_lock);
    board_id = data->board_id;
    mutex_unlock(&data->update_lock);

    /* Show vim sfp status */
    switch (board_id) {
        case VIM_8DE:
        case VIM_16CE:
            reg  = NOT_SUPPORT;
            break;
        case VIM_24CE:
            switch (attr->index) {
                /* reset */
                case RST_MOD_13 ... RST_MOD_16:
                    reg  = VIM_SFP_RST_MOD_P13_P16;
                    mask = 0x1 << (attr->index - RST_MOD_13) << 0x4;     /* Shift 4 bits, because RST_MOD_13 is start at bit 4 */
                    break;
                case RST_MOD_17 ... RST_MOD_24:
                    reg  = VIM_SFP_RST_MOD_P17_P24;
                    mask = 0x1 << (attr->index - RST_MOD_17);
                    break;

                /* LP_MODE */
                case LP_MODE_13 ... LP_MODE_16:
                    reg  = VIM_SFP_LP_MODE_P13_P16;
                    mask = 0x1 << (attr->index - LP_MODE_13) << 0x4;     /* Shift 4 bits, because LP_MODE_13 is start at bit 4 */
                    break;
                case LP_MODE_17 ... LP_MODE_24:
                    reg  = VIM_SFP_LP_MODE_P17_P24;
                    mask = 0x1 << (attr->index - LP_MODE_17);
                    break;

                /* TX_DIS */
                case TXDIS_13 ... TXDIS_16:
                    reg  = VIM_SFP_TXDIS_P13_P16;
                    mask = 0x1 << (attr->index - TXDIS_13) << 0x4;      /* Shift 4 bits, because TXDIS_13 is start at bit 4 */
                    break;
                case TXDIS_17 ... TXDIS_24:
                    reg  = VIM_SFP_TXDIS_P17_P24;
                    mask = 0x1 << (attr->index - TXDIS_17);
                    break;
                
                /* TX_DIS_B */
                case TXDIS_B_13 ... TXDIS_B_16:
                    reg  = VIM_SFP_TXDIS_B_P13_P16;
                    mask = 0x1 << (attr->index - TXDIS_B_13) << 0x4;      /* Shift 4 bits, because TXDIS_B_13 is start at bit 4 */
                    break;
                case TXDIS_B_17 ... TXDIS_B_24:
                    reg  = VIM_SFP_TXDIS_B_P17_P24;
                    mask = 0x1 << (attr->index - TXDIS_B_17);
                    break;

                default:
                    reg  = NOT_SUPPORT;
                    break;
            }
            break;
        case VIM_24YE:
            switch (attr->index) {
                /* TX_DIS */
                case TXDIS_13 ... TXDIS_16:
                    reg  = VIM_SFP_TXDIS_P13_P16;
                    mask = 0x1 << (attr->index - TXDIS_13) << 0x4;       /* Shift 4 bits, because TXDIS_13 is start at bit 4 */
                    break;
                case TXDIS_17 ... TXDIS_24:
                    reg  = VIM_SFP_TXDIS_P17_P24;
                    mask = 0x1 << (attr->index - TXDIS_17);
                    break;
                default:
                    reg  = NOT_SUPPORT;
                    break;
            }
            break;
        default:
            break;
    }

    if (reg != NOT_SUPPORT)
    {
        mutex_lock(&data->update_lock);
        /* Set PCA9548#1(0x76) CH3 to CH4 */
        status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_1_ADDRESS, CHANNEL_4_ADDRESS);
        if (unlikely(status < 0)) {
            DEBUG_PRINT("Switch channel failed! board_id = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_1_ADDRESS, CHANNEL_4_ADDRESS);
            goto exit;
        }

        status = extreme7830_32ce_8de_vim_cpld_read_internal(client, reg);
        if (unlikely(status < 0)) {
            DEBUG_PRINT("[set_vim_sfp_status] extreme7830_32ce_8de_vim_cpld_read_internal failed: attr->index:%d, reg=%d", attr->index, reg);
            goto exit;
        }

        if (on) {
            /* bit 0 --> 1 */
            status |= mask;
        }
        else {
            /* bit 1 --> 0 */
            status &= ~mask;
        }
    
        err = extreme7830_32ce_8de_vim_cpld_write_internal(client, reg, status);
        if (unlikely(err < 0)) {
            DEBUG_PRINT("[set_vim_sfp_status] extreme7830_32ce_8de_vim_cpld_write_internal failed: attr->index:%d, reg=%d, value=%d", attr->index, reg, status);
            goto exit;
        }

        mutex_unlock(&data->update_lock);
        return count;

        
    }
    else
    {
        return sprintf(buf, "%s\n", "NOT SUPPORT");
    }
    return count;

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t show_vim_sfp_status(struct device *dev, struct device_attribute *da,
                           char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_port_cpld_data *data = i2c_get_clientdata(client);
    int status = 0;
    u8 reg = 0, mask = 0;
    int board_id = 0;    

    /* Get VIM board ID */
    mutex_lock(&data->update_lock);
    board_id = data->board_id;
    mutex_unlock(&data->update_lock);


    /* Show vim sfp status */
    switch (board_id) {
        case VIM_8DE:
        case VIM_16CE:
            reg  = NOT_SUPPORT;
            break;

        case VIM_24CE:
            switch (attr->index) {
                /* present */
                case PRESENT_13 ... PRESENT_16:
                    reg  = VIM_SFP_PRESENT_P13_P16;
                    mask = 0x1 << (attr->index - PRESENT_13) << 0x4;    /* Shift 4 bits, because PRESENT_13 is start at bit 4 */
                    break;
                case PRESENT_17 ... PRESENT_24:
                    reg  = VIM_SFP_PRESENT_P17_P24;
                    mask = 0x1 << (attr->index - PRESENT_17);
                    break;
                
                /* reset */
                case RST_MOD_13 ... RST_MOD_16:
                    reg  = VIM_SFP_RST_MOD_P13_P16;
                    mask = 0x1 << (attr->index - RST_MOD_13) << 0x4;    /* Shift 4 bits, because RST_MOD_13 is start at bit 4 */
                    break;
                case RST_MOD_17 ... RST_MOD_24:
                    reg  = VIM_SFP_RST_MOD_P17_P24;
                    mask = 0x1 << (attr->index - RST_MOD_17);
                    break;
                
                /* LP_MODE */
                case LP_MODE_13 ... LP_MODE_16:
                    reg  = VIM_SFP_LP_MODE_P13_P16;
                    mask = 0x1 << (attr->index - LP_MODE_13) << 0x4;    /* Shift 4 bits, because LP_MODE_13 is start at bit 4 */
                    break;
                case LP_MODE_17 ... LP_MODE_24:
                    reg  = VIM_SFP_LP_MODE_P17_P24;
                    mask = 0x1 << (attr->index - LP_MODE_17);
                    break;

                /* TX_FAULT */
                case TXFAULT_13 ... TXFAULT_16:
                    reg  = VIM_SFP_TXFAULT_P13_P16;
                    mask = 0x1 << (attr->index - TXFAULT_13) << 0x4;    /* Shift 4 bits, because TXFAULT_13 is start at bit 4 */
                    break;
                case TXFAULT_17 ... TXFAULT_24:
                    reg  = VIM_SFP_TXFAULT_P17_P24;
                    mask = 0x1 << (attr->index - TXFAULT_17);
                    break;
                
                /* TX_FAULT_B */
                case TXFAULT_B_13 ... TXFAULT_B_16:
                    reg  = VIM_SFP_TXFAULT_B_P13_P16;
                    mask = 0x1 << (attr->index - TXFAULT_B_13) << 0x4;    /* Shift 4 bits, because TXFAULT_13 is start at bit 4 */
                    break;
                case TXFAULT_B_17 ... TXFAULT_B_24:
                    reg  = VIM_SFP_TXFAULT_B_P17_P24;
                    mask = 0x1 << (attr->index - TXFAULT_B_17);
                    break;

                /* TX_DIS */
                case TXDIS_13 ... TXDIS_16:
                    reg  = VIM_SFP_TXDIS_P13_P16;
                    mask = 0x1 << (attr->index - TXDIS_13) << 0x4;      /* Shift 4 bits, because TXDIS_13 is start at bit 4 */
                    break;
                case TXDIS_17 ... TXDIS_24:
                    reg  = VIM_SFP_TXDIS_P17_P24;
                    mask = 0x1 << (attr->index - TXDIS_17);
                    break;
                
                /* TX_DIS_B */
                case TXDIS_B_13 ... TXDIS_B_16:
                    reg  = VIM_SFP_TXDIS_B_P13_P16;
                    mask = 0x1 << (attr->index - TXDIS_B_13) << 0x4;      /* Shift 4 bits, because TXDIS_13 is start at bit 4 */
                    break;
                case TXDIS_B_17 ... TXDIS_B_24:
                    reg  = VIM_SFP_TXDIS_B_P17_P24;
                    mask = 0x1 << (attr->index - TXDIS_B_17);
                    break;

                /* RX_LOS */
                case RXLOS_13 ... RXLOS_16:
                    reg  = VIM_SFP_RXLOS_P13_P16;
                    mask = 0x1 << (attr->index - RXLOS_13) << 0x4;      /* Shift 4 bits, because RXLOS_13 is start at bit 4 */
                    break;
                case RXLOS_17 ... RXLOS_24:
                    reg  = VIM_SFP_RXLOS_P17_P24;
                    mask = 0x1 << (attr->index - RXLOS_17);
                    break;
                
                /* RX_LOS_B */
                case RXLOS_B_13 ... RXLOS_B_16:
                    reg  = VIM_SFP_RXLOS_B_P13_P16;
                    mask = 0x1 << (attr->index - RXLOS_B_13) << 0x4;      /* Shift 4 bits, because RXLOS_13 is start at bit 4 */
                    break;
                case RXLOS_B_17 ... RXLOS_B_24:
                    reg  = VIM_SFP_RXLOS_B_P17_P24;
                    mask = 0x1 << (attr->index - RXLOS_B_17);
                    break;
                
                default:
                    reg  = NOT_SUPPORT;
                    break;
            }
            break;
        case VIM_24YE:
            switch (attr->index) {
                /* present */
                case PRESENT_13 ... PRESENT_16:
                    reg  = VIM_SFP_PRESENT_P13_P16;
                    mask = 0x1 << (attr->index - PRESENT_13) << 0x4;    /* Shift 4 bits, because PRESENT_13 is start at bit 4 */
                    break;
                case PRESENT_17 ... PRESENT_24:
                    reg  = VIM_SFP_PRESENT_P17_P24;
                    mask = 0x1 << (attr->index - PRESENT_17);
                    break;

                /* TX_FAULT */
                case TXFAULT_13 ... TXFAULT_16:
                    reg  = VIM_SFP_TXFAULT_P13_P16;
                    mask = 0x1 << (attr->index - TXFAULT_13) << 0x4;    /* Shift 4 bits, because TXFAULT_13 is start at bit 4 */
                    break;
                case TXFAULT_17 ... TXFAULT_24:
                    reg  = VIM_SFP_TXFAULT_P17_P24;
                    mask = 0x1 << (attr->index - TXFAULT_17);
                    break;

                /* TX_DIS */
                case TXDIS_13 ... TXDIS_16:
                    reg  = VIM_SFP_TXDIS_P13_P16;
                    mask = 0x1 << (attr->index - TXDIS_13) << 0x4;      /* Shift 4 bits, because TXDIS_13 is start at bit 4 */
                    
                    break;
                case TXDIS_17 ... TXDIS_24:
                    reg  = VIM_SFP_TXDIS_P17_P24;
                    mask = 0x1 << (attr->index - TXDIS_17);
                    break;

                /* RX_LOS */
                case RXLOS_13 ... RXLOS_16:
                    reg  = VIM_SFP_RXLOS_P13_P16;
                    mask = 0x1 << (attr->index - RXLOS_13) << 0x4;      /* Shift 4 bits, because RXLOS_13 is start at bit 4 */
                    break;
                case RXLOS_17 ... RXLOS_24:
                    reg  = VIM_SFP_RXLOS_P17_P24;
                    mask = 0x1 << (attr->index - RXLOS_17);
                    break;

                default:
                    reg  = NOT_SUPPORT;
                    break;
            }
            break;
        default:
            break;
    }
    
    if (reg != NOT_SUPPORT)
    {
        mutex_lock(&data->update_lock);
        status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_1_ADDRESS, CHANNEL_4_ADDRESS);
        if (unlikely(status < 0)) {
            DEBUG_PRINT("Switch channel failed! board_id = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_1_ADDRESS, CHANNEL_4_ADDRESS);
            goto exit;
        }

        status = extreme7830_32ce_8de_vim_cpld_read_internal(client, reg);
        if (unlikely(status < 0)) {
            goto exit;
        }
        mutex_unlock(&data->update_lock);

        if (attr->index >= PRESENT_13 && attr->index <= PRESENT_24)
        {
            return sprintf(buf, "%d\n", !(!!(status & mask)));
        }
        else
        {
            return sprintf(buf, "%d\n", !!(status & mask));
        }

        
    }
    else
    {
        return sprintf(buf, "%s\n", "NOT SUPPORT");
    }
    

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t set_vim_board_id(struct device *dev, struct device_attribute *da, 
                           const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_port_cpld_data *data = i2c_get_clientdata(client);
    
    long set_value;
    int status;

    status = kstrtol(buf, 10, &set_value);
    if (status) {
        return status;
    }
    mutex_lock(&data->update_lock);
    data->board_id = set_value;
    mutex_unlock(&data->update_lock);

    DEBUG_PRINT("[set_vim_board_id] set board ID to %ld", set_value);

    return count;
}

static ssize_t show_vim_board_id(struct device *dev, struct device_attribute *da,
                           char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_port_cpld_data *data = i2c_get_clientdata(client);
   
	return sprintf(buf, "%d\n", data->board_id);

}

static ssize_t show_vim_cpld_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    int val = 0;
    struct i2c_client *client = to_i2c_client(dev);

    val = i2c_smbus_read_byte_data(client, VIM_CPLD_REG_ADDR_REVISION);

    if (val < 0) {
        dev_dbg(&client->dev, "cpld(0x%x) reg(VIM_CPLD_REG_ADDR_REVISION) err %d\n", client->addr, val);
    }

	val = val & VIM_CPLD_VERSION_BITS_MASK;
    DEBUG_PRINT("[show_vim_cpld_version] get vim_cpld_version %d", val);

    return sprintf(buf, "%d\n", val);
}

/*
 * CPLD client function
 *      - extreme7830_32ce_8de_vim_port_cpld_add_client
 *      - extreme7830_32ce_8de_vim_port_cpld_remove_client
 */

static void extreme7830_32ce_8de_vim_port_cpld_add_client(struct i2c_client *client)
{
    struct cpld_client_node *node = kzalloc(sizeof(struct cpld_client_node), GFP_KERNEL);

    if (!node) {
        dev_dbg(&client->dev, "Can't allocate cpld_client_node (0x%x)\n", client->addr);
        DEBUG_PRINT("Can't allocate cpld_client_node (0x%x)", client->addr);
        return;
    }

    node->client = client;
    DEBUG_PRINT("extreme7830_32ce_8de_vim_port_cpld_add_client success.");
    mutex_lock(&list_lock);
    list_add(&node->list, &cpld_client_list);
    mutex_unlock(&list_lock);
}

static void extreme7830_32ce_8de_vim_port_cpld_remove_client(struct i2c_client *client)
{
    struct list_head    *list_node = NULL;
    struct cpld_client_node *cpld_node = NULL;
    int found = 0;

    mutex_lock(&list_lock);

    list_for_each(list_node, &cpld_client_list) {
        cpld_node = list_entry(list_node, struct cpld_client_node, list);

        if (cpld_node->client == client) {
            found = 1;
            break;
        }
    }

    if (found) {
        list_del(list_node);
        kfree(cpld_node);
    }

    mutex_unlock(&list_lock);
}


/*
 * I2C init/probing/exit functions
 */
static int extreme7830_32ce_8de_vim_port_cpld_probe(struct i2c_client *client,
                                     const struct i2c_device_id *id)
{
    struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
    struct extreme7830_32ce_8de_vim_port_cpld_data *data;
    int ret = 0;
    const struct attribute_group *group = NULL;

    if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE)){
        DEBUG_PRINT("i2c_check_functionality failed");
        return -ENODEV;
    }
        

    data = kzalloc(sizeof(struct extreme7830_32ce_8de_vim_port_cpld_data), GFP_KERNEL);
    if (!data){
        DEBUG_PRINT("kzalloc failed");
        return -ENOMEM;
    }
        
    i2c_set_clientdata(client, data);
    data->type = id->driver_data;
    mutex_init(&data->update_lock);
	
    DEBUG_PRINT("data->type = %d", data->type);
	switch (data->type) 
	{
        case extreme7830_32ce_8de_vim1_cpld2:
            group = &extreme7830_32ce_8de_vim1_cpld2_group;
            /* Set Tx_Disable for transceiver EEPROM for Port 13-16 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_TXDIS_P13_P16, 0x0);
            /* Set Tx_Disable for transceiver EEPROM for Port 17-24 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_TXDIS_P17_P24, 0x0);
            break;
		case extreme7830_32ce_8de_vim2_cpld2:
            group = &extreme7830_32ce_8de_vim2_cpld2_group;
            /* Set Tx_Disable for transceiver EEPROM for Port 13-16 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_TXDIS_P13_P16, 0x0);
            /* Set Tx_Disable for transceiver EEPROM for Port 17-24 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_TXDIS_P17_P24, 0x0);
            break;
        default:
			break;
    }

    if (group) {
        ret = sysfs_create_group(&client->dev.kobj, group);
        if (ret) {
            DEBUG_PRINT("sysfs_create_group failed. ret = %d", ret);
            goto add_failed;
        }
        DEBUG_PRINT("sysfs_create_group sucess. ret = %d", ret);
    }

    dev_info(&client->dev,
             "device %s registered\n", client->name);

    extreme7830_32ce_8de_vim_port_cpld_add_client(client);

    return 0;

add_failed:
    kfree(data);
    return ret;
}

static int extreme7830_32ce_8de_vim_port_cpld_remove(struct i2c_client *client)
{
    struct extreme7830_32ce_8de_vim_port_cpld_data *data = i2c_get_clientdata(client);
    const struct attribute_group *group = NULL;

    extreme7830_32ce_8de_vim_port_cpld_remove_client(client);

    /* Remove sysfs hooks */
    switch (data->type) 
	{
        case extreme7830_32ce_8de_vim1_cpld2:
            group = &extreme7830_32ce_8de_vim1_cpld2_group;
            break;
		case extreme7830_32ce_8de_vim2_cpld2:
            group = &extreme7830_32ce_8de_vim2_cpld2_group;
            break;
        default:
			break;
    }

    if (group) {
        sysfs_remove_group(&client->dev.kobj, group);
    }

    kfree(data);

    return 0;
}

static int extreme7830_32ce_8de_vim_cpld_read_internal(struct i2c_client *client, u8 reg)
{
    int status = 0, retry = I2C_RW_RETRY_COUNT;

    while (retry) {
        status = i2c_smbus_read_byte_data(client, reg);
        if (unlikely(status < 0)) {
            msleep(I2C_RW_RETRY_INTERVAL);
            retry--;
            continue;
        }

        break;
    }
    return status;
}

static int extreme7830_32ce_8de_vim_cpld_write_internal(struct i2c_client *client, u8 reg, u8 value)
{
    int status = 0, retry = I2C_RW_RETRY_COUNT;

    while (retry) {
        status = i2c_smbus_write_byte_data(client, reg, value);
        if (unlikely(status < 0)) {
            msleep(I2C_RW_RETRY_INTERVAL);
            retry--;
            continue;
        }

        break;
    }
    return status;
}

static struct i2c_driver extreme7830_32ce_8de_vim_port_cpld_driver = {
    .driver        = {
        .name     = DRIVER_NAME,
        .owner    = THIS_MODULE,
    },
    .probe         = extreme7830_32ce_8de_vim_port_cpld_probe,
    .remove        = extreme7830_32ce_8de_vim_port_cpld_remove,
    .id_table      = extreme7830_32ce_8de_vim_port_cpld_id,
};

static int __init extreme7830_32ce_8de_vim_port_cpld_init(void)
{
    return i2c_add_driver(&extreme7830_32ce_8de_vim_port_cpld_driver);
}

static void __exit extreme7830_32ce_8de_vim_port_cpld_exit(void)
{
    i2c_del_driver(&extreme7830_32ce_8de_vim_port_cpld_driver);
}

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extreme 7830_32ce_8de VIM port CPLD driver");
MODULE_LICENSE("GPL");

module_init(extreme7830_32ce_8de_vim_port_cpld_init);
module_exit(extreme7830_32ce_8de_vim_port_cpld_exit);