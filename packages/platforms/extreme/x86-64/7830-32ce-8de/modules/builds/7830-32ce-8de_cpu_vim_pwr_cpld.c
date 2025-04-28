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

#define DRIVER_NAME                             "7830_i2c_vim_pwr_cpld"
#define I2C_RW_RETRY_COUNT                      10
#define I2C_RW_RETRY_INTERVAL                   60 /* ms */

#define DEBUG_MODE                              0

/* VIM */
#define VIM_CPLD1_ADDRESS                       0x5C
#define VIM_CPLD_REG_ADDR_BOARD_ID         	    0x01

#define PCA9548_8_ADDRESS                       0x77
#define PCA9548_1_ADDRESS                       0x76
#define CHANNEL_0_ADDRESS  				        0x01	/* CH0 */
#define CHANNEL_1_ADDRESS  				        0x02	/* CH1 */
#define CHANNEL_3_ADDRESS  				        0x08	/* CH3 */

#define VIM_BOARD_ID_MASK                       0x07    /* bit 0~2 */

/* VIM CPLD1 0x5c */
#define VIM_CPLD_REG_ADDR_REVISION              0x00
#define VIM_SFP_PRESENT_P1_P8                   0x10
#define VIM_SFP_PRESENT_P9_P16                  0x12
#define VIM_SFP_RST_MOD_P1_P8                   0x15
#define VIM_SFP_RST_MOD_P9_P16                  0x16
#define VIM_SFP_LP_MODE_P1_P8                   0x18
#define VIM_SFP_LP_MODE_P9_P16                  0x19
#define VIM_SFP_MOD_SEL_P1_P8                   0x1A
#define VIM_SFP_TXFAULT_P1_P8                   0x1B
#define VIM_SFP_TXFAULT_P9_P16                  0x1C
#define VIM_SFP_TXFAULT_B_P1_P8                 0x1D
#define VIM_SFP_TXFAULT_B_P9_P16                0x1E
#define VIM_SFP_TXDIS_P1_P8                     0x20
#define VIM_SFP_TXDIS_P9_P16                    0x21
#define VIM_SFP_TXDIS_B_P1_P8                   0x22
#define VIM_SFP_TXDIS_B_P9_P16                  0x23
#define VIM_SFP_RXLOS_P1_P8                     0x25
#define VIM_SFP_RXLOS_P9_P16                    0x26
#define VIM_SFP_RXLOS_B_P1_P8                   0x28
#define VIM_SFP_RXLOS_B_P9_P16                  0x29
#define VIM_PORT_LED_ON_OFF_CONTROL             0x40
#define VIM_POWER_LED                           0x40

#define NOT_SUPPORT                             0xFF
#define VIM_CPLD_VERSION_BITS_MASK    			0x0F 	/* bit 0~3 */
#define VIM_PORT_LED_ON_OFF_CONTROL_MASK		0x20 	/* bit 5 */
#define VIM_POWER_LED_MASK		                0x18 	/* bit 3~4 */


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

enum cpld_type {
    extreme7830_32ce_8de_vim1_cpld1,
    extreme7830_32ce_8de_vim2_cpld1, 
};

struct extreme7830_32ce_8de_vim_cpld_data {
    enum cpld_type type;
    struct device      *hwmon_dev;
    struct mutex        update_lock;
};

struct chip_desc {
    u8   nchans;
    u8   deselectChan;
};

enum vim_id
{
    VIM_1, 
    VIM_2,
    VIM_ID_MAX
};

enum vim_use_status
{
    VIM_1_USE, 
    VIM_2_USE,
    NOT_USE,
};

enum vim_type_id
{
    VIM_8DE, 
    VIM_16CE,
    VIM_24CE,
    VIM_24YE,
    VIM_NONE, 
    VIM_TYPE_ID_MAX
};


static const struct i2c_device_id extreme7830_32ce_8de_vim_cpld_id[] = {
    { "VIM1_CPLD0", extreme7830_32ce_8de_vim1_cpld1 },
    { "VIM2_CPLD0", extreme7830_32ce_8de_vim2_cpld1 },
    {}
};
MODULE_DEVICE_TABLE(i2c, extreme7830_32ce_8de_vim_cpld_id);


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
#define VIM_CPLD_VER_ATTR_ID(index)				    VIM##index##_CPLD_VERSION
#define VIM_PORT_LED_CONTROL_ATTR_ID(index)         VIM##index##_PORT_LED_CONTROL
#define VIM_POWER_LED_ATTR_ID(index)                VIM##index##_POWER_LED

enum extreme7830_32ce_8de_sys_cpld_sysfs_attributes {
    VIM_CPLD_VER_ATTR_ID(1), 
    VIM_CPLD_VER_ATTR_ID(2),

    PRESENT_ATTR_ID(1), 
    PRESENT_ATTR_ID(2),
    PRESENT_ATTR_ID(3),
    PRESENT_ATTR_ID(4),
    PRESENT_ATTR_ID(5),
    PRESENT_ATTR_ID(6),
    PRESENT_ATTR_ID(7), 
    PRESENT_ATTR_ID(8),
    PRESENT_ATTR_ID(9),
    PRESENT_ATTR_ID(10),
    PRESENT_ATTR_ID(11),
    PRESENT_ATTR_ID(12),
    PRESENT_ATTR_ID(13), 
    PRESENT_ATTR_ID(14),
    PRESENT_ATTR_ID(15),
    PRESENT_ATTR_ID(16),

    RST_MOD_ATTR_ID(1), 
    RST_MOD_ATTR_ID(2),
    RST_MOD_ATTR_ID(3),
    RST_MOD_ATTR_ID(4),
    RST_MOD_ATTR_ID(5),
    RST_MOD_ATTR_ID(6),
    RST_MOD_ATTR_ID(7), 
    RST_MOD_ATTR_ID(8),
    RST_MOD_ATTR_ID(9),
    RST_MOD_ATTR_ID(10),
    RST_MOD_ATTR_ID(11),
    RST_MOD_ATTR_ID(12),
    RST_MOD_ATTR_ID(13), 
    RST_MOD_ATTR_ID(14),
    RST_MOD_ATTR_ID(15),
    RST_MOD_ATTR_ID(16),

    LP_MODE_ATTR_ID(1), 
    LP_MODE_ATTR_ID(2),
    LP_MODE_ATTR_ID(3),
    LP_MODE_ATTR_ID(4),
    LP_MODE_ATTR_ID(5),
    LP_MODE_ATTR_ID(6),
    LP_MODE_ATTR_ID(7), 
    LP_MODE_ATTR_ID(8),
    LP_MODE_ATTR_ID(9),
    LP_MODE_ATTR_ID(10),
    LP_MODE_ATTR_ID(11),
    LP_MODE_ATTR_ID(12),
    LP_MODE_ATTR_ID(13), 
    LP_MODE_ATTR_ID(14),
    LP_MODE_ATTR_ID(15),
    LP_MODE_ATTR_ID(16),

    MOD_SEL_ATTR_ID(1), 
    MOD_SEL_ATTR_ID(2),
    MOD_SEL_ATTR_ID(3),
    MOD_SEL_ATTR_ID(4),
    MOD_SEL_ATTR_ID(5),
    MOD_SEL_ATTR_ID(6),
    MOD_SEL_ATTR_ID(7), 
    MOD_SEL_ATTR_ID(8),
    MOD_SEL_ATTR_ID(9),
    MOD_SEL_ATTR_ID(10),
    MOD_SEL_ATTR_ID(11),
    MOD_SEL_ATTR_ID(12),
    MOD_SEL_ATTR_ID(13), 
    MOD_SEL_ATTR_ID(14),
    MOD_SEL_ATTR_ID(15),
    MOD_SEL_ATTR_ID(16),

    TXFAULT_ATTR_ID(1), 
    TXFAULT_ATTR_ID(2),
    TXFAULT_ATTR_ID(3),
    TXFAULT_ATTR_ID(4),
    TXFAULT_ATTR_ID(5),
    TXFAULT_ATTR_ID(6),
    TXFAULT_ATTR_ID(7), 
    TXFAULT_ATTR_ID(8),
    TXFAULT_ATTR_ID(9),
    TXFAULT_ATTR_ID(10),
    TXFAULT_ATTR_ID(11),
    TXFAULT_ATTR_ID(12),
    TXFAULT_ATTR_ID(13), 
    TXFAULT_ATTR_ID(14),
    TXFAULT_ATTR_ID(15),
    TXFAULT_ATTR_ID(16),

    TXFAULT_B_ATTR_ID(1), 
    TXFAULT_B_ATTR_ID(2),
    TXFAULT_B_ATTR_ID(3),
    TXFAULT_B_ATTR_ID(4),
    TXFAULT_B_ATTR_ID(5),
    TXFAULT_B_ATTR_ID(6),
    TXFAULT_B_ATTR_ID(7), 
    TXFAULT_B_ATTR_ID(8),
    TXFAULT_B_ATTR_ID(9),
    TXFAULT_B_ATTR_ID(10),
    TXFAULT_B_ATTR_ID(11),
    TXFAULT_B_ATTR_ID(12),
    TXFAULT_B_ATTR_ID(13), 
    TXFAULT_B_ATTR_ID(14),
    TXFAULT_B_ATTR_ID(15),
    TXFAULT_B_ATTR_ID(16),

    TXDIS_ATTR_ID(1), 
    TXDIS_ATTR_ID(2),
    TXDIS_ATTR_ID(3),
    TXDIS_ATTR_ID(4),
    TXDIS_ATTR_ID(5),
    TXDIS_ATTR_ID(6),
    TXDIS_ATTR_ID(7), 
    TXDIS_ATTR_ID(8),
    TXDIS_ATTR_ID(9),
    TXDIS_ATTR_ID(10),
    TXDIS_ATTR_ID(11),
    TXDIS_ATTR_ID(12),
    TXDIS_ATTR_ID(13), 
    TXDIS_ATTR_ID(14),
    TXDIS_ATTR_ID(15),
    TXDIS_ATTR_ID(16),

    TXDIS_B_ATTR_ID(1), 
    TXDIS_B_ATTR_ID(2),
    TXDIS_B_ATTR_ID(3),
    TXDIS_B_ATTR_ID(4),
    TXDIS_B_ATTR_ID(5),
    TXDIS_B_ATTR_ID(6),
    TXDIS_B_ATTR_ID(7), 
    TXDIS_B_ATTR_ID(8),
    TXDIS_B_ATTR_ID(9),
    TXDIS_B_ATTR_ID(10),
    TXDIS_B_ATTR_ID(11),
    TXDIS_B_ATTR_ID(12),
    TXDIS_B_ATTR_ID(13), 
    TXDIS_B_ATTR_ID(14),
    TXDIS_B_ATTR_ID(15),
    TXDIS_B_ATTR_ID(16),

    RXLOS_ATTR_ID(1), 
    RXLOS_ATTR_ID(2),
    RXLOS_ATTR_ID(3),
    RXLOS_ATTR_ID(4),
    RXLOS_ATTR_ID(5),
    RXLOS_ATTR_ID(6),
    RXLOS_ATTR_ID(7), 
    RXLOS_ATTR_ID(8),
    RXLOS_ATTR_ID(9),
    RXLOS_ATTR_ID(10),
    RXLOS_ATTR_ID(11),
    RXLOS_ATTR_ID(12),
    RXLOS_ATTR_ID(13), 
    RXLOS_ATTR_ID(14),
    RXLOS_ATTR_ID(15),
    RXLOS_ATTR_ID(16),

    RXLOS_B_ATTR_ID(1), 
    RXLOS_B_ATTR_ID(2),
    RXLOS_B_ATTR_ID(3),
    RXLOS_B_ATTR_ID(4),
    RXLOS_B_ATTR_ID(5),
    RXLOS_B_ATTR_ID(6),
    RXLOS_B_ATTR_ID(7), 
    RXLOS_B_ATTR_ID(8),
    RXLOS_B_ATTR_ID(9),
    RXLOS_B_ATTR_ID(10),
    RXLOS_B_ATTR_ID(11),
    RXLOS_B_ATTR_ID(12),
    RXLOS_B_ATTR_ID(13), 
    RXLOS_B_ATTR_ID(14),
    RXLOS_B_ATTR_ID(15),
    RXLOS_B_ATTR_ID(16),

    VIM_PORT_LED_CONTROL_ATTR_ID(1), 
    VIM_PORT_LED_CONTROL_ATTR_ID(2),

    VIM_POWER_LED_ATTR_ID(1), 
    VIM_POWER_LED_ATTR_ID(2),
};


/* sysfs attributes for hwmon */
static ssize_t show_vim_sfp_status(struct device *dev, struct device_attribute *da,
                           char *buf);
static ssize_t set_vim_sfp_status(struct device *dev, struct device_attribute *da, 
                      const char *buf, size_t count);
static ssize_t show_vim_cpld_version(struct device *dev, struct device_attribute *da,
                           char *buf);
static ssize_t show_vim_port_led_control(struct device *dev, struct device_attribute *da,
                           char *buf);
static ssize_t set_vim_port_led_control(struct device *dev, struct device_attribute *da, 
                      const char *buf, size_t count);
static ssize_t show_vim_power_led(struct device *dev, struct device_attribute *da,
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

/* VIM CPLD Version */
#define DECLARE_VIM_CPLD_VERSION_DEVICE_ATTR(index) \
	static SENSOR_DEVICE_ATTR(vim##index##_cpld_version, S_IRUGO, show_vim_cpld_version, NULL, VIM##index##_CPLD_VERSION)
#define DECLARE_VIM_CPLD_VERSION_ATTR(index)  &sensor_dev_attr_vim##index##_cpld_version.dev_attr.attr

/* VIM Port LED Control */
#define DECLARE_VIM_PORT_LED_CONTROL_DEVICE_ATTR(index) \
	static SENSOR_DEVICE_ATTR(vim##index##_port_led_control, S_IWUSR | S_IRUGO, show_vim_port_led_control, set_vim_port_led_control, VIM##index##_PORT_LED_CONTROL)
#define DECLARE_VIM_PORT_LED_CONTROL_ATTR(index)  &sensor_dev_attr_vim##index##_port_led_control.dev_attr.attr

/* VIM Power LED */
#define DECLARE_VIM_POWER_LED_DEVICE_ATTR(index) \
	static SENSOR_DEVICE_ATTR(vim##index##_power_led, S_IRUGO, show_vim_power_led, NULL, VIM##index##_POWER_LED)
#define DECLARE_VIM_POWER_LED_ATTR(index)  &sensor_dev_attr_vim##index##_power_led.dev_attr.attr

DECLARE_VIM_CPLD_VERSION_DEVICE_ATTR(1);
DECLARE_VIM_CPLD_VERSION_DEVICE_ATTR(2);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(1);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(2);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(3);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(4);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(5);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(6);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(7);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(8);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(9);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(10);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(11);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(12);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(13);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(14);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(15);
DECLARE_VIM_SFP_SENSOR_DEVICE_ATTR(16);
DECLARE_VIM_PORT_LED_CONTROL_DEVICE_ATTR(1);
DECLARE_VIM_PORT_LED_CONTROL_DEVICE_ATTR(2);
DECLARE_VIM_POWER_LED_DEVICE_ATTR(1);
DECLARE_VIM_POWER_LED_DEVICE_ATTR(2);


/* VIM#1 CPLD1 */
static struct attribute *extreme7830_32ce_8de_vim1_cpld1_attributes[] = {
    DECLARE_VIM_CPLD_VERSION_ATTR(1),
    DECLARE_VIM_SFP_ATTR(1), 
    DECLARE_VIM_SFP_ATTR(2), 
    DECLARE_VIM_SFP_ATTR(3), 
    DECLARE_VIM_SFP_ATTR(4), 
    DECLARE_VIM_SFP_ATTR(5), 
    DECLARE_VIM_SFP_ATTR(6), 
    DECLARE_VIM_SFP_ATTR(7), 
    DECLARE_VIM_SFP_ATTR(8), 
    DECLARE_VIM_SFP_ATTR(9), 
    DECLARE_VIM_SFP_ATTR(10), 
    DECLARE_VIM_SFP_ATTR(11), 
    DECLARE_VIM_SFP_ATTR(12), 
    DECLARE_VIM_SFP_ATTR(13), 
    DECLARE_VIM_SFP_ATTR(14), 
    DECLARE_VIM_SFP_ATTR(15), 
    DECLARE_VIM_SFP_ATTR(16), 
    DECLARE_VIM_PORT_LED_CONTROL_ATTR(1),
    DECLARE_VIM_POWER_LED_ATTR(1),

    NULL
};
static const struct attribute_group extreme7830_32ce_8de_vim1_cpld1_group = {
    .attrs = extreme7830_32ce_8de_vim1_cpld1_attributes,
};


/* VIM#2 CPLD1 */
static struct attribute *extreme7830_32ce_8de_vim2_cpld1_attributes[] = {
    DECLARE_VIM_CPLD_VERSION_ATTR(2),
    DECLARE_VIM_SFP_ATTR(1), 
    DECLARE_VIM_SFP_ATTR(2), 
    DECLARE_VIM_SFP_ATTR(3), 
    DECLARE_VIM_SFP_ATTR(4), 
    DECLARE_VIM_SFP_ATTR(5), 
    DECLARE_VIM_SFP_ATTR(6), 
    DECLARE_VIM_SFP_ATTR(7), 
    DECLARE_VIM_SFP_ATTR(8), 
    DECLARE_VIM_SFP_ATTR(9), 
    DECLARE_VIM_SFP_ATTR(10), 
    DECLARE_VIM_SFP_ATTR(11), 
    DECLARE_VIM_SFP_ATTR(12), 
    DECLARE_VIM_SFP_ATTR(13), 
    DECLARE_VIM_SFP_ATTR(14), 
    DECLARE_VIM_SFP_ATTR(15), 
    DECLARE_VIM_SFP_ATTR(16), 
    DECLARE_VIM_PORT_LED_CONTROL_ATTR(2),
    DECLARE_VIM_POWER_LED_ATTR(2),

    NULL
};
static const struct attribute_group extreme7830_32ce_8de_vim2_cpld1_group = {
    .attrs = extreme7830_32ce_8de_vim2_cpld1_attributes,
};


/*
 * Attributes function
 *      - show_vim_sfp_status   : Get VIM SFP attribute
 *      - set_vim_sfp_status    : Set VIM SFP attribute
 *      - show_vim_cpld_version : Get VIM CPLD version
 *      - show_vim_port_led_control   : Get VIM port LED control attribute
 *      - set_vim_port_led_control    : Set VIM port LED control attribute
 *      - show_vim_power_led    : Get VIM power LED attribute
 */
static ssize_t set_vim_sfp_status(struct device *dev, struct device_attribute *da, 
                      const char *buf, size_t count)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);

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
    switch (data->type) 
	{
        case extreme7830_32ce_8de_vim1_cpld1:

            status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_8_ADDRESS, CHANNEL_0_ADDRESS);
            if (unlikely(status < 0)) {
                DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_8_ADDRESS, CHANNEL_0_ADDRESS);
                goto exit;
            }
            break;

		case extreme7830_32ce_8de_vim2_cpld1:

            status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_8_ADDRESS, CHANNEL_1_ADDRESS);
            if (unlikely(status < 0)) {
                DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_8_ADDRESS, CHANNEL_1_ADDRESS);
                goto exit;
            }
            break;

        default:
            goto exit;
			break;
    }

    status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_1_ADDRESS, CHANNEL_3_ADDRESS);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_1_ADDRESS, CHANNEL_3_ADDRESS);
        goto exit;
    }

    
    /* Get board id from power CPLD 0x5c */
    status = extreme7830_32ce_8de_vim_cpld_read_internal(client, VIM_CPLD_REG_ADDR_BOARD_ID);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("vim extreme7830_32ce_8de_vim_cpld_read_internal failed: attr->index:%d, reg=%d", attr->index, VIM_CPLD_REG_ADDR_BOARD_ID);
        goto exit;
    }

    board_id = status & VIM_BOARD_ID_MASK;
    DEBUG_PRINT("[VIM CPU PWR CPLD DEBUG]: attr->index:%d, board_id=%d", attr->index, board_id);
    mutex_unlock(&data->update_lock);
    

    /* Show vim sfp attribute */
    switch (board_id) {
        case VIM_8DE:
            switch (attr->index) {
                /* reset */
                case RST_MOD_1 ... RST_MOD_8:
                    reg  = VIM_SFP_RST_MOD_P1_P8;
                    mask = 0x1 << (attr->index - RST_MOD_1);
                    break;

                /* LP_MODE */
                case LP_MODE_1 ... LP_MODE_8:
                    reg  = VIM_SFP_LP_MODE_P1_P8;
                    mask = 0x1 << (attr->index - LP_MODE_1);
                    break;

                /* MODULE_SEL */
                case MOD_SEL_1 ... MOD_SEL_8:
                    reg  = VIM_SFP_MOD_SEL_P1_P8;
                    mask = 0x1 << (attr->index - MOD_SEL_1);
                    break;

                default:
                    reg = NOT_SUPPORT;
            }

            break;
        case VIM_16CE:
            switch (attr->index) {
                /* reset */
                case RST_MOD_1 ... RST_MOD_8:
                    reg  = VIM_SFP_RST_MOD_P1_P8;
                    mask = 0x1 << (attr->index - RST_MOD_1);
                    break;
                case RST_MOD_9 ... RST_MOD_16:
                    reg  = VIM_SFP_RST_MOD_P9_P16;
                    mask = 0x1 << (attr->index - RST_MOD_9);
                    break;

                /* LP_MODE */
                case LP_MODE_1 ... LP_MODE_8:
                    reg  = VIM_SFP_LP_MODE_P1_P8;
                    mask = 0x1 << (attr->index - LP_MODE_1);
                    break;
                case LP_MODE_9 ... LP_MODE_16:
                    reg  = VIM_SFP_LP_MODE_P9_P16;
                    mask = 0x1 << (attr->index - LP_MODE_9);
                    break;

                default:
                    reg = NOT_SUPPORT;
            }

            break;
        case VIM_24CE:
            switch (attr->index) {
                /* reset */
                case RST_MOD_1 ... RST_MOD_8:
                    reg  = VIM_SFP_RST_MOD_P1_P8;
                    mask = 0x1 << (attr->index - RST_MOD_1);
                    break;
                case RST_MOD_9 ... RST_MOD_12:
                    reg  = VIM_SFP_RST_MOD_P9_P16;
                    mask = 0x1 << (attr->index - RST_MOD_9);     
                    break;

                /* LP_MODE */
                case LP_MODE_1 ... LP_MODE_8:
                    reg  = VIM_SFP_LP_MODE_P1_P8;
                    mask = 0x1 << (attr->index - LP_MODE_1);
                    break;
                case LP_MODE_9 ... LP_MODE_12:
                    reg  = VIM_SFP_LP_MODE_P9_P16;
                    mask = 0x1 << (attr->index - LP_MODE_9);
                    break;

                /* TX_DIS */
                case TXDIS_1 ... TXDIS_8:
                    reg  = VIM_SFP_TXDIS_P1_P8;
                    mask = 0x1 << (attr->index - TXDIS_1);
                    break;
                case TXDIS_9 ... TXDIS_12:
                    reg  = VIM_SFP_TXDIS_P9_P16;
                    mask = 0x1 << (attr->index - TXDIS_9);
                    break;

                /* TX_DIS_B */
                case TXDIS_B_1 ... TXDIS_B_8:
                    reg  = VIM_SFP_TXDIS_B_P1_P8;
                    mask = 0x1 << (attr->index - TXDIS_B_1);
                    break;
                case TXDIS_B_9 ... TXDIS_B_12:
                    reg  = VIM_SFP_TXDIS_B_P9_P16;
                    mask = 0x1 << (attr->index - TXDIS_B_9);
                    break;

                default:
                    reg  = NOT_SUPPORT;
                    break;
            }
            break;
        case VIM_24YE:
            switch (attr->index) {
                /* TX_DIS */
                case TXDIS_1 ... TXDIS_8:
                    reg  = VIM_SFP_TXDIS_P1_P8;
                    mask = 0x1 << (attr->index - TXDIS_1);
                    break;
                case TXDIS_9 ... TXDIS_12:
                    reg  = VIM_SFP_TXDIS_P9_P16;
                    mask = 0x1 << (attr->index - TXDIS_9);
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
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);
    int status = 0;
    u8 reg = 0, mask = 0;
    int board_id = 0;    

    /* Get VIM board ID */
    mutex_lock(&data->update_lock);
    switch (data->type) 
	{
        case extreme7830_32ce_8de_vim1_cpld1:
            status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_8_ADDRESS, CHANNEL_0_ADDRESS);
            if (unlikely(status < 0)) {
                DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_8_ADDRESS, CHANNEL_0_ADDRESS);
                goto exit;
            }
            break;

		case extreme7830_32ce_8de_vim2_cpld1:
            status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_8_ADDRESS, CHANNEL_1_ADDRESS);
            if (unlikely(status < 0)) {
                DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_8_ADDRESS, CHANNEL_1_ADDRESS);
                goto exit;
            }
            break;

        default:
            goto exit;
			break;
    }

    status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_1_ADDRESS, CHANNEL_3_ADDRESS);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_1_ADDRESS, CHANNEL_3_ADDRESS);
        goto exit;
    }

    
    /* Get board id from power CPLD 0x5c */
    status = extreme7830_32ce_8de_vim_cpld_read_internal(client, VIM_CPLD_REG_ADDR_BOARD_ID);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("vim extreme7830_32ce_8de_vim_cpld_read_internal failed: attr->index:%d, reg=%d", attr->index, VIM_CPLD_REG_ADDR_BOARD_ID);
        goto exit;
    }

    board_id = status & VIM_BOARD_ID_MASK;
    DEBUG_PRINT("[VIM CPU PWR CPLD DEBUG]: attr->index:%d, board_id=%d", attr->index, board_id);
    mutex_unlock(&data->update_lock);

    /* Show vim sfp attribute */
    switch (board_id) {
        case VIM_8DE:
            switch (attr->index) {
                /* present */
                case PRESENT_1 ... PRESENT_8:
                    reg  = VIM_SFP_PRESENT_P1_P8;
                    mask = 0x1 << (attr->index - PRESENT_1);
                    break;

                /* reset */
                case RST_MOD_1 ... RST_MOD_8:
                    reg  = VIM_SFP_RST_MOD_P1_P8;
                    mask = 0x1 << (attr->index - RST_MOD_1);
                    break;

                /* LP_MODE */
                case LP_MODE_1 ... LP_MODE_8:
                    reg  = VIM_SFP_LP_MODE_P1_P8;
                    mask = 0x1 << (attr->index - LP_MODE_1);
                    break;

                /* MODULE_SEL */
                case MOD_SEL_1 ... MOD_SEL_8:
                    reg  = VIM_SFP_MOD_SEL_P1_P8;
                    mask = 0x1 << (attr->index - MOD_SEL_1);
                    break;

                default:
                    reg  = NOT_SUPPORT;
                    break;
            }
            break;
        case VIM_16CE:
            switch (attr->index) {
                /* present */
                case PRESENT_1 ... PRESENT_8:
                    reg  = VIM_SFP_PRESENT_P1_P8;
                    mask = 0x1 << (attr->index - PRESENT_1);   
                    break;
                case PRESENT_9 ... PRESENT_16:
                    reg  = VIM_SFP_PRESENT_P9_P16;
                    mask = 0x1 << (attr->index - PRESENT_9);   
                    break;

                /* reset */
                case RST_MOD_1 ... RST_MOD_8:
                    reg  = VIM_SFP_RST_MOD_P1_P8;
                    mask = 0x1 << (attr->index - RST_MOD_1);
                    break;
                case RST_MOD_9 ... RST_MOD_16:
                    reg  = VIM_SFP_RST_MOD_P9_P16;
                    mask = 0x1 << (attr->index - RST_MOD_9);
                    break;

                /* LP_MODE */
                case LP_MODE_1 ... LP_MODE_8:
                    reg  = VIM_SFP_LP_MODE_P1_P8;
                    mask = 0x1 << (attr->index - LP_MODE_1);
                    break;
                case LP_MODE_9 ... LP_MODE_16:
                    reg  = VIM_SFP_LP_MODE_P9_P16;
                    mask = 0x1 << (attr->index - LP_MODE_9);
                    break;

                default:
                    reg  = NOT_SUPPORT;
                    break;
            }
            
            break;

        case VIM_24CE:
            switch (attr->index) {
                /* present */
                case PRESENT_1 ... PRESENT_8:
                    reg  = VIM_SFP_PRESENT_P1_P8;
                    mask = 0x1 << (attr->index - PRESENT_1);
                    break;
                case PRESENT_9 ... PRESENT_12:
                    reg  = VIM_SFP_PRESENT_P9_P16;
                    mask = 0x1 << (attr->index - PRESENT_9);
                    break;
                
                /* reset */
                case RST_MOD_1 ... RST_MOD_8:
                    reg  = VIM_SFP_RST_MOD_P1_P8;
                    mask = 0x1 << (attr->index - RST_MOD_1);
                    break;
                case RST_MOD_9 ... RST_MOD_12:
                    reg  = VIM_SFP_RST_MOD_P9_P16;
                    mask = 0x1 << (attr->index - RST_MOD_9);     
                    break;

                /* LP_MODE */
                case LP_MODE_1 ... LP_MODE_8:
                    reg  = VIM_SFP_LP_MODE_P1_P8;
                    mask = 0x1 << (attr->index - LP_MODE_1);
                    break;
                case LP_MODE_9 ... LP_MODE_12:
                    reg  = VIM_SFP_LP_MODE_P9_P16;
                    mask = 0x1 << (attr->index - LP_MODE_9);
                    break;

                /* TX_FAULT */
                case TXFAULT_1 ... TXFAULT_8:
                    reg  = VIM_SFP_TXFAULT_P1_P8;
                    mask = 0x1 << (attr->index - TXFAULT_1);
                    break;
                case TXFAULT_9 ... TXFAULT_12:
                    reg  = VIM_SFP_TXFAULT_P9_P16;
                    mask = 0x1 << (attr->index - TXFAULT_9);
                    break;
                
                /* TX_FAULT_B */
                case TXFAULT_B_1 ... TXFAULT_B_8:
                    reg  = VIM_SFP_TXFAULT_B_P1_P8;
                    mask = 0x1 << (attr->index - TXFAULT_B_1);
                    break;
                case TXFAULT_B_9 ... TXFAULT_B_12:
                    reg  = VIM_SFP_TXFAULT_B_P9_P16;
                    mask = 0x1 << (attr->index - TXFAULT_B_9);
                    break;

                /* TX_DIS */
                case TXDIS_1 ... TXDIS_8:
                    reg  = VIM_SFP_TXDIS_P1_P8;
                    mask = 0x1 << (attr->index - TXDIS_1);
                    break;
                case TXDIS_9 ... TXDIS_12:
                    reg  = VIM_SFP_TXDIS_P9_P16;
                    mask = 0x1 << (attr->index - TXDIS_9);
                    break;
                
                /* TX_DIS_B */
                case TXDIS_B_1 ... TXDIS_B_8:
                    reg  = VIM_SFP_TXDIS_B_P1_P8;
                    mask = 0x1 << (attr->index - TXDIS_B_1);
                    break;
                case TXDIS_B_9 ... TXDIS_B_12:
                    reg  = VIM_SFP_TXDIS_B_P9_P16;
                    mask = 0x1 << (attr->index - TXDIS_B_9);
                    break;
                
                /* RX_LOS */
                case RXLOS_1 ... RXLOS_8:
                    reg  = VIM_SFP_RXLOS_P1_P8;
                    mask = 0x1 << (attr->index - RXLOS_1);
                    break;
                case RXLOS_9 ... RXLOS_12:
                    reg  = VIM_SFP_RXLOS_P9_P16;
                    mask = 0x1 << (attr->index - RXLOS_9);
                    break;
                
                /* RX_LOS_B */
                case RXLOS_B_1 ... RXLOS_B_8:
                    reg  = VIM_SFP_RXLOS_B_P1_P8;
                    mask = 0x1 << (attr->index - RXLOS_B_1);
                    break;
                case RXLOS_B_9 ... RXLOS_B_12:
                    reg  = VIM_SFP_RXLOS_B_P9_P16;
                    mask = 0x1 << (attr->index - RXLOS_B_9);
                    break;
                
                default:
                    reg  = NOT_SUPPORT;
                    break;
            }
            break;
        case VIM_24YE:
            switch (attr->index) {
                /* present */
                case PRESENT_1 ... PRESENT_8:
                    reg  = VIM_SFP_PRESENT_P1_P8;
                    mask = 0x1 << (attr->index - PRESENT_1);
                    break;
                case PRESENT_9 ... PRESENT_12:
                    reg  = VIM_SFP_PRESENT_P9_P16;
                    mask = 0x1 << (attr->index - PRESENT_9);
                    break;

                /* TX_FAULT */
                case TXFAULT_1 ... TXFAULT_8:
                    reg  = VIM_SFP_TXFAULT_P1_P8;
                    mask = 0x1 << (attr->index - TXFAULT_1);
                    break;
                case TXFAULT_9 ... TXFAULT_12:
                    reg  = VIM_SFP_TXFAULT_P9_P16;
                    mask = 0x1 << (attr->index - TXFAULT_9);
                    break;

                /* TX_DIS */
                case TXDIS_1 ... TXDIS_8:
                    reg  = VIM_SFP_TXDIS_P1_P8;
                    mask = 0x1 << (attr->index - TXDIS_1);
                    break;
                case TXDIS_9 ... TXDIS_12:
                    reg  = VIM_SFP_TXDIS_P9_P16;
                    mask = 0x1 << (attr->index - TXDIS_9);
                    break;

                /* RX_LOS */
                case RXLOS_1 ... RXLOS_8:
                    reg  = VIM_SFP_RXLOS_P1_P8;
                    mask = 0x1 << (attr->index - RXLOS_1);
                    break;
                case RXLOS_9 ... RXLOS_12:
                    reg  = VIM_SFP_RXLOS_P9_P16;
                    mask = 0x1 << (attr->index - RXLOS_9);
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
        status = extreme7830_32ce_8de_vim_cpld_read_internal(client, reg);
        if (unlikely(status < 0)) {
            goto exit;
        }
        mutex_unlock(&data->update_lock);

        if (attr->index >= PRESENT_1 && attr->index <= PRESENT_16)
        {
            DEBUG_PRINT("[VIM CPU PWR CPLD DEBUG]: PRESENT attr->index:%d, board_id=%d, status=%d, mask=%d, value=%d", attr->index, board_id, status, mask, !(!!(status & mask)));
            return sprintf(buf, "%d\n", !(!!(status & mask)));
        }
        else
        {
            DEBUG_PRINT("[VIM CPU PWR CPLD DEBUG]: OTHER attr->index:%d, board_id=%d, status=%d, mask=%d, value=%d", attr->index, board_id, status, mask, !!(status & mask));
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

static ssize_t show_vim_cpld_version(struct device *dev, struct device_attribute *da,
                           char *buf)
{
    int val = 0;
    struct i2c_client *client = to_i2c_client(dev);

    val = i2c_smbus_read_byte_data(client, VIM_CPLD_REG_ADDR_REVISION);

    if (val < 0) {
        dev_dbg(&client->dev, "cpld(0x%x) reg(VIM_CPLD_REG_ADDR_REVISION) err %d\n", client->addr, val);
    }

	val = val & VIM_CPLD_VERSION_BITS_MASK;

    return sprintf(buf, "%d\n", val);
}

static ssize_t set_vim_port_led_control(struct device *dev, struct device_attribute *da, 
                      const char *buf, size_t count)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);

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
    switch (data->type) 
	{
        case extreme7830_32ce_8de_vim1_cpld1:

            status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_8_ADDRESS, CHANNEL_0_ADDRESS);
            if (unlikely(status < 0)) {
                DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_8_ADDRESS, CHANNEL_0_ADDRESS);
                goto exit;
            }
            break;

		case extreme7830_32ce_8de_vim2_cpld1:

            status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_8_ADDRESS, CHANNEL_1_ADDRESS);
            if (unlikely(status < 0)) {
                DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_8_ADDRESS, CHANNEL_1_ADDRESS);
                goto exit;
            }
            break;

        default:
            goto exit;
			break;
    }

    status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_1_ADDRESS, CHANNEL_3_ADDRESS);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_1_ADDRESS, CHANNEL_3_ADDRESS);
        goto exit;
    }

    
    /* Get board id from power CPLD 0x5c */
    status = extreme7830_32ce_8de_vim_cpld_read_internal(client, VIM_CPLD_REG_ADDR_BOARD_ID);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("vim extreme7830_32ce_8de_vim_cpld_read_internal failed: attr->index:%d, reg=%d", attr->index, VIM_CPLD_REG_ADDR_BOARD_ID);
        goto exit;
    }

    board_id = status & VIM_BOARD_ID_MASK;
    DEBUG_PRINT("[VIM CPU PWR CPLD DEBUG]: attr->index:%d, board_id=%d", attr->index, board_id);
    mutex_unlock(&data->update_lock);
    

    /* Show vim sfp attribute */
    switch (board_id) {
        case VIM_8DE:
        case VIM_16CE:
            switch (attr->index) {
                /* port LED control */
                case VIM1_PORT_LED_CONTROL:
                case VIM2_PORT_LED_CONTROL:
                    reg  = VIM_PORT_LED_ON_OFF_CONTROL;
                    mask = VIM_PORT_LED_ON_OFF_CONTROL_MASK;
                    break;
                default:
                    reg  = NOT_SUPPORT;
                    break;
            }
            break;
        case VIM_24CE:
        case VIM_24YE:
            reg  = NOT_SUPPORT;
            break;
        default:
            break;
    }

    if (reg != NOT_SUPPORT)
    {
        mutex_lock(&data->update_lock);
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

static ssize_t show_vim_port_led_control(struct device *dev, struct device_attribute *da,
                           char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);
    int status = 0;
    u8 reg = 0, mask = 0;
    int board_id = 0;    

    /* Get VIM board ID */
    mutex_lock(&data->update_lock);
    switch (data->type) 
	{
        case extreme7830_32ce_8de_vim1_cpld1:
            status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_8_ADDRESS, CHANNEL_0_ADDRESS);
            if (unlikely(status < 0)) {
                DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_8_ADDRESS, CHANNEL_0_ADDRESS);
                goto exit;
            }
            break;

		case extreme7830_32ce_8de_vim2_cpld1:
            status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_8_ADDRESS, CHANNEL_1_ADDRESS);
            if (unlikely(status < 0)) {
                DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_8_ADDRESS, CHANNEL_1_ADDRESS);
                goto exit;
            }
            break;

        default:
            goto exit;
			break;
    }

    status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_1_ADDRESS, CHANNEL_3_ADDRESS);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_1_ADDRESS, CHANNEL_3_ADDRESS);
        goto exit;
    }

    
    /* Get board id from power CPLD 0x5c */
    status = extreme7830_32ce_8de_vim_cpld_read_internal(client, VIM_CPLD_REG_ADDR_BOARD_ID);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("vim extreme7830_32ce_8de_vim_cpld_read_internal failed: attr->index:%d, reg=%d", attr->index, VIM_CPLD_REG_ADDR_BOARD_ID);
        goto exit;
    }

    board_id = status & VIM_BOARD_ID_MASK;
    DEBUG_PRINT("[VIM CPU PWR CPLD DEBUG]: attr->index:%d, board_id=%d", attr->index, board_id);
    mutex_unlock(&data->update_lock);

    /* Show vim sfp attribute */
    switch (board_id) {
        case VIM_8DE:
        case VIM_16CE:
            switch (attr->index) {
                /* port LED control */
                case VIM1_PORT_LED_CONTROL:
                case VIM2_PORT_LED_CONTROL:
                    reg  = VIM_PORT_LED_ON_OFF_CONTROL;
                    mask = VIM_PORT_LED_ON_OFF_CONTROL_MASK;
                    break;
                default:
                    reg  = NOT_SUPPORT;
                    break;
            }
            break;
        case VIM_24CE:
        case VIM_24YE:
            reg  = NOT_SUPPORT;
            break;
        default:
            break;
    }
    
    if (reg != NOT_SUPPORT)
    {
        mutex_lock(&data->update_lock);
        status = extreme7830_32ce_8de_vim_cpld_read_internal(client, reg);
        if (unlikely(status < 0)) {
            goto exit;
        }
        mutex_unlock(&data->update_lock);

        DEBUG_PRINT("[VIM CPU PWR CPLD DEBUG]: OTHER attr->index:%d, board_id=%d, status=%d, mask=%d, value=%d", attr->index, board_id, status, mask, !!(status & mask));
        return sprintf(buf, "%d\n", !!(status & mask));
    }
    else
    {
        return sprintf(buf, "%s\n", "NOT SUPPORT");
    }
    

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t show_vim_power_led(struct device *dev, struct device_attribute *da,
                           char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);
    int status = 0;
    u8 reg = 0, mask = 0;
    int board_id = 0;    

    /* Get VIM board ID */
    mutex_lock(&data->update_lock);
    switch (data->type) 
	{
        case extreme7830_32ce_8de_vim1_cpld1:
            status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_8_ADDRESS, CHANNEL_0_ADDRESS);
            if (unlikely(status < 0)) {
                DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_8_ADDRESS, CHANNEL_0_ADDRESS);
                goto exit;
            }
            break;

		case extreme7830_32ce_8de_vim2_cpld1:
            status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_8_ADDRESS, CHANNEL_1_ADDRESS);
            if (unlikely(status < 0)) {
                DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_8_ADDRESS, CHANNEL_1_ADDRESS);
                goto exit;
            }
            break;

        default:
            goto exit;
			break;
    }

    status = extreme7830_32ce_8de_vim_cpld_write_internal(client, PCA9548_1_ADDRESS, CHANNEL_3_ADDRESS);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("Switch channel failed! attr->index = %d, add=0x%02X, val=0x%02X", attr->index, PCA9548_1_ADDRESS, CHANNEL_3_ADDRESS);
        goto exit;
    }

    
    /* Get board id from power CPLD 0x5c */
    status = extreme7830_32ce_8de_vim_cpld_read_internal(client, VIM_CPLD_REG_ADDR_BOARD_ID);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("vim extreme7830_32ce_8de_vim_cpld_read_internal failed: attr->index:%d, reg=%d", attr->index, VIM_CPLD_REG_ADDR_BOARD_ID);
        goto exit;
    }

    board_id = status & VIM_BOARD_ID_MASK;
    DEBUG_PRINT("[VIM CPU PWR CPLD DEBUG]: attr->index:%d, board_id=%d", attr->index, board_id);
    mutex_unlock(&data->update_lock);

    /* Show vim sfp attribute */
    switch (board_id) {
        case VIM_8DE:
        case VIM_16CE:
        case VIM_24CE:
        case VIM_24YE:
            switch (attr->index) {
                /* VIM Power LED */
                case VIM1_POWER_LED:
                case VIM2_POWER_LED:
                    reg  = VIM_POWER_LED;
                    mask = VIM_POWER_LED_MASK;
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
        status = extreme7830_32ce_8de_vim_cpld_read_internal(client, reg);
        if (unlikely(status < 0)) {
            goto exit;
        }
        mutex_unlock(&data->update_lock);

        DEBUG_PRINT("[VIM CPU PWR CPLD DEBUG]: OTHER attr->index:%d, board_id=%d, status=%d, mask=%d, value=%d", attr->index, board_id, status, mask, !!(status & mask));
        return sprintf(buf, "%d\n", (status & mask) >> 0x3);
    }
    else
    {
        return sprintf(buf, "%s\n", "NOT SUPPORT");
    }
    

exit:
    mutex_unlock(&data->update_lock);
    return status;
}


/*
 * CPLD client function
 *      - extreme7830_32ce_8de_vim_cpld_add_client
 *      - extreme7830_32ce_8de_vim_cpld_remove_client
 */

static void extreme7830_32ce_8de_i2c_vim_pwr_cpld_add_client(struct i2c_client *client)
{
    struct cpld_client_node *node = kzalloc(sizeof(struct cpld_client_node), GFP_KERNEL);

    if (!node) {
        dev_dbg(&client->dev, "Can't allocate cpld_client_node (0x%x)\n", client->addr);
        DEBUG_PRINT("Can't allocate cpld_client_node (0x%x)", client->addr);
        return;
    }

    node->client = client;
    DEBUG_PRINT("extreme7830_32ce_8de_i2c_vim_pwr_cpld_add_client success.");
    mutex_lock(&list_lock);
    list_add(&node->list, &cpld_client_list);
    mutex_unlock(&list_lock);
}

static void extreme7830_32ce_8de_i2c_vim_pwr_cpld_remove_client(struct i2c_client *client)
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
static int extreme7830_32ce_8de_i2c_vim_pwr_cpld_probe(struct i2c_client *client,
                                     const struct i2c_device_id *id)
{
    struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
    struct extreme7830_32ce_8de_vim_cpld_data *data;
    int ret = 0;
    const struct attribute_group *group = NULL;

    if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE)){
        DEBUG_PRINT("i2c_check_functionality failed");
        return -ENODEV;
    }
        

    data = kzalloc(sizeof(struct extreme7830_32ce_8de_vim_cpld_data), GFP_KERNEL);
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
        case extreme7830_32ce_8de_vim1_cpld1:
            group = &extreme7830_32ce_8de_vim1_cpld1_group;
            /* Set reset for transceiver EEPROM for Port 1-8 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_RST_MOD_P1_P8, 0xff);
            /* Set reset for transceiver EEPROM for Port 9-16 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_RST_MOD_P9_P16, 0xff);

            /* Set Module Selector for transceiver EEPROM for Port 1-8 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_MOD_SEL_P1_P8, 0x0);

            /* Set Tx_Disable for transceiver EEPROM for Port 13-16 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_TXDIS_P1_P8, 0x0);
            /* Set Tx_Disable for transceiver EEPROM for Port 17-24 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_TXDIS_P9_P16, 0x0);
            break;
        case extreme7830_32ce_8de_vim2_cpld1:
            group = &extreme7830_32ce_8de_vim2_cpld1_group;
            /* Set reset for transceiver EEPROM for Port 1-8 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_RST_MOD_P1_P8, 0xff);
            /* Set reset for transceiver EEPROM for Port 9-16 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_RST_MOD_P9_P16, 0xff);

            /* Set Module Selector for transceiver EEPROM for Port 1-8 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_MOD_SEL_P1_P8, 0x0);

            /* Set Tx_Disable for transceiver EEPROM for Port 13-16 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_TXDIS_P1_P8, 0x0);
            /* Set Tx_Disable for transceiver EEPROM for Port 17-24 */
            extreme7830_32ce_8de_vim_cpld_write_internal(client, VIM_SFP_TXDIS_P9_P16, 0x0);
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

    extreme7830_32ce_8de_i2c_vim_pwr_cpld_add_client(client);

    return 0;

add_failed:
    kfree(data);
    return ret;
}

static int extreme7830_32ce_8de_i2c_vim_pwr_cpld_remove(struct i2c_client *client)
{
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);
    const struct attribute_group *group = NULL;

    extreme7830_32ce_8de_i2c_vim_pwr_cpld_remove_client(client);

    /* Remove sysfs hooks */
    switch (data->type) 
	{
        case extreme7830_32ce_8de_vim1_cpld1:
            group = &extreme7830_32ce_8de_vim1_cpld1_group;
            break;
        case extreme7830_32ce_8de_vim2_cpld1:
            group = &extreme7830_32ce_8de_vim2_cpld1_group;
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

static struct i2c_driver extreme7830_32ce_8de_i2c_vim_pwr_cpld_driver = {
    .driver        = {
        .name     = DRIVER_NAME,
        .owner    = THIS_MODULE,
    },
    .probe         = extreme7830_32ce_8de_i2c_vim_pwr_cpld_probe,
    .remove        = extreme7830_32ce_8de_i2c_vim_pwr_cpld_remove,
    .id_table      = extreme7830_32ce_8de_vim_cpld_id,
};

static int __init extreme7830_32ce_8de_i2c_vim_pwr_cpld_init(void)
{
    return i2c_add_driver(&extreme7830_32ce_8de_i2c_vim_pwr_cpld_driver);
}

static void __exit extreme7830_32ce_8de_i2c_vim_pwr_cpld_exit(void)
{
    i2c_del_driver(&extreme7830_32ce_8de_i2c_vim_pwr_cpld_driver);
}

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extreme 7830_32ce_8de VIM power CPLD driver access from CPU");
MODULE_LICENSE("GPL");

module_init(extreme7830_32ce_8de_i2c_vim_pwr_cpld_init);
module_exit(extreme7830_32ce_8de_i2c_vim_pwr_cpld_exit);