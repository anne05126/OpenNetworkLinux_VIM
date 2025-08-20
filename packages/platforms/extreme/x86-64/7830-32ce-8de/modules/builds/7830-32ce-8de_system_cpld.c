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

#define DRIVER_NAME                             "7830_i2c_sys_cpld"
#define I2C_RW_RETRY_COUNT                      10
#define I2C_RW_RETRY_INTERVAL                   60 /* ms */

/* Reference from System CPLD spec v06_20240627 */
#define CPLD_REG_ADDR_VIM_PRESENT         	    0x30
#define CPLD_REG_ADDR_VIM_RESET				    0x23	/* bit 2: VIM#1, bit 3: VIM#2 */
#define CPLD_REG_ADDR_VIM1_PWR_CTRL			    0x60
#define CPLD_REG_ADDR_VIM2_PWR_CTRL			    0x61
#define CPLD_REG_ADDR_VIM_PWR_GOOD			    0x13

#define PRESENT_VIM1_BIT				        0x0
#define PRESENT_VIM2_BIT				        0x1
#define RESET_VIM1_BIT					        0x2
#define RESET_VIM2_BIT					        0x3
#define PWR_GOOD_VIM1_BIT                       0x0
#define PWR_GOOD_VIM2_BIT                       0x1

#define CPLD_REG_ADDR_REVISION                  0x00
#define CPLD_VERSION_BITS_MASK    		        0xF

#define DEBUG_MODE                              1

/* VIM */
#define VIM_CPLD1_ADDRESS                       0x5C
#define VIM_CPLD2_ADDRESS                       0x58
#define VIM_CPLD_REG_ADDR_BOARD_ID         	    0x01

#define VIM_POWER_GOOD                          (1)
#define VIM_POWER_FAIL                          (0)

static unsigned int debug = 0;
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
    extreme7830_32ce_8de_sys_cpld,
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

enum vim_power_good_bits
{
    VIM_1_MP5990 = 0,
    VIM_2_MP5990,
    VIM_1_ISOLATION_IC,
    VIM_2_ISOLATION_IC,
};

static const struct i2c_device_id extreme7830_32ce_8de_vim_cpld_id[] = {
    { "system_cpld", extreme7830_32ce_8de_sys_cpld },
    {}
};
MODULE_DEVICE_TABLE(i2c, extreme7830_32ce_8de_vim_cpld_id);

#define VIM_PRESENT_ATTR_ID(index)         			VIM_##index##_PRESENT
#define VIM_RESET_ATTR_ID(index)					VIM_##index##_RESET
#define VIM_PWR_CTRL_ATTR_ID(index)                 VIM_##index##_PWR_CTRL
#define VIM_PWR_GOOD_ATTR_ID(index)                 VIM_##index##_PWR_GOOD


enum extreme7830_32ce_8de_sys_cpld_sysfs_attributes {
    CPLD_VERSION,
    ACCESS,
    MODULE_PRESENT_ALL,

    /* transceiver attributes */
    VIM_PRESENT_ATTR_ID(1),
    VIM_PRESENT_ATTR_ID(2),
    VIM_RESET_ATTR_ID(1),
    VIM_RESET_ATTR_ID(2),
    VIM_PWR_CTRL_ATTR_ID(1),
    VIM_PWR_CTRL_ATTR_ID(2),
    VIM_PWR_GOOD_ATTR_ID(1),
    VIM_PWR_GOOD_ATTR_ID(2),

};


/* sysfs attributes for hwmon */
static ssize_t show_vim_status(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t access(struct device *dev, struct device_attribute *da, const char *buf, size_t count);
static ssize_t show_sys_cpld_version(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t show_vim_power_control(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t set_vim_power_control(struct device *dev, struct device_attribute *da, const char *buf, size_t count);
static ssize_t show_vim_power_good(struct device *dev, struct device_attribute *da, char *buf);

static int extreme7830_32ce_8de_vim_cpld_read_internal(struct i2c_client *client, u8 reg);
static int extreme7830_32ce_8de_vim_cpld_write_internal(struct i2c_client *client, u8 reg, u8 value);

/* VIM_PRESENT attributes */
#define DECLARE_VIM_PRESENT_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(vim_##index##_present, S_IRUGO, show_vim_status, NULL, VIM_##index##_PRESENT)
#define DECLARE_VIM_PRESENT_ATTR(index)  &sensor_dev_attr_vim_##index##_present.dev_attr.attr

/* VIM_RESET attributes */
#define DECLARE_VIM_RESET_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(vim_##index##_reset, S_IRUGO, show_vim_status, NULL, VIM_##index##_RESET)
#define DECLARE_VIM_RESET_ATTR(index)  &sensor_dev_attr_vim_##index##_reset.dev_attr.attr

/* VIM_PWR_CTRL attributes */
#define DECLARE_VIM_PWR_CTRL_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(vim_##index##_pwr_ctrl, S_IWUSR | S_IRUGO, show_vim_power_control, set_vim_power_control, VIM_##index##_PWR_CTRL)
#define DECLARE_VIM_PWR_CTRL_ATTR(index)  &sensor_dev_attr_vim_##index##_pwr_ctrl.dev_attr.attr

/* VIM_PWR_GOOD attributes */
#define DECLARE_VIM_PWR_GOOD_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(vim_##index##_pwr_good, S_IRUGO, show_vim_power_good, NULL, VIM_##index##_PWR_GOOD)
#define DECLARE_VIM_PWR_GOOD_ATTR(index)  &sensor_dev_attr_vim_##index##_pwr_good.dev_attr.attr

/* access attributes */
static SENSOR_DEVICE_ATTR(access, S_IWUSR, NULL, access, ACCESS);

/* show_sys_cpld_version attributes */
static SENSOR_DEVICE_ATTR(version, S_IRUGO, show_sys_cpld_version, NULL, CPLD_VERSION);

DECLARE_VIM_PRESENT_SENSOR_DEVICE_ATTR(1);
DECLARE_VIM_PRESENT_SENSOR_DEVICE_ATTR(2);

DECLARE_VIM_RESET_SENSOR_DEVICE_ATTR(1);
DECLARE_VIM_RESET_SENSOR_DEVICE_ATTR(2);

DECLARE_VIM_PWR_CTRL_SENSOR_DEVICE_ATTR(1);
DECLARE_VIM_PWR_CTRL_SENSOR_DEVICE_ATTR(2);

DECLARE_VIM_PWR_GOOD_SENSOR_DEVICE_ATTR(1);
DECLARE_VIM_PWR_GOOD_SENSOR_DEVICE_ATTR(2);

/* SYSTEM CPLD */
static struct attribute *extreme7830_32ce_8de_sys_cpld_attributes[] = {
    &sensor_dev_attr_version.dev_attr.attr,
    &sensor_dev_attr_access.dev_attr.attr,

    /* transceiver attributes */
    DECLARE_VIM_PRESENT_ATTR(1),
    DECLARE_VIM_PRESENT_ATTR(2),
    DECLARE_VIM_RESET_ATTR(1),
    DECLARE_VIM_RESET_ATTR(2),
    DECLARE_VIM_PWR_CTRL_ATTR(1),
    DECLARE_VIM_PWR_CTRL_ATTR(2),
    DECLARE_VIM_PWR_GOOD_ATTR(1),
    DECLARE_VIM_PWR_GOOD_ATTR(2),
    NULL
};
static const struct attribute_group extreme7830_32ce_8de_sys_cpld_group = {
    .attrs = extreme7830_32ce_8de_sys_cpld_attributes,
};

/*
 * Attributes function
 *      - show_vim_status           : VIM present and reset
 *      - access                    : Access cpld register
 *      - show_sys_cpld_version     : Sys cpld version
 *      - show_vim_power_control    : Get VIM power control
 *      - set_vim_power_control     : Set VIM power control
 *      - show_vim_power_good       : Get VIM power good
 */

static ssize_t show_vim_status(struct device *dev, struct device_attribute *da,
                           char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);
    int status = 0;
    u8 reg = 0, mask = 0;

    switch (attr->index) {
		/* PRESENT
         * i2cget -y -f 0 0x6e 0x30
         */
		case VIM_1_PRESENT:
			reg  = CPLD_REG_ADDR_VIM_PRESENT;
			mask = 0x1 << PRESENT_VIM1_BIT;
			break;
		case VIM_2_PRESENT:
			reg  = CPLD_REG_ADDR_VIM_PRESENT;
			mask = 0x1 << PRESENT_VIM2_BIT;
			break;

        /* RESET
         * i2cget -y -f 0 0x6e 0x23
         */
        case VIM_1_RESET:
			reg  = CPLD_REG_ADDR_VIM_RESET;
			mask = 0x1 << RESET_VIM1_BIT;
			break;
		case VIM_2_RESET:
			reg  = CPLD_REG_ADDR_VIM_RESET;
			mask = 0x1 << RESET_VIM2_BIT;
			break;
		default:
			DEBUG_PRINT("vim show_vim_status failed: attr->index:%d", attr->index);
			return 0;
    }

    mutex_lock(&data->update_lock);
    status = extreme7830_32ce_8de_vim_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
		DEBUG_PRINT("vim extreme7830_32ce_8de_vim_cpld_read_internal failed: attr->index:%d, reg=%d", attr->index, reg);
        goto exit;
    }
    mutex_unlock(&data->update_lock);


    if (attr->index >= VIM_1_PRESENT && attr->index<= VIM_2_RESET)
    {
		return sprintf(buf, "%d\n", !!(status & mask));
    }

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t show_vim_power_control(struct device *dev, struct device_attribute *da,
			char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);
    int status = 0;
    u8 reg = 0;

    switch (attr->index) {
		case VIM_1_PWR_CTRL:
			reg  = CPLD_REG_ADDR_VIM1_PWR_CTRL;
			break;
		case VIM_2_PWR_CTRL:
			reg  = CPLD_REG_ADDR_VIM2_PWR_CTRL;
			break;
		default:
			DEBUG_PRINT("vim show_vim_power_control failed: attr->index:%d", attr->index);
			return 0;
    }

    mutex_lock(&data->update_lock);
    status = extreme7830_32ce_8de_vim_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
		DEBUG_PRINT("vim extreme7830_32ce_8de_vim_cpld_read_internal failed: attr->index:%d, reg=%d", attr->index, reg);
        goto exit;
    }
    mutex_unlock(&data->update_lock);

	return sprintf(buf, "0x%02x\n", status);

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t set_vim_power_control(struct device *dev, struct device_attribute *da,
		       const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);

	int status= -ENOENT;
	long reg_value;
	int error;
	u8 reg = 0;

	switch (attr->index) {
		case VIM_1_PWR_CTRL:
			reg  = CPLD_REG_ADDR_VIM1_PWR_CTRL;
			break;
		case VIM_2_PWR_CTRL:
			reg  = CPLD_REG_ADDR_VIM2_PWR_CTRL;
			break;
		default:
			DEBUG_PRINT("vim set_vim_power_control failed: attr->index:%d", attr->index);
			return 0;
    }


	error = kstrtol(buf, 10, &reg_value);
	if (error)
		return error;

    status = extreme7830_32ce_8de_vim_cpld_write_internal(client, reg, reg_value);
    if (unlikely(status < 0)) {
        goto exit;
    }

    mutex_unlock(&data->update_lock);
    return count;

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static ssize_t show_vim_power_good(struct device *dev, struct device_attribute *da,
			char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);
    int status = 0;
    u8 reg = 0, mask = 0;

    /* MP5990 and Isolation IC of VIM Power Good */
    reg  = CPLD_REG_ADDR_VIM_PWR_GOOD;
    switch (attr->index) {
		case VIM_1_PWR_GOOD:
			mask = (0x1 << VIM_1_MP5990) | (0x1 << VIM_1_ISOLATION_IC);
			break;
		case VIM_2_PWR_GOOD:
            mask = (0x1 << VIM_2_MP5990) | (0x1 << VIM_2_ISOLATION_IC);
			break;
		default:
			DEBUG_PRINT("vim show_vim_power_good failed: attr->index:%d", attr->index);
			return 0;
    }

    mutex_lock(&data->update_lock);
    status = extreme7830_32ce_8de_vim_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
        DEBUG_PRINT("vim extreme7830_32ce_8de_vim_cpld_read_internal failed: attr->index:%d, reg=%d", attr->index, reg);
        goto exit;
    }
    mutex_unlock(&data->update_lock);

    /* Confirm that both the MP5990 and the Isolation IC indicate power good */
    if ((status & mask) == mask)
    {
        return sprintf(buf, "%d\n", VIM_POWER_GOOD);
    }
    else
    {
        return sprintf(buf, "%d\n", VIM_POWER_FAIL);
    }

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t access(struct device *dev, struct device_attribute *da,
                      const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);

    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);
    int status;
    u32 addr, val;

    if (sscanf(buf, "0x%x 0x%x", &addr, &val) != 2) {
        return -EINVAL;
    }

    if (addr > 0xFF || val > 0xFF) {
        return -EINVAL;
    }

    mutex_lock(&data->update_lock);
    status = extreme7830_32ce_8de_vim_cpld_write_internal(client, addr, val);
    if (unlikely(status < 0)) {
        goto exit;
    }
    mutex_unlock(&data->update_lock);
    return count;

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t show_sys_cpld_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    int val = 0;
    struct i2c_client *client = to_i2c_client(dev);

    val = i2c_smbus_read_byte_data(client, CPLD_REG_ADDR_REVISION);

    if (val < 0) {
        dev_dbg(&client->dev, "cpld(0x%x) reg(CPLD_REG_ADDR_REVISION) err %d\n", client->addr, val);
    }

	val = val & CPLD_VERSION_BITS_MASK;

    return sprintf(buf, "%d\n", val);
}


/*
 * CPLD client function
 *      - extreme7830_32ce_8de_vim_cpld_add_client
 *      - extreme7830_32ce_8de_vim_cpld_remove_client
 */

static void extreme7830_32ce_8de_vim_cpld_add_client(struct i2c_client *client)
{
    struct cpld_client_node *node = kzalloc(sizeof(struct cpld_client_node), GFP_KERNEL);

    if (!node) {
        dev_dbg(&client->dev, "Can't allocate cpld_client_node (0x%x)\n", client->addr);
        DEBUG_PRINT("Can't allocate cpld_client_node (0x%x)", client->addr);
        return;
    }

    node->client = client;
    DEBUG_PRINT("extreme7830_32ce_8de_vim_cpld_add_client success.");
    mutex_lock(&list_lock);
    list_add(&node->list, &cpld_client_list);
    mutex_unlock(&list_lock);
}

static void extreme7830_32ce_8de_vim_cpld_remove_client(struct i2c_client *client)
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
static int extreme7830_32ce_8de_vim_cpld_probe(struct i2c_client *client,
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
		case extreme7830_32ce_8de_sys_cpld:
			group = &extreme7830_32ce_8de_sys_cpld_group;
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

    extreme7830_32ce_8de_vim_cpld_add_client(client);

    return 0;

add_failed:
    kfree(data);
    return ret;
}

static int extreme7830_32ce_8de_vim_cpld_remove(struct i2c_client *client)
{
    struct extreme7830_32ce_8de_vim_cpld_data *data = i2c_get_clientdata(client);
    const struct attribute_group *group = NULL;

    extreme7830_32ce_8de_vim_cpld_remove_client(client);

    /* Remove sysfs hooks */
    switch (data->type)
	{
		case extreme7830_32ce_8de_sys_cpld:
			group = &extreme7830_32ce_8de_sys_cpld_group;
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


static struct i2c_driver extreme7830_32ce_8de_vim_cpld_driver = {
    .driver        = {
        .name     = DRIVER_NAME,
        .owner    = THIS_MODULE,
    },
    .probe         = extreme7830_32ce_8de_vim_cpld_probe,
    .remove        = extreme7830_32ce_8de_vim_cpld_remove,
    .id_table      = extreme7830_32ce_8de_vim_cpld_id,
};

static int __init extreme7830_32ce_8de_vim_cpld_init(void)
{
    return i2c_add_driver(&extreme7830_32ce_8de_vim_cpld_driver);
}

static void __exit extreme7830_32ce_8de_vim_cpld_exit(void)
{
    i2c_del_driver(&extreme7830_32ce_8de_vim_cpld_driver);
}

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extreme 7830_32ce_8de VIM CPLD driver");
MODULE_LICENSE("GPL");

module_init(extreme7830_32ce_8de_vim_cpld_init);
module_exit(extreme7830_32ce_8de_vim_cpld_exit);