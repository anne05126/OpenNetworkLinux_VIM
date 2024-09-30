/*
 * A LED kernel dirver for extremenetworks 7830-32ce-8de
 *
 * Copyright (C) 2023 Alphanetworks Technology Corporation.
 * Ruby Wang <Ru-xin_Wang@alphanetworks.com>
 *
 * Based on:
 * Copyright (C) 2023 Alphanetworks Technology Corporation.
 * Anne Liou <anne_liou@alphanetworks.com>
 *
 * Based on:
 * Copyright (C) 2021 Alphanetworks Technology Corporation.
 * Fillmore Chen <fillmore_chen@alphanetworks.com>
 * 
 * Based on:
 * Copyright (C) 2021 Accton Technology Corporation.
 * Copyright (C)  Alex Lai <alex_lai@edge-core.com>
 *
 * Based on:
 *	pca954x.c from Kumar Gala <galak@kernel.crashing.org>
 * Copyright (C) 2006
 *
 * Based on:
 *	pca954x.c from Ken Harrenstien
 * Copyright (C) 2004 Google, Inc. (Ken Harrenstien)
 *
 * Based on:
 *	i2c-virtual_cb.c from Brian Kuschak <bkuschak@yahoo.com>
 * 	and pca9540.c from Jean Delvare <khali@linux-fr.org>.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/platform_device.h>

#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/delay.h>
#include <linux/printk.h>


/* Base on Power CPLD spec v10 */
#define DRIVER_NAME								"7830_i2c_led"
#define I2C_RW_RETRY_COUNT                		10
#define I2C_RW_RETRY_INTERVAL             		60 /* ms */

#define PWRCPLD_LED_Ctrl_0_OFFSET		        0x50

#define sw_led_control                      	0x80 

/* LED Control Register 0
 * bit 7   : 0=hw control,   1=factory test (CPU)
 * bit 6, 2: Reseved
 * bit 1   : Power 0=OFF, 1=Solid Green
 * bit 0   : Security 0=OFF 1=ON
 */
#define SEC_LED_BIT_OFFSET						0x0
#define PWR_LED_BIT_OFFSET						0x1     


/* POWER_LED (map to driver) */
#define PWR_LED_MODE_OFF               			0x0
#define PWR_LED_MODE_GREEN                		0x1

/* Security_LED (map to driver) */
#define SEC_LED_MODE_OFF               			0x0
#define SEC_LED_MODE_BLUE              			0x1

static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");


#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)

/* set_led_by_cpu */
static LIST_HEAD(cpld_client_list);
static struct mutex     list_lock;

struct cpld_client_node {
    struct i2c_client *client;
    struct list_head   list;
};

enum cpld_type {
    extreme7830_32ce_8de_power_cpld
};

struct extreme7830_32ce_8de_power_cpld_data {
    enum cpld_type type;
    struct device      *hwmon_dev;
    struct mutex        update_lock;
};

struct chip_desc {
    u8   nchans;
    u8   deselectChan;
};

static const struct i2c_device_id extreme7830_32ce_8de_power_cpld_id[] = {
    { "power_cpld", extreme7830_32ce_8de_power_cpld },
    { }
};
MODULE_DEVICE_TABLE(i2c, extreme7830_32ce_8de_power_cpld_id);

static ssize_t set_led_by_cpu(struct device *dev, struct device_attribute *da, const char *buf, size_t count);
static ssize_t show_led_by_cpu(struct device *dev, struct device_attribute *da, char *buf);
static int extreme7830_32ce_8de_cpld_read_internal(struct i2c_client *client, u8 reg);
static int extreme7830_32ce_8de_cpld_write_internal(struct i2c_client *client, u8 reg, u8 value);



enum led_data_index {
	PWR_INDEX = 0,
 	SEC_INDEX
};

enum extreme7830_32ce_8de_led_sysfs_attrs {
	LED_PWR,
	LED_SEC	
};


static SENSOR_DEVICE_ATTR(led_pwr, S_IWUSR | S_IRUGO, show_led_by_cpu, set_led_by_cpu, LED_PWR);
static SENSOR_DEVICE_ATTR(led_security, S_IWUSR | S_IRUGO, show_led_by_cpu, set_led_by_cpu, LED_SEC);


static struct attribute *extreme7830_32ce_8de_led_attributes[] = {
	&sensor_dev_attr_led_pwr.dev_attr.attr,
	&sensor_dev_attr_led_security.dev_attr.attr,
	NULL
};

static const struct attribute_group extreme7830_32ce_8de_led_group = {
	.attrs = extreme7830_32ce_8de_led_attributes,
};

static ssize_t show_led_by_cpu(struct device *dev, struct device_attribute *da,
			char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_power_cpld_data *data = i2c_get_clientdata(client);

	int value = 0;
	int status = 0;
    u8 reg = 0, mask = 0;
	reg  = PWRCPLD_LED_Ctrl_0_OFFSET;
    
    mutex_lock(&data->update_lock);
    status = extreme7830_32ce_8de_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
        goto exit;
    }

	switch (attr->index)
	{   
        case LED_PWR:
			mask = 0x1 << PWR_LED_BIT_OFFSET;
			value = (status & mask) >> PWR_LED_BIT_OFFSET;
			break;
        case LED_SEC:
			mask = 0x1 << SEC_LED_BIT_OFFSET;
			value = (status & mask) >> SEC_LED_BIT_OFFSET;
            break;
        default:
            return -ENODEV;
    }

	mutex_unlock(&data->update_lock);
    return sprintf(buf, "%d\n", value);

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

/* set_led_by_cpu */
static ssize_t set_led_by_cpu(struct device *dev, struct device_attribute *da,
		       const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme7830_32ce_8de_power_cpld_data *data = i2c_get_clientdata(client);

	int status= -ENOENT;
	long mode;
	int error;
	u8 reg = 0, mask = 0;
	int value = 0;

	reg  = PWRCPLD_LED_Ctrl_0_OFFSET;

	error = kstrtol(buf, 10, &mode);
	if (error)
		return error;
    
    mutex_lock(&data->update_lock);
    status = extreme7830_32ce_8de_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
        goto exit;
    }

	switch (attr->index) {
		case LED_PWR:
			mask = ~(0x1 << PWR_LED_BIT_OFFSET);
			value = status & mask;
			if (mode == PWR_LED_MODE_OFF) {
				status = value;
			}
			else if (mode == PWR_LED_MODE_GREEN) {
				mask = PWR_LED_MODE_GREEN << PWR_LED_BIT_OFFSET;
				status = value | mask;
			}
			else {
				error = -EINVAL;
				goto exit;
			}
			break; 
		case LED_SEC:
			mask = ~(0x1 << SEC_LED_BIT_OFFSET);
			value = status & mask;
			if(mode == SEC_LED_MODE_OFF) {
				status = value;
			}
			else if(mode == SEC_LED_MODE_BLUE) {
				mask = SEC_LED_MODE_BLUE << SEC_LED_BIT_OFFSET;
				status = value | mask;
			}
			else {
				error = -EINVAL;
				goto exit;
			}
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	status |= sw_led_control;

	DEBUG_PRINT("set_led_by_cpu: attr->index:%d, mask:0x%x, value:0x%x, status:0x%x", attr->index, mask, value, status);
	
    status = extreme7830_32ce_8de_cpld_write_internal(client, reg, status);
    if (unlikely(status < 0)) {
        goto exit;
    }

    mutex_unlock(&data->update_lock);
    return count;

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static int extreme7830_32ce_8de_cpld_write_internal(struct i2c_client *client, u8 reg, u8 value)
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
    DEBUG_PRINT("extreme7830_32ce_8de_cpld_write_internal: reg:0x%x, val:0x%x , status:0x%x", reg, value, status);
    return status;
}

static int extreme7830_32ce_8de_cpld_read_internal(struct i2c_client *client, u8 reg)
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
    DEBUG_PRINT("extreme7830_32ce_8de_cpld_read_internal: reg:0x%x, status:0x%x", reg, status);
    return status;
}

static void extreme7830_32ce_8de_cpld_add_client(struct i2c_client *client)
{
    struct cpld_client_node *node = kzalloc(sizeof(struct cpld_client_node), GFP_KERNEL);

    if (!node) {
        dev_dbg(&client->dev, "Can't allocate cpld_client_node (0x%x)\n", client->addr);
        return;
    }

    node->client = client;

    mutex_lock(&list_lock);
    list_add(&node->list, &cpld_client_list);
    mutex_unlock(&list_lock);
}

static void extreme7830_32ce_8de_cpld_remove_client(struct i2c_client *client)
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

static int extreme7830_32ce_8de_power_cpld_probe(struct i2c_client *client,
                                     const struct i2c_device_id *id)
{
    struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
    struct extreme7830_32ce_8de_power_cpld_data *data;
    int ret = 0;
    const struct attribute_group *group = NULL;

    if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE))
        return -ENODEV;

    data = kzalloc(sizeof(struct extreme7830_32ce_8de_power_cpld_data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    i2c_set_clientdata(client, data);
    data->type = id->driver_data;
    mutex_init(&data->update_lock);

    /* Register sysfs hooks */
    switch (data->type) {
		case extreme7830_32ce_8de_power_cpld:
			group = &extreme7830_32ce_8de_led_group;
			break;
		default:
			break;
    }

    if (group) {
        ret = sysfs_create_group(&client->dev.kobj, group);
        if (ret) {
            goto add_failed;
        }
    }

    dev_info(&client->dev,
             "device %s registered\n", client->name);

    extreme7830_32ce_8de_cpld_add_client(client);

    return 0;

add_failed:
    kfree(data);
    return ret;
}

static int extreme7830_32ce_8de_power_cpld_remove(struct i2c_client *client)
{
    struct extreme7830_32ce_8de_power_cpld_data *data = i2c_get_clientdata(client);
    const struct attribute_group *group = NULL;

    extreme7830_32ce_8de_cpld_remove_client(client);

    /* Remove sysfs hooks */
    switch (data->type) {
		case extreme7830_32ce_8de_power_cpld:
			group = &extreme7830_32ce_8de_led_group;
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

static struct i2c_driver extreme7830_32ce_8de_i2c_cpld_driver = {
    .driver        = {
        .name     = DRIVER_NAME,
        .owner    = THIS_MODULE,
    },
    .probe         = extreme7830_32ce_8de_power_cpld_probe,
    .remove        = extreme7830_32ce_8de_power_cpld_remove,
    .id_table      = extreme7830_32ce_8de_power_cpld_id,
};

static int __init extreme7830_32ce_8de_led_init(void)
{
	mutex_init(&list_lock);
    return i2c_add_driver(&extreme7830_32ce_8de_i2c_cpld_driver);
}

static void __exit extreme7830_32ce_8de_led_exit(void)
{
	i2c_del_driver(&extreme7830_32ce_8de_i2c_cpld_driver);
}

module_init(extreme7830_32ce_8de_led_init);
module_exit(extreme7830_32ce_8de_led_exit);

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extremenetworks 7830-32ce-8de LED driver");
MODULE_LICENSE("GPL");

