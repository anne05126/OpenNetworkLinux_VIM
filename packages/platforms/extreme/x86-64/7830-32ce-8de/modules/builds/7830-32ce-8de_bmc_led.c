/*
 * A LED IPMI kernel dirver for extremenetworks 7830-32ce-8de
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
#include <linux/ipmi.h>
#include <linux/ipmi_smi.h>
#include <linux/platform_device.h>

#include <linux/delay.h>
#include <linux/printk.h>


/* Base on Power CPLD spec v10 */
#define DRVNAME 					        	"7830_bmc_led"

#define IPMI_APP_NETFN							0x6
#define IPMI_READ_WRITE_CMD						0x52
#define IPMI_PWRCPLD_BUS						0x09
#define IPMI_PWRCPLD_ADDRESS					0xbc	/* 0x5e << 1 = 0xbc */
#define IPMI_READ_BYTE							0x01 	/* Read */

#define IPMI_LED_Ctrl_1_OFFSET		        	0x51

#define SW_LED_CONTROL                      	0x80


/* LED Control Register 1
 * bit 7   : 0=hw control,   1=factory test (BMC)
 * bit 6, 5: Reseved
 * bit 4, 3: FAN 00=OFF, 01=Solid Green, 10=Solid Amber, 11=Blinking Amber-Green
 * bit 2, 0: PSU 000=OFF, 001=Blinking Green, 010=Solid Green, 011=Blinking Amber, 100=Solid Amber, 101=Blinking Green and Amber, 110-111=Reserved
 */
#define PSU_LED_BIT_OFFSET                  	0x0
#define FAN_LED_BIT_OFFSET                  	0x3
#define LED_CONTROL_BIT_OFFSET					0x7

/* FAN_LED (map to driver) */
#define FAN_LED_MODE_OFF                    	0x0
#define FAN_LED_MODE_GREEN                  	0x1
#define FAN_LED_MODE_AMBER                  	0x2
#define FAN_LED_MODE_GREEN_AMBER_BLINKING   	0x3

/* PSU_LED (map to driver) */
#define PSU_LED_MODE_OFF                   		0x0
#define PSU_LED_MODE_GREEN_BLINKING        		0x1
#define PSU_LED_MODE_GREEN			        	0x2
#define PSU_LED_MODE_AMBER_BLINKING        		0x3
#define PSU_LED_MODE_AMBER        				0x4
#define PSU_LED_MODE_GREEN_AMBER_BLINKING  		0x5

/* LED_CONTROL */
#define CONTROL_BY_HW                           0x0
#define CONTROL_BY_SW                           0x1

#define IPMI_TIMEOUT				        	(20 * HZ)
#define IPMI_ERR_RETRY_TIMES		        	1

static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");


#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)

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

static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data);
static ssize_t set_led_by_bmc(struct device *dev, struct device_attribute *da, const char *buf, size_t count);
static ssize_t show_led_by_bmc(struct device *dev, struct device_attribute *da, char *buf);
static int extreme7830_32ce_8de_led_probe(struct platform_device *pdev);
static int extreme7830_32ce_8de_led_remove(struct platform_device *pdev);


enum led_data_index {
 	FAN_INDEX = 0,
 	PSU_INDEX,
 	LED_CONTROL_INDEX
};

typedef struct ipmi_user *ipmi_user_t;

struct ipmi_data {
	struct completion   		read_complete;
	struct ipmi_addr    		address;
	ipmi_user_t         		user;
	int                 		interface;

	struct kernel_ipmi_msg 		tx_message;
	long                   		tx_msgid;

	void            			*rx_msg_data;
	unsigned short   			rx_msg_len;
	unsigned char    			rx_result;
	int              			rx_recv_type;

	struct ipmi_user_hndl 		ipmi_hndlrs;
};

struct extreme7830_32ce_8de_led_data {
	struct platform_device 			*pdev;
	struct mutex                    update_lock;
	char                            valid[2];           /* != 0 if registers are valid */
                                                        /* 0: LED Control Register 0, 1: LED Control Register 4 */
	unsigned long                   last_updated[2];    /* In jiffies  0: LED Control Register 0, 1: LED Control Register 4 */
	struct ipmi_data ipmi;
	unsigned char                   ipmi_resp[2];       /* 0: LED Control Register 0, 1: LED Control Register 4 */
	unsigned char                   ipmi_tx_data[4];
};

struct extreme7830_32ce_8de_led_data *data = NULL;

static struct platform_driver extreme7830_32ce_8de_led_driver = {
	.probe      = extreme7830_32ce_8de_led_probe,
	.remove     = extreme7830_32ce_8de_led_remove,
	.driver     = {
		.name   = DRVNAME,
		.owner  = THIS_MODULE,
	},
};

enum extreme7830_32ce_8de_led_sysfs_attrs {
	LED_FAN,
	LED_PSU,
	LED_CONTROL
};


static SENSOR_DEVICE_ATTR(led_fan, S_IWUSR | S_IRUGO, show_led_by_bmc, set_led_by_bmc, LED_FAN);
static SENSOR_DEVICE_ATTR(led_psu, S_IWUSR | S_IRUGO, show_led_by_bmc, set_led_by_bmc, LED_PSU);
static SENSOR_DEVICE_ATTR(led_control, S_IWUSR | S_IRUGO, show_led_by_bmc, set_led_by_bmc, LED_CONTROL);

static struct attribute *extreme7830_32ce_8de_led_attributes[] = {
	&sensor_dev_attr_led_fan.dev_attr.attr,
	&sensor_dev_attr_led_psu.dev_attr.attr,
	&sensor_dev_attr_led_control.dev_attr.attr,
	NULL
};

static const struct attribute_group extreme7830_32ce_8de_led_group = {
	.attrs = extreme7830_32ce_8de_led_attributes,
};

/* Functions to talk to the IPMI layer */

/* Initialize IPMI address, message buffers and user data */
static int init_ipmi_data(struct ipmi_data *ipmi, int iface,
			  struct device *dev)
{
	int err;

	init_completion(&ipmi->read_complete);

	/* Initialize IPMI address */
	ipmi->address.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	ipmi->address.channel = IPMI_BMC_CHANNEL;
	ipmi->address.data[0] = 0;
	ipmi->interface = iface;

	/* Initialize message buffers */
	ipmi->tx_msgid = 0;
	ipmi->tx_message.netfn = IPMI_APP_NETFN;

	ipmi->ipmi_hndlrs.ipmi_recv_hndl = ipmi_msg_handler;

	/* Create IPMI messaging interface user */
	err = ipmi_create_user(ipmi->interface, &ipmi->ipmi_hndlrs,
			       ipmi, &ipmi->user);
	if (err < 0) {
		dev_err(dev, "Unable to register user with IPMI "
			"interface %d\n", ipmi->interface);
		return -EACCES;
	}

	return 0;
}

/* Send an IPMI command */
static int _ipmi_send_message(struct ipmi_data *ipmi, unsigned char cmd,
			     unsigned char *tx_data, unsigned short tx_len,
			     unsigned char *rx_data, unsigned short rx_len)
{
	int err;

	ipmi->tx_message.cmd      = cmd;
	ipmi->tx_message.data     = tx_data;
	ipmi->tx_message.data_len = tx_len;
	ipmi->rx_msg_data         = rx_data;
	ipmi->rx_msg_len          = rx_len;

	err = ipmi_validate_addr(&ipmi->address, sizeof(ipmi->address));
	if (err)
		goto addr_err;

	ipmi->tx_msgid++;
	err = ipmi_request_settime(ipmi->user, &ipmi->address, ipmi->tx_msgid,
				   &ipmi->tx_message, ipmi, 0, 0, 0);
	if (err)
		goto ipmi_req_err;

	err = wait_for_completion_timeout(&ipmi->read_complete, IPMI_TIMEOUT);
	if (!err)
		goto ipmi_timeout_err;

	return 0;

ipmi_timeout_err:
	err = -ETIMEDOUT;
	dev_err(&data->pdev->dev, "request_timeout=%x\n", err);
	return err;
ipmi_req_err:
	dev_err(&data->pdev->dev, "request_settime=%x\n", err);
	return err;
addr_err:
	dev_err(&data->pdev->dev, "validate_addr=%x\n", err);
	return err;
}

/* Send an IPMI command with retry */
static int ipmi_send_message(struct ipmi_data *ipmi, unsigned char cmd,
			     unsigned char *tx_data, unsigned short tx_len,
			     unsigned char *rx_data, unsigned short rx_len)
{
	int status = 0, retry = 0;

	for (retry = 0; retry <= IPMI_ERR_RETRY_TIMES; retry++) {
		status = _ipmi_send_message(ipmi, cmd, tx_data, tx_len,
					    rx_data, rx_len);
		if (unlikely(status != 0)) {
			dev_err(&data->pdev->dev,
				"ipmi_send_message_%d err status(%d)\r\n",
				retry, status);
			continue;
		}

		if (unlikely(ipmi->rx_result != 0)) {
			dev_err(&data->pdev->dev,
				"ipmi_send_message_%d err result(%d)\r\n",
				retry, ipmi->rx_result);
			continue;
		}

		break;
	}

	return status;
}

/* Dispatch IPMI messages to callers */
static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data)
{
	unsigned short 		rx_len;
	struct ipmi_data 	*ipmi = user_msg_data;

	if (msg->msgid != ipmi->tx_msgid) {
		dev_err(&data->pdev->dev, "Mismatch between received msgid "
			"(%02x) and transmitted msgid (%02x)!\n",
			(int)msg->msgid,
			(int)ipmi->tx_msgid);
		ipmi_free_recv_msg(msg);
		return;
	}

	ipmi->rx_recv_type = msg->recv_type;
	if (msg->msg.data_len > 0)
		ipmi->rx_result = msg->msg.data[0];
	else
		ipmi->rx_result = IPMI_UNKNOWN_ERR_COMPLETION_CODE;

	if (msg->msg.data_len > 1) {
		rx_len = msg->msg.data_len - 1;
		if (ipmi->rx_msg_len < rx_len)
			rx_len = ipmi->rx_msg_len;
		ipmi->rx_msg_len = rx_len;
		memcpy(ipmi->rx_msg_data, msg->msg.data + 1, ipmi->rx_msg_len);
	} else {
		ipmi->rx_msg_len = 0;
	}

	ipmi_free_recv_msg(msg);
	complete(&ipmi->read_complete);
}

static struct extreme7830_32ce_8de_led_data *
extreme7830_32ce_8de_led_update_device(struct device_attribute *da)
{
	struct sensor_device_attribute 	*attr = to_sensor_dev_attr(da);
	unsigned char group = attr->index;
	int status = 0;

	switch (attr->index)
	{
		case LED_FAN:
		case LED_PSU:
		case LED_CONTROL:
			group = 0;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	if (time_before(jiffies, data->last_updated[group] + HZ * 5) &&
                data->valid[group])
		return data;

	data->valid[group] = 0;

	/* Get LED status from ipmi */
	data->ipmi_tx_data[0] = IPMI_PWRCPLD_BUS;
	data->ipmi_tx_data[1] = IPMI_PWRCPLD_ADDRESS;
	data->ipmi_tx_data[2] = IPMI_READ_BYTE;

	switch (attr->index)
    {
		case LED_FAN:
		case LED_PSU:
		case LED_CONTROL:
			data->ipmi_tx_data[3] = IPMI_LED_Ctrl_1_OFFSET;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				   data->ipmi_tx_data, 4,
				   &data->ipmi_resp[group],
				   sizeof(data->ipmi_resp[group]));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated[group] = jiffies;
	data->valid[group] = 1;

exit:
	return data;
}

static ssize_t show_led_by_bmc(struct device *dev, struct device_attribute *da,
			char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char group = attr->index;
	int value = 0;
	int error = 0;
	u8 mask = 0;

	mutex_lock(&data->update_lock);

	data = extreme7830_32ce_8de_led_update_device(da);

	switch (attr->index)
	{
		case LED_FAN:
		case LED_PSU:
		case LED_CONTROL:
			group = 0;
			break;
		default:
			error = -EIO;
			goto exit;
	}

	if (!data->valid[group]) {
		error = -EIO;
		goto exit;
	}

    DEBUG_PRINT("extreme7830_32ce_8de_484t show_led_by_bmc: ipmi_resp[%d]:0x%x", group, data->ipmi_resp[group]);

	switch (attr->index) {
		case LED_FAN:
			mask = 0x3 << FAN_LED_BIT_OFFSET;
			value = (data->ipmi_resp[group] & mask) >> FAN_LED_BIT_OFFSET;
			break;
		case LED_PSU:
			mask = 0x7 << PSU_LED_BIT_OFFSET;
			value = (data->ipmi_resp[group] & mask) >> PSU_LED_BIT_OFFSET;
			break;
		case LED_CONTROL:
			mask = 0x1 << LED_CONTROL_BIT_OFFSET;
			value = (data->ipmi_resp[group] & mask) >> LED_CONTROL_BIT_OFFSET;
			break;
	default:
		error = -EINVAL;
		goto exit;
	}

	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", value);

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static ssize_t set_led_by_bmc(struct device *dev, struct device_attribute *da,
		       const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char group = attr->index;
	long mode;
	int error;
	u8 mask = 0;
	int value = 0;

	switch (attr->index)
	{
		case LED_FAN:
		case LED_PSU:
		case LED_CONTROL:
			group = 0;
			break;
		default:
			error = -EIO;
			goto exit;
	}

	error = kstrtol(buf, 10, &mode);
	if (error)
		return error;

	mutex_lock(&data->update_lock);

	data = extreme7830_32ce_8de_led_update_device(da);
	if (!data->valid[group]) {
		error = -EIO;
		goto exit;
	}

	DEBUG_PRINT("set_led_by_bmc: attr->index:%d, data->ipmi_resp[%d]:0x%x", attr->index, group, data->ipmi_resp[group]);

	switch (attr->index) {
		case LED_FAN:
			mask = ~(0x3 << FAN_LED_BIT_OFFSET);
			value = data->ipmi_resp[group] & mask;
			if(mode == FAN_LED_MODE_OFF) {
				data->ipmi_resp[group] = value;
			}
			else if(mode == FAN_LED_MODE_GREEN) {
				mask = FAN_LED_MODE_GREEN << FAN_LED_BIT_OFFSET;
				data->ipmi_resp[group] = value | mask;
			}
			else if(mode == FAN_LED_MODE_AMBER) {
				mask = FAN_LED_MODE_AMBER << FAN_LED_BIT_OFFSET;
				data->ipmi_resp[group] = value | mask;
			}
			else if(mode == FAN_LED_MODE_GREEN_AMBER_BLINKING) {
				mask = FAN_LED_MODE_GREEN_AMBER_BLINKING << FAN_LED_BIT_OFFSET;
				data->ipmi_resp[group] = value | mask;
			}
			else {
				error = -EINVAL;
				goto exit;
			}
			break;

		case LED_PSU:
			mask = ~(0x7 << PSU_LED_BIT_OFFSET);
			value = data->ipmi_resp[group] & mask;
			if(mode == PSU_LED_MODE_OFF) {
				data->ipmi_resp[group] = value;
			}
			else if(mode == PSU_LED_MODE_GREEN) {
				mask = PSU_LED_MODE_GREEN << PSU_LED_BIT_OFFSET;
				data->ipmi_resp[group] = value | mask;
			}
			else if(mode == PSU_LED_MODE_GREEN_BLINKING) {
				mask = PSU_LED_MODE_GREEN_BLINKING << PSU_LED_BIT_OFFSET;
				data->ipmi_resp[group] = value | mask;
			}
			else if(mode == PSU_LED_MODE_AMBER) {
				mask = PSU_LED_MODE_AMBER << PSU_LED_BIT_OFFSET;
				data->ipmi_resp[group] = value | mask;
			}
			else if(mode == PSU_LED_MODE_AMBER_BLINKING) {
				mask = PSU_LED_MODE_AMBER_BLINKING << PSU_LED_BIT_OFFSET;
				data->ipmi_resp[group] = value | mask;
			}
			else if(mode == PSU_LED_MODE_GREEN_AMBER_BLINKING) {
				mask = PSU_LED_MODE_GREEN_AMBER_BLINKING << PSU_LED_BIT_OFFSET;
				data->ipmi_resp[group] = value | mask;
			}
			else {
				error = -EINVAL;
				goto exit;
			}
			break;
		case LED_CONTROL:
			mask = ~(0x1 << LED_CONTROL_BIT_OFFSET);
			value = data->ipmi_resp[group] & mask;
			if(mode == CONTROL_BY_HW) {
				data->ipmi_resp[group] = value;
			}
			else if(mode == CONTROL_BY_SW) {
				mask = CONTROL_BY_SW << LED_CONTROL_BIT_OFFSET;
				data->ipmi_resp[group] = value | mask;
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

	DEBUG_PRINT("set_led_by_bmc: attr->index:%d, mask:0x%x, value:0x%x, data->ipmi_resp[%d]:0x%x", attr->index, mask, value, group, data->ipmi_resp[group]);

	/* Send IPMI write command */
	data->ipmi_tx_data[0] = IPMI_PWRCPLD_BUS;
	data->ipmi_tx_data[1] = IPMI_PWRCPLD_ADDRESS;
	data->ipmi_tx_data[2] = IPMI_READ_BYTE;

	switch (attr->index)
	{
		case LED_FAN:
		case LED_PSU:
		case LED_CONTROL:
			data->ipmi_tx_data[3] = IPMI_LED_Ctrl_1_OFFSET;
			break;
		default:
			error = -EIO;
			goto exit;
	}

	switch (attr->index)
	{
		case LED_FAN:
		case LED_PSU:
			data->ipmi_tx_data[4] = data->ipmi_resp[group] | SW_LED_CONTROL;
			break;
		case LED_CONTROL:
			data->ipmi_tx_data[4] = data->ipmi_resp[group];
			break;
		default:
			break;
	}

	error = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				   data->ipmi_tx_data, 5,
				   NULL, 0);

	if (unlikely(error != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		error = -EIO;
		goto exit;
	}

	error = count;

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static int extreme7830_32ce_8de_led_probe(struct platform_device *pdev)
{
	int status = -1;

	/* Register sysfs hooks */
	status = sysfs_create_group(&pdev->dev.kobj, &extreme7830_32ce_8de_led_group);
	if (status) {
		goto exit;
	}

	dev_info(&pdev->dev, "device created\n");

	return 0;

exit:
	return status;
}

static int extreme7830_32ce_8de_led_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &extreme7830_32ce_8de_led_group);

	return 0;
}

static int __init extreme7830_32ce_8de_led_init(void)
{
	int ret;

	data = kzalloc(sizeof(struct extreme7830_32ce_8de_led_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	mutex_init(&data->update_lock);
	data->valid[0] = 0;
	data->valid[1] = 0;

	ret = platform_driver_register(&extreme7830_32ce_8de_led_driver);
	if (ret < 0)
		goto dri_reg_err;

	data->pdev = platform_device_register_simple(DRVNAME, -1, NULL, 0);
	if (IS_ERR(data->pdev)) {
		ret = PTR_ERR(data->pdev);
		goto dev_reg_err;
	}

	/* Set up IPMI interface */
	ret = init_ipmi_data(&data->ipmi, 0, &data->pdev->dev);
	if (ret)
		goto ipmi_err;

	return 0;

ipmi_err:
	platform_device_unregister(data->pdev);
dev_reg_err:
	platform_driver_unregister(&extreme7830_32ce_8de_led_driver);
dri_reg_err:
	kfree(data);
alloc_err:
	return ret;
}

static void __exit extreme7830_32ce_8de_led_exit(void)
{
	ipmi_destroy_user(data->ipmi.user);
	platform_device_unregister(data->pdev);
	platform_driver_unregister(&extreme7830_32ce_8de_led_driver);
	kfree(data);
}

module_init(extreme7830_32ce_8de_led_init);
module_exit(extreme7830_32ce_8de_led_exit);

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extremenetworks 7830-32ce-8de LED driver");
MODULE_LICENSE("GPL");

