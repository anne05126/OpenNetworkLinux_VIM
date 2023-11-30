/*
 * A Power CPLD IPMI kernel dirver for alphanetworks scg60d0-484t
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

#define DRVNAME 					"7830_pwr_cpld"

#define IPMI_APP_NETFN							0x6
#define IPMI_READ_WRITE_CMD						0x52
#define IPMI_PCPLD_BUS							0x09
#define IPMI_PCPLD_ADDRESS						0xbc	/* 0x51 << 1 = 0xbc */
#define IPMI_READ_BYTE							0x01 	/* Read */

#define IPMI_PCPLD_VER_OFFSET		        	0x00

#define IPMI_TIMEOUT				(20 * HZ)
#define IPMI_ERR_RETRY_TIMES		1

#define CPLD_VERSION_BITS_MASK    	0x7


static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");


#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)


static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data);
static ssize_t show_cpld_version(struct device *dev, struct device_attribute *da, char *buf);
static int extreme7830_pwr_cpld_probe(struct platform_device *pdev);
static int extreme7830_pwr_cpld_remove(struct platform_device *pdev);

typedef struct ipmi_user *ipmi_user_t;

struct ipmi_data {
	struct completion   	read_complete;
	struct ipmi_addr		address;
	ipmi_user_t         	user;
	int                 	interface;

	struct kernel_ipmi_msg 	tx_message;
	long                   	tx_msgid;

	void            		*rx_msg_data;
	unsigned short   		rx_msg_len;
	unsigned char    		rx_result;
	int              		rx_recv_type;

	struct ipmi_user_hndl 	ipmi_hndlrs;
};

struct extreme7830_pwr_cpld_data {
	struct platform_device 			*pdev;
	struct mutex 					update_lock;
	char             				cpld_ver_valid;      	/* != 0 if registers are valid */
	struct ipmi_data 				ipmi;
	unsigned char    				ipmi_resp_cpld;         /* PWR CPLD */
	unsigned char 					ipmi_tx_data[4];
};

struct extreme7830_pwr_cpld_data *data = NULL;

static struct platform_driver extreme7830_pwr_cpld_driver = {
	.probe      = extreme7830_pwr_cpld_probe,
	.remove     = extreme7830_pwr_cpld_remove,
	.driver     = {
		.name   = DRVNAME,
		.owner  = THIS_MODULE,
	},
};

enum extreme7830_pwr_cpld_sysfs_attrs {
	PWR_CPLD_VER,
};


static SENSOR_DEVICE_ATTR(pwr_cpld_ver, S_IRUGO, show_cpld_version, NULL, PWR_CPLD_VER);

static struct attribute *extreme7830_pwr_cpld_attributes[] = {
	/* pwr cpld attributes */
	&sensor_dev_attr_pwr_cpld_ver.dev_attr.attr,
	NULL

};

static const struct attribute_group extreme7830_pwr_cpld_group = {
	.attrs = extreme7830_pwr_cpld_attributes,
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

static struct extreme7830_pwr_cpld_data *
extreme7830_pwr_cpld_update_cpld_ver(void)
{
	int status = 0;

	data->cpld_ver_valid = 0;

	data->ipmi_tx_data[0] = IPMI_PCPLD_BUS;
	data->ipmi_tx_data[1] = IPMI_PCPLD_ADDRESS;
	data->ipmi_tx_data[2] = IPMI_READ_BYTE;
	data->ipmi_tx_data[3] = IPMI_PCPLD_VER_OFFSET;

	status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				   data->ipmi_tx_data, 4,
				   &data->ipmi_resp_cpld, 
				   sizeof(data->ipmi_resp_cpld));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->cpld_ver_valid = 1;
exit:
	return data;
}

static ssize_t show_cpld_version(struct device *dev, 
                struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char value = 0;
	int error = 0;

	switch (attr->index) {
	case PWR_CPLD_VER:
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&data->update_lock);

	data = extreme7830_pwr_cpld_update_cpld_ver();
	if (!data->cpld_ver_valid) {
		error = -EIO;
		goto exit;
	}

	value = data->ipmi_resp_cpld & CPLD_VERSION_BITS_MASK;
	mutex_unlock(&data->update_lock);
	
	DEBUG_PRINT("7830_32ce_8de show_cpld_version: data->ipmi_resp_cpld:0x%x", data->ipmi_resp_cpld);
				
	return sprintf(buf, "%d\n", value);

exit:
	mutex_unlock(&data->update_lock);
	return error;    
}

static int extreme7830_pwr_cpld_probe(struct platform_device *pdev)
{
	int status = -1;

	/* Register sysfs hooks */
	status = sysfs_create_group(&pdev->dev.kobj, &extreme7830_pwr_cpld_group);
	if (status) {
		goto exit;
	}

	dev_info(&pdev->dev, "device created\n");

	return 0;

exit:
	return status;
}

static int extreme7830_pwr_cpld_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &extreme7830_pwr_cpld_group);
	return 0;
}

static int __init extreme7830_pwr_cpld_init(void)
{
	int ret;

	data = kzalloc(sizeof(struct extreme7830_pwr_cpld_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	mutex_init(&data->update_lock);

	data->cpld_ver_valid = 0;

	ret = platform_driver_register(&extreme7830_pwr_cpld_driver);
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
	platform_driver_unregister(&extreme7830_pwr_cpld_driver);
dri_reg_err:
	kfree(data);
alloc_err:
	return ret;
}

static void __exit extreme7830_pwr_cpld_exit(void)
{
	ipmi_destroy_user(data->ipmi.user);
	platform_device_unregister(data->pdev);
	platform_driver_unregister(&extreme7830_pwr_cpld_driver);
	kfree(data);
}

module_init(extreme7830_pwr_cpld_init);
module_exit(extreme7830_pwr_cpld_exit);

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extremenetworks 7830-32ce-8de Power CPLD driver");
MODULE_LICENSE("GPL");
