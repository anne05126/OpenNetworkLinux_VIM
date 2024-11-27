/*
 * A VIM EEPROM IPMI kernel dirver for extremenetworks 7830-32ce-8de
 *
 * Copyright (C) 2023 Alphanetworks Technology Corporation.
 * Ruby Wang <Ru-xin_Wang@alphanetworks.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/ipmi.h>
#include <linux/ipmi_smi.h>
#include <linux/platform_device.h>

#define DRVNAME "7830_vim_eeprom"


/* Get Transceiver EEPROM */
#define IPMI_SENSOR_NETFN				0x34
#define IPMI_VIM_EEPROM_READ_CMD		0x00

#define IPMI_OEM_OFFSET_VIM1			0x01
#define IPMI_OEM_OFFSET_VIM2			0x02

#define IPMI_EEPROM_READ_OFFSET			0x01
#define IPMI_DOM_READ_OFFSET			0x02

#define IPMI_TIMEOUT				(20 * HZ)
#define IPMI_ERR_RETRY_TIMES		1

#define EEPROM_SIZE      			256

#define DEBUG_MODE 1

static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");

#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)

static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data);
static ssize_t show_eeprom(struct device *dev, struct device_attribute *attr, char *buf);
static int extreme7830_32ce_8de_vim_eeprom_probe(struct platform_device *pdev);
static int extreme7830_32ce_8de_vim_eeprom_remove(struct platform_device *pdev);

typedef struct ipmi_user *ipmi_user_t;

struct ipmi_data {
	struct completion	read_complete;
	struct ipmi_addr	address;
	ipmi_user_t 		user;
	int 				interface;

	struct kernel_ipmi_msg tx_message;
	long				   tx_msgid;

	void			*rx_msg_data;
	unsigned short	 rx_msg_len;
	unsigned char	 rx_result;
	int 			 rx_recv_type;

	struct ipmi_user_hndl ipmi_hndlrs;
};

struct ipmi_vim_eeprom_resp_data {
	unsigned char   eeprom[EEPROM_SIZE];
};

enum vim_id
{
    VIM_1, 
    VIM_2,
    VIM_ID_MAX
};


struct extreme7830_32ce_8de_vim_eeprom_data {
	struct platform_device *pdev;
	struct mutex	 update_lock;
	char			 valid[2]; 					/* 0: VIM1, 1: VIM2 */
	unsigned long	 last_updated[2];	 		/* In jiffies, 0: VIM1, 1: VIM2 */
	struct ipmi_data ipmi;
	struct ipmi_vim_eeprom_resp_data ipmi_resp[2]; 	/* 0: VIM1, 1: VIM2 */
	unsigned char ipmi_tx_data[5];
};

struct extreme7830_32ce_8de_vim_eeprom_data *data = NULL;

static struct platform_driver extreme7830_32ce_8de_vim_eeprom_driver = {
	.probe		= extreme7830_32ce_8de_vim_eeprom_probe,
	.remove 	= extreme7830_32ce_8de_vim_eeprom_remove,
	.driver 	= {
		.name	= DRVNAME,
		.owner	= THIS_MODULE,
	},
};

#define VIM_EEPROM_ATTR(vim_id) 		VIM##vim_id##_EEPROM

enum extreme7830_32ce_8de_vim_eeprom_sysfs_attrs {
	/* VIM attributes */
	VIM_EEPROM_ATTR(1), 
	VIM_EEPROM_ATTR(2)
};

/* VIM eeprom attributes */
#define DECLARE_VIM_EEPROM_SENSOR_DEVICE_ATTR(vim_id) \
		static SENSOR_DEVICE_ATTR(vim##vim_id##_eeprom, S_IRUGO, show_eeprom,	 NULL, VIM##vim_id##_EEPROM); \

#define DECLARE_VIM_EEPROM_ATTR(vim_id) \
		&sensor_dev_attr_vim##vim_id##_eeprom.dev_attr.attr

		
DECLARE_VIM_EEPROM_SENSOR_DEVICE_ATTR(1);
DECLARE_VIM_EEPROM_SENSOR_DEVICE_ATTR(2);

static struct attribute *extreme7830_32ce_8de_vim_eeprom_attributes[] = {
	/* vim_eeprom attributes */
	DECLARE_VIM_EEPROM_ATTR(1),
	DECLARE_VIM_EEPROM_ATTR(2),
	NULL
};

static const struct attribute_group extreme7830_32ce_8de_vim_eeprom_group = {
	.attrs = extreme7830_32ce_8de_vim_eeprom_attributes,
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
	ipmi->tx_message.netfn = IPMI_SENSOR_NETFN;

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
static int ipmi_send_message(struct ipmi_data *ipmi, unsigned char cmd,
									 unsigned char *tx_data, unsigned short tx_len,
									 unsigned char *rx_data, unsigned short rx_len)
{
	int err;

	ipmi->tx_message.cmd	  = cmd;
	ipmi->tx_message.data	  = tx_data;
	ipmi->tx_message.data_len = tx_len;
	ipmi->rx_msg_data		  = rx_data;
	ipmi->rx_msg_len		  = rx_len;

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

/* Dispatch IPMI messages to callers */
static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data)
{
	unsigned short rx_len;
	struct ipmi_data *ipmi = user_msg_data;

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
	} else
		ipmi->rx_msg_len = 0;

	ipmi_free_recv_msg(msg);
	complete(&ipmi->read_complete);
}

#define VALIDATE_PRESENT_RETURN(id) \
do { \
	if (vim_present == 1) { \
		mutex_unlock(&data->update_lock);   \
		return -ENXIO; \
	} \
} while (0)

static int read_vim_present_from_sysfs(int vim_id)
{
    struct file *f;
    char buf[16];
	char path[128];  /* Used to store formatted paths */
    int present;
	loff_t pos = 0;

	/* Open the formatted sysfs file */
	snprintf(path, sizeof(path), "/sys/bus/i2c/devices/0-006e/vim_%d_present", vim_id+1);
    f = filp_open(path, O_RDONLY, 0);
    if (IS_ERR(f)) {
        pr_err("Failed to open sysfs file: %s\n", path);
        return -EIO;
    }

	/* Read data from file */
    if (kernel_read(f, buf, sizeof(buf) - 1, &pos) < 0) {
        pr_err("Failed to read from sysfs file: %s\n", path);
        filp_close(f, NULL);
        return -EIO;
    }
    buf[sizeof(buf) - 1] = '\0';
    present = simple_strtol(buf, NULL, 10);

	/* Close the file and restore the memory segment */
    filp_close(f, NULL);

    return present;
}

static struct extreme7830_32ce_8de_vim_eeprom_data *extreme7830_32ce_8de_vim_eeprom_update_eeprom(struct device_attribute *da)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int status = 0;

	if (time_before(jiffies, data->last_updated[attr->index] + HZ * 5) && data->valid[attr->index]) {
		return data;
	}

	data->valid[attr->index] = 0;

	data->ipmi.tx_message.netfn = IPMI_SENSOR_NETFN;

	/* Get VIM EEPROM - Read EEPROM information */
	switch (attr->index) 
    {
		case VIM1_EEPROM:
			data->ipmi_tx_data[0] = IPMI_OEM_OFFSET_VIM1;
			break;
		case VIM2_EEPROM:
			data->ipmi_tx_data[0] = IPMI_OEM_OFFSET_VIM2;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_VIM_EEPROM_READ_CMD,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp[attr->index].eeprom,
				   sizeof(data->ipmi_resp[attr->index].eeprom));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated[attr->index] = jiffies;
	data->valid[attr->index] = 1;

exit:
	return data;
}

static ssize_t show_eeprom(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char vid = attr->index % VIM_ID_MAX;
	int error = 0;
	int count = 0;
	int vim_present = 0;

	mutex_lock(&data->update_lock);

	switch (vid) {
		case VIM_1:
			vim_present = read_vim_present_from_sysfs(VIM_1);
			break;
		case VIM_2:
			vim_present = read_vim_present_from_sysfs(VIM_2);
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	DEBUG_PRINT("7830_32ce_8de show_vim_board_id: vim%d_present=%d", vid+1, vim_present);
	
	switch (attr->index) {
		case VIM1_EEPROM:
		case VIM2_EEPROM:
			VALIDATE_PRESENT_RETURN(vid);
			data = extreme7830_32ce_8de_vim_eeprom_update_eeprom(da);
			if (!data->valid[attr->index]) {
				error = -EIO;
				goto exit;
			}

			memcpy(buf, data->ipmi_resp[attr->index].eeprom, sizeof(data->ipmi_resp[attr->index].eeprom));
			count = sizeof(data->ipmi_resp[attr->index].eeprom);
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	mutex_unlock(&data->update_lock);

	DEBUG_PRINT("\n Show_eeprom: attr->index:%d, attr:%d, count:%d \n", attr->index, attr->index, count);

	return count;

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static int extreme7830_32ce_8de_vim_eeprom_probe(struct platform_device *pdev)
{
	int status = -1;

	/* Register sysfs hooks */	
	status = sysfs_create_group(&pdev->dev.kobj, &extreme7830_32ce_8de_vim_eeprom_group);
	if (status) {
		goto exit;
	}


	dev_info(&pdev->dev, "device created\n");

	return 0;

exit:
	return status;
}

static int extreme7830_32ce_8de_vim_eeprom_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &extreme7830_32ce_8de_vim_eeprom_group);
	return 0;
}

static int __init extreme7830_32ce_8de_vim_eeprom_init(void)
{
	int ret;

	data = kzalloc(sizeof(struct extreme7830_32ce_8de_vim_eeprom_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	mutex_init(&data->update_lock);
	data->valid[0] = 0;
	data->valid[1] = 0;

	ret = platform_driver_register(&extreme7830_32ce_8de_vim_eeprom_driver);
	if (ret < 0) {
		goto dri_reg_err;
	}

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
	platform_driver_unregister(&extreme7830_32ce_8de_vim_eeprom_driver);
dri_reg_err:
	kfree(data);
alloc_err:
	return ret;
}

static void __exit extreme7830_32ce_8de_vim_eeprom_exit(void)
{
	ipmi_destroy_user(data->ipmi.user);
	platform_device_unregister(data->pdev);
	platform_driver_unregister(&extreme7830_32ce_8de_vim_eeprom_driver);
	kfree(data);
}

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extremenetworks 7830-32ce-8de VIM EEPROM driver");
MODULE_LICENSE("GPL");

module_init(extreme7830_32ce_8de_vim_eeprom_init);
module_exit(extreme7830_32ce_8de_vim_eeprom_exit);
