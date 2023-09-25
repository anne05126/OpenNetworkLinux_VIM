/*
 * A FAN IPMI kernel dirver for extremenetworks 8730-32d
 *
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

#define DRVNAME                                 "8730_fan"
#define IPMI_SENSOR_NETFN                       0x34

#define IPMI_SENSOR_READ_CMD 		            0xe
#define IPMI_RPM_READ_CMD 		            	0xf
#define IPMI_FAULT_READ_CMD 		            0x10
#define IPMI_PWM_READ_CMD 		                0x4
#define IPMI_PWM_WRITE_CMD 		                0x5
#define IPMI_EEPROM_READ_CMD 		            0x2

#define IPMI_TIMEOUT                            (20 * HZ)
#define IPMI_ERR_RETRY_TIMES                    1

#define IPMI_SENSOR_OFFSET_FAN1                 0x1
#define IPMI_SENSOR_OFFSET_FAN2                 0x2
#define IPMI_SENSOR_OFFSET_FAN3                 0x3
#define IPMI_SENSOR_OFFSET_FAN4                 0x4
#define IPMI_SENSOR_OFFSET_FAN5                 0x5
#define IPMI_SENSOR_OFFSET_FAN6                 0x6
#define IPMI_SENSOR_OFFSET_FAN7                 0x7

#define EEPROM_SIZE      						256

static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");


#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)


static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data);
static ssize_t show_fan(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t show_fan_speed(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t show_fan_pwm(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t show_fan_eeprom(struct device *dev, struct device_attribute *da, char *buf);
static int extreme8730_32d_fan_probe(struct platform_device *pdev);
static int extreme8730_32d_fan_remove(struct platform_device *pdev);

enum fan_id {
	FAN_1,
	FAN_2,
	FAN_3,
	FAN_4,
	FAN_5,
	FAN_6,
	FAN_7,
	NUM_OF_FAN
};

enum fan_data_index {
	FAN_PRESENT = 0,
	FAN_DIR,
	FAN_FAULT,
	FAN_SPEED,
	FAN_PWM,
	FAN_EEPROM,
	FAN_DATA_COUNT
};

enum fan_byte_index {
	FAN_BYTE0 = 0,
	FAN_BYTE1,
	FAN_BYTE2,
	FAN_BYTE3,
	FAN_BYTE_COUNT
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

struct ipmi_fan_resp_data {
    unsigned char   status[4];	/* 4 bytes for each fan. 0: present, 1: air flow */
	unsigned char   rpm[4];	/* 4 bytes for each fan. */
	unsigned char   eeprom[EEPROM_SIZE];	/* 256 bytes for each fan. */
};

struct extreme8730_32d_fan_data {
	struct platform_device 			*pdev;
	struct mutex                    update_lock;
	char             				fan_speed_valid[7];         /* 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, 4: FAN5, 5: FAN6, 6: FAN7 */
	char             				fan_pwm_valid[7];           /* 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, 4: FAN5, 5: FAN6, 6: FAN7 */
	char             				fan_eeprom_valid[7];        /* 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, 4: FAN5, 5: FAN6, 6: FAN7 */
	char                            valid[7];                   /* != 0 if registers are valid,
                                                                   0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, 4: FAN5, 5: FAN6, 6: FAN7 */
	unsigned long                   last_updated[7];            /* In jiffies 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, 4: FAN5, 5: FAN6, 6: FAN7 */
	unsigned long                   last_updated_eeprom[7];     /* In jiffies 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, 4: FAN5, 5: FAN6, 6: FAN7 */
	struct ipmi_data                ipmi;
	struct ipmi_fan_resp_data   	ipmi_resp_fan[7];    		/* 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, 4: FAN5, 5: FAN6, 6: FAN7 */
	unsigned char					ipmi_resp_fan_fault[7];		/* 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, 4: FAN5, 5: FAN6, 6: FAN7 */
	unsigned char                   ipmi_resp_fan_pwm[7];       /* 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, 4: FAN5, 5: FAN6, 6: FAN7 */
	unsigned char                   ipmi_tx_data[4];
};

struct extreme8730_32d_fan_data *data = NULL;

static struct platform_driver extreme8730_32d_fan_driver = {
	.probe      = extreme8730_32d_fan_probe,
	.remove     = extreme8730_32d_fan_remove,
	.driver     = {
		.name   = DRVNAME,
		.owner  = THIS_MODULE,
	},
};

#define FAN_PRESENT_ATTR_ID(index)		FAN##index##_PRESENT
#define FAN_DIR_ATTR_ID(index)			FAN##index##_DIR
#define FAN_FAULT_ATTR_ID(index)		FAN##index##_FAULT
#define FAN_RPM_ATTR_ID(index)			FAN##index##_INPUT
#define FAN_PWM_ATTR_ID(index)			FAN##index##_PWM
#define FAN_EEPROM_ATTR_ID(index)		FAN##index##_EEPROM

#define FAN_ATTR(fan_id) \
	FAN_PRESENT_ATTR_ID(fan_id),    	\
	FAN_DIR_ATTR_ID(fan_id),        	\
	FAN_FAULT_ATTR_ID(fan_id),    		\
	FAN_RPM_ATTR_ID(fan_id),			\
	FAN_PWM_ATTR_ID(fan_id),			\
	FAN_EEPROM_ATTR_ID(fan_id)

enum extreme8730_32d_fan_sysfs_attrs {
	/* fan attributes */
	FAN_ATTR(1),
	FAN_ATTR(2),
	FAN_ATTR(3),
	FAN_ATTR(4),
	FAN_ATTR(5),
	FAN_ATTR(6),
	FAN_ATTR(7),
	NUM_OF_FAN_ATTR,
	NUM_OF_PER_FAN_ATTR = (NUM_OF_FAN_ATTR/NUM_OF_FAN)
};

/* fan attributes */
#define DECLARE_FAN_SENSOR_DEVICE_ATTR(index) \
	static SENSOR_DEVICE_ATTR(fan##index##_present, S_IRUGO, show_fan, NULL, \
                  FAN##index##_PRESENT); \
    static SENSOR_DEVICE_ATTR(fan##index##_dir, S_IRUGO, show_fan, NULL, \
				  FAN##index##_DIR); \
	static SENSOR_DEVICE_ATTR(fan##index##_fault, S_IRUGO, show_fan, NULL, \
				  FAN##index##_FAULT); \
	static SENSOR_DEVICE_ATTR(fan##index##_input, S_IRUGO, show_fan_speed, NULL, \
				  FAN##index##_INPUT); \
	static SENSOR_DEVICE_ATTR(fan##index##_pwm, S_IRUGO, \
				  show_fan_pwm, NULL, FAN##index##_PWM); \
	static SENSOR_DEVICE_ATTR(fan##index##_eeprom, S_IRUGO, \
				  show_fan_eeprom, NULL, FAN##index##_EEPROM)
                  
                  
#define DECLARE_FAN_ATTR(index) \
	&sensor_dev_attr_fan##index##_present.dev_attr.attr, \
	&sensor_dev_attr_fan##index##_dir.dev_attr.attr, \
	&sensor_dev_attr_fan##index##_fault.dev_attr.attr, \
	&sensor_dev_attr_fan##index##_input.dev_attr.attr, \
	&sensor_dev_attr_fan##index##_pwm.dev_attr.attr, \
	&sensor_dev_attr_fan##index##_eeprom.dev_attr.attr

DECLARE_FAN_SENSOR_DEVICE_ATTR(1);
DECLARE_FAN_SENSOR_DEVICE_ATTR(2);
DECLARE_FAN_SENSOR_DEVICE_ATTR(3);
DECLARE_FAN_SENSOR_DEVICE_ATTR(4);
DECLARE_FAN_SENSOR_DEVICE_ATTR(5);
DECLARE_FAN_SENSOR_DEVICE_ATTR(6);
DECLARE_FAN_SENSOR_DEVICE_ATTR(7);


static struct attribute *extreme8730_32d_fan_attributes[] = {
	/* fan attributes */
	DECLARE_FAN_ATTR(1),
	DECLARE_FAN_ATTR(2),
	DECLARE_FAN_ATTR(3),
	DECLARE_FAN_ATTR(4),
	DECLARE_FAN_ATTR(5),
	DECLARE_FAN_ATTR(6),
	DECLARE_FAN_ATTR(7),
	NULL
};

static const struct attribute_group extreme8730_32d_fan_group = {
	.attrs = extreme8730_32d_fan_attributes,
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

static struct extreme8730_32d_fan_data *
extreme8730_32d_fan_update_device(struct device_attribute *da)
{
	struct sensor_device_attribute 	*attr = to_sensor_dev_attr(da);
	unsigned char                   fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int status = 0;

	if (time_before(jiffies, data->last_updated[fid] + HZ * 5) && 
                data->valid[fid])
		return data;

	data->valid[fid] = 0;

	switch (fid) 
	{
		case FAN_1:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN1;
			break;
		case FAN_2:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN2;
			break;
		case FAN_3:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN3;
			break;
		case FAN_4:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN4;
			break;
		case FAN_5:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN5;
			break;
		case FAN_6:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN6;
			break;
		case FAN_7:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN7;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	/* Get FAN status from ipmi */
	status = ipmi_send_message(&data->ipmi, IPMI_SENSOR_READ_CMD,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp_fan[fid].status,
				   sizeof(data->ipmi_resp_fan[fid].status));
	
	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	/* Get FAN fault from ipmi */
	status = ipmi_send_message(&data->ipmi, IPMI_FAULT_READ_CMD,
				   data->ipmi_tx_data, 1,
				   &data->ipmi_resp_fan_fault[fid],
				   sizeof(data->ipmi_resp_fan_fault[fid]));
	
	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated[fid] = jiffies;
	data->valid[fid] = 1;

exit:
	return data;
}

static struct extreme8730_32d_fan_data *
extreme8730_32d_fan_update_fan_speed(struct device_attribute *da)
{
	struct sensor_device_attribute 	*attr = to_sensor_dev_attr(da);
	unsigned char                   fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int status = 0;

	data->fan_speed_valid[fid] = 0;

	/* Get FAN SPEED from ipmi */
	switch (fid) 
	{
		case FAN_1:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN1;
			break;
		case FAN_2:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN2;
			break;
		case FAN_3:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN3;
			break;
		case FAN_4:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN4;
			break;
		case FAN_5:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN5;
			break;
		case FAN_6:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN6;
			break;
		case FAN_7:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN7;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_RPM_READ_CMD,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp_fan[fid].rpm,
				   sizeof(data->ipmi_resp_fan[fid].rpm));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->fan_speed_valid[fid] = 1;
exit:
	return data;
}

static struct extreme8730_32d_fan_data *
extreme8730_32d_fan_update_fan_pwm(struct device_attribute *da)
{
	struct sensor_device_attribute 	*attr = to_sensor_dev_attr(da);
	unsigned char                   fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int status = 0;

	data->fan_pwm_valid[fid] = 0;

	/* Get FAN SPEED from ipmi */
	switch (fid) 
	{
		case FAN_1:
		case FAN_2:
		case FAN_3:
		case FAN_4:
		case FAN_5:
		case FAN_6:
		case FAN_7:
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_PWM_READ_CMD,
				   data->ipmi_tx_data, 0,
				   &data->ipmi_resp_fan_pwm[fid],
				   sizeof(data->ipmi_resp_fan_pwm[fid]));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->fan_pwm_valid[fid] = 1;
exit:
	return data;
}

static struct extreme8730_32d_fan_data *
extreme8730_32d_fan_update_fan_eeprom(struct device_attribute *da)
{
	struct sensor_device_attribute 	*attr = to_sensor_dev_attr(da);
	unsigned char                   fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int status = 0;

	if (time_before(jiffies, data->last_updated_eeprom[fid] + HZ * 5) && data->fan_eeprom_valid[fid]) {
		return data;
	}

	data->fan_eeprom_valid[fid] = 0;

	/* Get FAN EEPROM from ipmi */
	switch (fid) 
	{
		case FAN_1:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN1;
			break;
		case FAN_2:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN2;
			break;
		case FAN_3:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN3;
			break;
		case FAN_4:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN4;
			break;
		case FAN_5:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN5;
			break;
		case FAN_6:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN6;
			break;
		case FAN_7:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN7;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	/* Get eeprom from ipmi */
	status = ipmi_send_message(&data->ipmi, IPMI_EEPROM_READ_CMD,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp_fan[fid].eeprom,
				   sizeof(data->ipmi_resp_fan[fid].eeprom));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated_eeprom[fid] = jiffies;
	data->fan_eeprom_valid[fid] = 1;

exit:
	return data;
}

#define VALIDATE_PRESENT_RETURN(id) \
do { \
	if (present == 0) { \
		mutex_unlock(&data->update_lock);   \
		return -ENXIO; \
	} \
} while (0)

static ssize_t show_fan(struct device *dev, struct device_attribute *da,
			char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int value = 0;

	int present = 0;
	int error = 0;

	mutex_lock(&data->update_lock);

	data = extreme8730_32d_fan_update_device(da);
	if (!data->valid[fid]) {
		error = -EIO;
		goto exit;
	}

	present = data->ipmi_resp_fan[fid].status[FAN_PRESENT];

	switch (attr->index) {
	case FAN1_PRESENT:
	case FAN2_PRESENT:
	case FAN3_PRESENT:
	case FAN4_PRESENT:
	case FAN5_PRESENT:
	case FAN6_PRESENT:
	case FAN7_PRESENT:
		value = present;
		break;
	case FAN1_DIR:
	case FAN2_DIR:
	case FAN3_DIR:
	case FAN4_DIR:
	case FAN5_DIR:
	case FAN6_DIR:
	case FAN7_DIR:
		VALIDATE_PRESENT_RETURN(fid);
		value = data->ipmi_resp_fan[fid].status[FAN_DIR];
		break;
	case FAN1_FAULT:
	case FAN2_FAULT:
	case FAN3_FAULT:
	case FAN4_FAULT:
	case FAN5_FAULT:
	case FAN6_FAULT:
	case FAN7_FAULT:
		VALIDATE_PRESENT_RETURN(fid);
		value = data->ipmi_resp_fan_fault[fid];
		break;
	default:
		error = -EINVAL;
		goto exit;
	}

	DEBUG_PRINT("8730_32d_fan show_fan: fid:%d, attr:%d, value:%d \n", fid, attr->index, value);

	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", present ? value : 0);

exit:
	mutex_unlock(&data->update_lock);
	return error;    
}

static ssize_t show_fan_speed(struct device *dev, 
                struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int value = 0;
	int error = 0;

	mutex_lock(&data->update_lock);

	data = extreme8730_32d_fan_update_fan_speed(da);
	if (!data->fan_speed_valid[fid]) {
		error = -EIO;
		goto exit;
	}

	switch (attr->index) {
	case FAN1_INPUT:
	case FAN2_INPUT:
	case FAN3_INPUT:
	case FAN4_INPUT:
	case FAN5_INPUT:
	case FAN6_INPUT:
	case FAN7_INPUT:
		value = ((int)data->ipmi_resp_fan[fid].rpm[FAN_BYTE0] | 
				(int)data->ipmi_resp_fan[fid].rpm[FAN_BYTE1] << 8 |
				(int)data->ipmi_resp_fan[fid].rpm[FAN_BYTE2] << 16 |
				(int)data->ipmi_resp_fan[fid].rpm[FAN_BYTE3] << 32);
		break;
	default:
		error = -EINVAL;
		goto exit;
	}

	DEBUG_PRINT("8730_32d_fan show_fan_speed: fid:%d, attr:%d, value:%d \n", fid, attr->index, value);

	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", value);

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static ssize_t show_fan_pwm(struct device *dev, 
                struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int value = 0;
	int error = 0;

	mutex_lock(&data->update_lock);

	data = extreme8730_32d_fan_update_fan_pwm(da);
	if (!data->fan_pwm_valid[fid]) {
		error = -EIO;
		goto exit;
	}

	switch (attr->index) {
	case FAN1_PWM:
	case FAN2_PWM:
	case FAN3_PWM:
	case FAN4_PWM:
	case FAN5_PWM:
	case FAN6_PWM:
	case FAN7_PWM:
		value = data->ipmi_resp_fan_pwm[fid];
		break;
	default:
		error = -EINVAL;
		goto exit;
	}

	DEBUG_PRINT("8730_32d_fan show_fan_pwm: fid:%d, attr:%d, value:%d \n", fid, attr->index, value);

	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", value);

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static ssize_t show_fan_eeprom(struct device *dev, 
                struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int error = 0;
	int count = 0;

	mutex_lock(&data->update_lock);

	data = extreme8730_32d_fan_update_fan_eeprom(da);
	if (!data->fan_eeprom_valid[fid]) {
		error = -EIO;
		goto exit;
	}
	
	switch (attr->index) {
		case FAN1_EEPROM:
		case FAN2_EEPROM:
		case FAN3_EEPROM:
		case FAN4_EEPROM:
		case FAN5_EEPROM:
		case FAN6_EEPROM:
		case FAN7_EEPROM:
			memcpy(buf, data->ipmi_resp_fan[fid].eeprom, sizeof(data->ipmi_resp_fan[fid].eeprom));
			count = sizeof(data->ipmi_resp_fan[fid].eeprom);
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	mutex_unlock(&data->update_lock);

	DEBUG_PRINT("\n 8730_32d_fan show_fan_eeprom: fid:%d, attr:%d, count:%d \n", fid, attr->index, count);

	return count;

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static int extreme8730_32d_fan_probe(struct platform_device *pdev)
{
	int status = -1;

	/* Register sysfs hooks */
	status = sysfs_create_group(&pdev->dev.kobj, &extreme8730_32d_fan_group);
	if (status) {
		goto exit;
	}
    
	dev_info(&pdev->dev, "device created\n");

	return 0;

exit:
	return status;
}

static int extreme8730_32d_fan_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &extreme8730_32d_fan_group);

	return 0;
}

static int __init extreme8730_32d_fan_init(void)
{
	int ret;

	data = kzalloc(sizeof(struct extreme8730_32d_fan_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	mutex_init(&data->update_lock);
	data->valid[0] = 0;
	data->valid[1] = 0;
	data->valid[2] = 0;
	data->valid[3] = 0;
	data->valid[4] = 0;
	data->valid[5] = 0;
	data->valid[6] = 0;

	ret = platform_driver_register(&extreme8730_32d_fan_driver);
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
	platform_driver_unregister(&extreme8730_32d_fan_driver);
dri_reg_err:
	kfree(data);
alloc_err:
	return ret;
}

static void __exit extreme8730_32d_fan_exit(void)
{
	ipmi_destroy_user(data->ipmi.user);
	platform_device_unregister(data->pdev);
	platform_driver_unregister(&extreme8730_32d_fan_driver);
	kfree(data);
}

module_init(extreme8730_32d_fan_init);
module_exit(extreme8730_32d_fan_exit);

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extremenetworks 8730-32d FAN driver");
MODULE_LICENSE("GPL");

