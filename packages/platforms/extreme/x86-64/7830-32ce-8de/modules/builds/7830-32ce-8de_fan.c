/*
 * A FAN IPMI kernel dirver for extremenetworks 7830-32ce-8de
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

#define DRVNAME                                 	"7830_fan"

/* Base on Power CPLD spec v10 */
/* PRESENT, Air Flow Direction, PWM */
#define IPMI_APP_NETFN								0x6
#define IPMI_READ_WRITE_CMD							0x52
#define IPMI_PWRCPLD_BUS							0x09
#define IPMI_PWRCPLD_ADDRESS						0xbc	/* 0x5e << 1 = 0xbc */
#define IPMI_READ_BYTE								0x01 	/* Read */

#define IPMI_SENSOR_OFFSET_FAN_PRESENT          	0x61
#define IPMI_SENSOR_OFFSET_FAN_DIR             		0x63
#define IPMI_SENSOR_OFFSET_FAN_PWM           		0x65

#define IPMI_SENSOR_FAN1_BIT						0x3
#define IPMI_SENSOR_FAN2_BIT						0x2
#define IPMI_SENSOR_FAN3_BIT						0x1
#define IPMI_SENSOR_FAN4_BIT						0x0


/* RPM */
#define IPMI_SENSOR_NETFN                           0x4
#define IPMI_SENSOR_READ_CMD 		                0x2d
#define IPMI_FACTORS_READ_CMD 						0x23
#define IPMI_FACTORS_READING_BYTE					0x00

#define IPMI_SENSOR_READING_OFFSET_TEMP_CPU			0x74
#define IPMI_SENSOR_READING_OFFSET_TEMP_TMP75_0		0x60

#define IPMI_SENSOR_OFFSET_FAN1_INLET				0x30
#define IPMI_SENSOR_OFFSET_FAN1_OUTLET				0x31
#define IPMI_SENSOR_OFFSET_FAN2_INLET				0x32
#define IPMI_SENSOR_OFFSET_FAN2_OUTLET				0x33
#define IPMI_SENSOR_OFFSET_FAN3_INLET				0x34
#define IPMI_SENSOR_OFFSET_FAN3_OUTLET				0x35
#define IPMI_SENSOR_OFFSET_FAN4_INLET				0x36
#define IPMI_SENSOR_OFFSET_FAN4_OUTLET				0x37

#define IPMI_TIMEOUT                            	(20 * HZ)
#define IPMI_ERR_RETRY_TIMES                    	1

static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");


#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)


static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data);
static ssize_t show_fan(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t show_fan_speed(struct device *dev, struct device_attribute *da, char *buf);
static int extreme7830_32ce_8de_fan_probe(struct platform_device *pdev);
static int extreme7830_32ce_8de_fan_remove(struct platform_device *pdev);

enum fan_id {
	FAN_1,
	FAN_2,
	FAN_3,
	FAN_4,
	NUM_OF_FAN
};

enum fan_data_index {
	FAN_PRESENT = 0,
	FAN_DIR,
	FAN_FAULT,
	FAN_SPEED,
	FAN_PWM,
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
	unsigned char   rpm_inlet[4];				/* 4 bytes for each fan. */
	unsigned char   rpm_outlet[4];				/* 4 bytes for each fan. */
	unsigned char   rpm_inlet_factors[7];		/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
	unsigned char	rpm_outlet_factors[7];		/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
};

struct extreme7830_32ce_8de_fan_data {
	struct platform_device 			*pdev;
	struct mutex                    update_lock;
	char             				fan_speed_valid[4];         /* 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, */
	char                            valid[4];                   /* != 0 if registers are valid,
                                                                   0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, */
	unsigned long                   last_updated[4];            /* In jiffies 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, */
	struct ipmi_data                ipmi;
	unsigned char   				ipmi_resp_present;			/* 1 byte for each fan. 0: not present, 1: present */
	unsigned char   				ipmi_resp_dir;				/* 1 byte for each fan. 0: Front to back, 1: back to front */
	struct ipmi_fan_resp_data   	ipmi_resp_fan[4];    		/* 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, */
	unsigned char                   ipmi_resp_fan_pwm;       	/* 0: FAN1, 1: FAN2, 2: FAN3, 3: FAN4, */
	unsigned char                   ipmi_tx_data[4];
};

struct extreme7830_32ce_8de_fan_data *data = NULL;

static struct platform_driver extreme7830_32ce_8de_fan_driver = {
	.probe      = extreme7830_32ce_8de_fan_probe,
	.remove     = extreme7830_32ce_8de_fan_remove,
	.driver     = {
		.name   = DRVNAME,
		.owner  = THIS_MODULE,
	},
};

#define FAN_PRESENT_ATTR_ID(index)		FAN##index##_PRESENT
#define FAN_DIR_ATTR_ID(index)			FAN##index##_DIR
#define FAN_RPM_INLET_ATTR_ID(index)	FAN##index##_INPUT_INLET
#define FAN_RPM_OUTLET_ATTR_ID(index)	FAN##index##_INPUT_OUTLET
#define FAN_PWM_ATTR_ID(index)			FAN##index##_PWM

#define FAN_ATTR(fan_id) \
	FAN_PRESENT_ATTR_ID(fan_id),    	\
	FAN_DIR_ATTR_ID(fan_id),        	\
	FAN_RPM_INLET_ATTR_ID(fan_id),    	\
	FAN_RPM_OUTLET_ATTR_ID(fan_id),		\
	FAN_PWM_ATTR_ID(fan_id)

enum extreme7830_32ce_8de_fan_sysfs_attrs {
	/* fan attributes */
	FAN_ATTR(1),
	FAN_ATTR(2),
	FAN_ATTR(3),
	FAN_ATTR(4),
	NUM_OF_FAN_ATTR,
	NUM_OF_PER_FAN_ATTR = (NUM_OF_FAN_ATTR/NUM_OF_FAN)
};

/* fan attributes */
#define DECLARE_FAN_SENSOR_DEVICE_ATTR(index) \
	static SENSOR_DEVICE_ATTR(fan##index##_present, S_IRUGO, show_fan, NULL, \
                  FAN##index##_PRESENT); \
    static SENSOR_DEVICE_ATTR(fan##index##_dir, S_IRUGO, show_fan, NULL, \
				  FAN##index##_DIR); \
	static SENSOR_DEVICE_ATTR(fan##index##_input_inlet, S_IRUGO, show_fan_speed, NULL, \
				  FAN##index##_INPUT_INLET); \
	static SENSOR_DEVICE_ATTR(fan##index##_input_outlet, S_IRUGO, show_fan_speed, NULL, \
				  FAN##index##_INPUT_OUTLET); \
	static SENSOR_DEVICE_ATTR(fan##index##_pwm, S_IRUGO, show_fan_speed, NULL, \
				  FAN##index##_PWM)

#define DECLARE_FAN_ATTR(index) \
	&sensor_dev_attr_fan##index##_present.dev_attr.attr, \
	&sensor_dev_attr_fan##index##_dir.dev_attr.attr, \
	&sensor_dev_attr_fan##index##_input_inlet.dev_attr.attr, \
	&sensor_dev_attr_fan##index##_input_outlet.dev_attr.attr, \
	&sensor_dev_attr_fan##index##_pwm.dev_attr.attr


DECLARE_FAN_SENSOR_DEVICE_ATTR(1);
DECLARE_FAN_SENSOR_DEVICE_ATTR(2);
DECLARE_FAN_SENSOR_DEVICE_ATTR(3);
DECLARE_FAN_SENSOR_DEVICE_ATTR(4);


static struct attribute *extreme7830_32ce_8de_fan_attributes[] = {
	/* fan attributes */
	DECLARE_FAN_ATTR(1),
	DECLARE_FAN_ATTR(2),
	DECLARE_FAN_ATTR(3),
	DECLARE_FAN_ATTR(4),
	NULL
};

static const struct attribute_group extreme7830_32ce_8de_fan_group = {
	.attrs = extreme7830_32ce_8de_fan_attributes,
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

static struct extreme7830_32ce_8de_fan_data *
extreme7830_32ce_8de_fan_update_device(struct device_attribute *da)
{
	struct sensor_device_attribute 	*attr = to_sensor_dev_attr(da);
	unsigned char                   fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int status = 0;

	if (time_before(jiffies, data->last_updated[fid] + HZ * 5) && 
                data->valid[fid])
		return data;

	data->valid[fid] = 0;

	/* Get FAN present from ipmi */
	/* ipmitool raw 0x06 0x52 0x09 0xbc 0x1 0x61 */
	data->ipmi.tx_message.netfn = IPMI_APP_NETFN;
	data->ipmi_tx_data[0] = IPMI_PWRCPLD_BUS;
	data->ipmi_tx_data[1] = IPMI_PWRCPLD_ADDRESS;
	data->ipmi_tx_data[2] = IPMI_READ_BYTE;

	switch (fid) 
	{
		case FAN_1:
		case FAN_2:
		case FAN_3:
		case FAN_4:
			data->ipmi_tx_data[3] = IPMI_SENSOR_OFFSET_FAN_PRESENT;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				   data->ipmi_tx_data, 4,
				   &data->ipmi_resp_present,
				   sizeof(data->ipmi_resp_present));
	
	DEBUG_PRINT("Get FAN present: present len:%ld \n", sizeof(data->ipmi_resp_present));

	if (unlikely(status != 0)){
		DEBUG_PRINT("Get FAN present failed: status != 0\n");
		goto exit;
	}
	if (unlikely(data->ipmi.rx_result != 0)) {
		DEBUG_PRINT("Get FAN present failed: data->ipmi.rx_result != 0\n");
		status = -EIO;
		goto exit;
	}

	/* Get FAN dir from ipmi */
	/* ipmitool raw 0x06 0x52 0x09 0xbc 0x1 0x63 */
	data->ipmi.tx_message.netfn = IPMI_APP_NETFN;
	data->ipmi_tx_data[0] = IPMI_PWRCPLD_BUS;
	data->ipmi_tx_data[1] = IPMI_PWRCPLD_ADDRESS;
	data->ipmi_tx_data[2] = IPMI_READ_BYTE;

	switch (fid) 
	{
		case FAN_1:
		case FAN_2:
		case FAN_3:
		case FAN_4:
			data->ipmi_tx_data[3] = IPMI_SENSOR_OFFSET_FAN_DIR;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				   data->ipmi_tx_data, 4,
				   &data->ipmi_resp_dir,
				   sizeof(data->ipmi_resp_dir));
	
	DEBUG_PRINT("Get FAN dir: dir len:%ld \n", sizeof(data->ipmi_resp_dir));

	if (unlikely(status != 0)){
		DEBUG_PRINT("Get FAN dir failed: status != 0\n");
		goto exit;
	}
	if (unlikely(data->ipmi.rx_result != 0)) {
		DEBUG_PRINT("Get FAN dir failed: data->ipmi.rx_result != 0\n");
		status = -EIO;
		goto exit;
	}

	data->valid[fid] = 1;
	data->last_updated[fid] = jiffies;

exit:
	return data;
}

static struct extreme7830_32ce_8de_fan_data *
extreme7830_32ce_8de_fan_update_fan_speed(struct device_attribute *da)
{
	struct sensor_device_attribute 	*attr = to_sensor_dev_attr(da);
	unsigned char                   fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int status = 0;

	data->fan_speed_valid[fid] = 0;

	/* Get FAN SPEED (inlet) from ipmi */
	/*  
		ipmitool raw 0x4 0x2d 0x30
		ipmitool raw 0x4 0x2d 0x32
		ipmitool raw 0x4 0x2d 0x34
		ipmitool raw 0x4 0x2d 0x36
	*/
	data->ipmi.tx_message.netfn = IPMI_SENSOR_NETFN;

	switch (fid) 
	{
		case FAN_1:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN1_INLET;
			break;
		case FAN_2:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN2_INLET;
			break;
		case FAN_3:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN3_INLET;
			break;
		case FAN_4:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN4_INLET;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_SENSOR_READ_CMD ,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp_fan[fid].rpm_inlet, 
				   sizeof(data->ipmi_resp_fan[fid].rpm_inlet));

	if (unlikely(status != 0)){
		DEBUG_PRINT("Get FAN SPEED (inlet) failed: status != 0\n");
		goto exit;
	}
	if (unlikely(data->ipmi.rx_result != 0)) {
		DEBUG_PRINT("Get FAN SPEED (inlet) failed: data->ipmi.rx_result != 0\n");
		status = -EIO;
		goto exit;
	}

	/* Get FAN SPEED (outlet) from ipmi */
	/*  
		ipmitool raw 0x4 0x2d 0x31
		ipmitool raw 0x4 0x2d 0x33
		ipmitool raw 0x4 0x2d 0x35
		ipmitool raw 0x4 0x2d 0x37
	*/
	data->ipmi.tx_message.netfn = IPMI_SENSOR_NETFN;

	switch (fid) 
	{
		case FAN_1:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN1_OUTLET;
			break;
		case FAN_2:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN2_OUTLET;
			break;
		case FAN_3:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN3_OUTLET;
			break;
		case FAN_4:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN4_OUTLET;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_SENSOR_READ_CMD ,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp_fan[fid].rpm_outlet, 
				   sizeof(data->ipmi_resp_fan[fid].rpm_outlet));

	if (unlikely(status != 0)){
		DEBUG_PRINT("Get FAN SPEED (outlet) failed: status != 0\n");
		goto exit;
	}
	if (unlikely(data->ipmi.rx_result != 0)) {
		DEBUG_PRINT("Get FAN SPEED (outlet) failed: data->ipmi.rx_result != 0\n");
		status = -EIO;
		goto exit;
	}

	/* Get FAN SPEED (inlet factors) from ipmi */
	/*  
		ipmitool raw 0x4 0x23 0x30 0x0
		ipmitool raw 0x4 0x23 0x32 0x0
		ipmitool raw 0x4 0x23 0x34 0x0
		ipmitool raw 0x4 0x23 0x36 0x0
	*/
	data->ipmi.tx_message.netfn = IPMI_SENSOR_NETFN;

	switch (fid) 
	{
		case FAN_1:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN1_INLET;
			break;
		case FAN_2:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN2_INLET;
			break;
		case FAN_3:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN3_INLET;
			break;
		case FAN_4:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN4_INLET;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	data->ipmi_tx_data[1] = IPMI_FACTORS_READING_BYTE;
	status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD ,
				   data->ipmi_tx_data, 2,
				   data->ipmi_resp_fan[fid].rpm_inlet_factors, 
				   sizeof(data->ipmi_resp_fan[fid].rpm_inlet_factors));

	
	/* Get FAN SPEED (outlet factors) from ipmi */
	/*  
		ipmitool raw 0x4 0x23 0x31 0x0
		ipmitool raw 0x4 0x23 0x33 0x0
		ipmitool raw 0x4 0x23 0x35 0x0
		ipmitool raw 0x4 0x23 0x37 0x0
	*/
	data->ipmi.tx_message.netfn = IPMI_SENSOR_NETFN;

	switch (fid) 
	{
		case FAN_1:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN1_OUTLET;
			break;
		case FAN_2:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN2_OUTLET;
			break;
		case FAN_3:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN3_OUTLET;
			break;
		case FAN_4:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_FAN4_OUTLET;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	data->ipmi_tx_data[1] = IPMI_FACTORS_READING_BYTE;
	status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD ,
				   data->ipmi_tx_data, 2,
				   data->ipmi_resp_fan[fid].rpm_outlet_factors, 
				   sizeof(data->ipmi_resp_fan[fid].rpm_outlet_factors));

	if (unlikely(status != 0)){
		DEBUG_PRINT("Get FAN SPEED (factors) failed: status != 0\n");
		goto exit;
	}
	if (unlikely(data->ipmi.rx_result != 0)) {
		DEBUG_PRINT("Get FAN SPEED (factors) failed: data->ipmi.rx_result != 0\n");
		status = -EIO;
		goto exit;
	}

	/* Get FAN PWM from ipmi */
	/* ipmitool raw 0x06 0x52 0x09 0xbc 0x1 0x65 */
	data->ipmi.tx_message.netfn = IPMI_APP_NETFN;
	data->ipmi_tx_data[0] = IPMI_PWRCPLD_BUS;
	data->ipmi_tx_data[1] = IPMI_PWRCPLD_ADDRESS;
	data->ipmi_tx_data[2] = IPMI_READ_BYTE;

	switch (fid) 
	{
		case FAN_1:
		case FAN_2:
		case FAN_3:
		case FAN_4:
			data->ipmi_tx_data[3] = IPMI_SENSOR_OFFSET_FAN_PWM;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				   data->ipmi_tx_data, 4,
				   &data->ipmi_resp_fan_pwm,
				   sizeof(data->ipmi_resp_fan_pwm));
	
	DEBUG_PRINT("Get FAN PWM: PWM len:%ld \n", sizeof(data->ipmi_resp_fan_pwm));

	if (unlikely(status != 0)){
		DEBUG_PRINT("Get FAN PWM failed: status != 0\n");
		goto exit;
	}
	if (unlikely(data->ipmi.rx_result != 0)) {
		DEBUG_PRINT("Get FAN PWM failed: data->ipmi.rx_result != 0\n");
		status = -EIO;
		goto exit;
	}

	data->fan_speed_valid[fid] = 1;


exit:
	return data;
}

#define VALIDATE_PRESENT_RETURN(id) \
do { \
	if (present == 1) { \
		mutex_unlock(&data->update_lock);   \
		return -ENXIO; \
	} \
} while (0)

static u16 M_factors_get(u16 M_LS, u16 M_MS_raw)
{
	u16 M = 0;
	u16 M_MS = 0;
	
	M_MS = (M_MS_raw & 0xc0) << 2;
	M = M_MS | M_LS;

    return M;
}

static u16 B_factors_get(u16 B_LS, u16 B_MS_raw)
{
	u16 B = 0;
	u16 B_MS = 0;
	
	B_MS = (B_MS_raw & 0xc0) << 2;
	B = (B_MS | B_LS) & 0x3ff;

    return B;
}

static int two_complement_to_int(u16 data, u8 valid_bit, int mask)
{
    u16 valid_data = data & mask;
    bool is_negative = valid_data >> (valid_bit - 1);

    return is_negative ? (-(((~valid_data) & mask) + 1)) : valid_data;
}

static int exponentInt(const int base, int n)
{
    int i, p = base;

	if(n == 0)
		return 1;
		
    for (i = 1; i < n; ++i)
        p *= base;
	
    return p;
}

static int result_convert(int x, u8 fid, u8 attr)
{
	/* 
	 * IPMI Section 35.5, 36.3 
	 * y = L[(Mx + (B * 10 ^ K1)) * 10 ^ K2] units 
	 * where:
	 * x - Raw reading (get sensor reading CMD response byte 2)
	 * y - Converted reading
	 * L[] - Linearization function specified by 'linearization type'
	 * M - Signed integer constant multiplier (get sensor reading factors CMD response byte 3, 4[7:6])
	 * B - Signed additive 'offset' (get sensor reading factors CMD response byte 5, 6[7:6])
	 * K1 - Signed Exponent(B). (get sensor reading factors CMD response byte 8[3:0])
	 * K2 - Signed Result Exponent(R). (get sensor reading factors CMD response byte 8[7:4])
	 */

	u16 M = 0;
	u16 B = 0;
	int K1 = 0;
	int K2 = 0;
	int multiplier = 0; 
	int exponent1 = 0;
	int exponent2 = 0;
	int y1 = 0;
	int y2 = 0;
	int value = 0;

	switch (attr) {
			case FAN1_INPUT_INLET:
			case FAN2_INPUT_INLET:
			case FAN3_INPUT_INLET:
			case FAN4_INPUT_INLET:
				M = M_factors_get(data->ipmi_resp_fan[fid].rpm_inlet_factors[1], data->ipmi_resp_fan[fid].rpm_inlet_factors[2]);
				B = B_factors_get(data->ipmi_resp_fan[fid].rpm_inlet_factors[3], data->ipmi_resp_fan[fid].rpm_inlet_factors[4]);
				K1 = two_complement_to_int(data->ipmi_resp_fan[fid].rpm_inlet_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_resp_fan[fid].rpm_inlet_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_fan result_convert: x:%d, fid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, fid, attr, M, B, K1, K2);
				break;
			case FAN1_INPUT_OUTLET:
			case FAN2_INPUT_OUTLET:
			case FAN3_INPUT_OUTLET:
			case FAN4_INPUT_OUTLET:
				M = M_factors_get(data->ipmi_resp_fan[fid].rpm_outlet_factors[1], data->ipmi_resp_fan[fid].rpm_outlet_factors[2]);
				B = B_factors_get(data->ipmi_resp_fan[fid].rpm_outlet_factors[3], data->ipmi_resp_fan[fid].rpm_outlet_factors[4]);
				K1 = two_complement_to_int(data->ipmi_resp_fan[fid].rpm_outlet_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_resp_fan[fid].rpm_outlet_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_fan result_convert: x:%d, fid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, fid, attr, M, B, K1, K2);
				break;
			
	}

	exponent1 = K2 + multiplier;
	exponent2 = K1 + K2 + multiplier;

	if(exponent1 >= 0)
		y1 = M * x * exponentInt(10, exponent1);
	else
		y1 = M * x / exponentInt(10, -exponent1);

	if(exponent2 >= 0)
		y2 = B * exponentInt(10, exponent2);
	else
		y2 = B / exponentInt(10, -exponent2);

	/* y = (M * x + (B * 10 ^ K1)) * 10 ^ K2 */
	/* value = (M * x * exponentInt(10, K2 + multiplier)) + (B * exponentInt(10, K1 + K2 + multiplier)) */
	value = y1 + y2;

	DEBUG_PRINT("7830_32ce_8de_fan result_convert: value:%d \n", value);

	return value;
}


static ssize_t show_fan(struct device *dev, struct device_attribute *da,
			char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char fid = attr->index / NUM_OF_PER_FAN_ATTR;
	int value = 0;

	int present = 0;
	int error = 0;
	u8 mask = 0;

	mutex_lock(&data->update_lock);

	data = extreme7830_32ce_8de_fan_update_device(da);
	if (!data->valid[fid]) {
		DEBUG_PRINT("show_fan failed: data->valid[fid]=1\n");
		error = -EIO;
		goto exit;
	}

	switch (fid) 
	{
		case FAN_1:
			mask = 0x1 << IPMI_SENSOR_FAN1_BIT;
			break;
		case FAN_2:
			mask = 0x1 << IPMI_SENSOR_FAN2_BIT;
			break;
		case FAN_3:
			mask = 0x1 << IPMI_SENSOR_FAN3_BIT;
			break;
		case FAN_4:
			mask = 0x1 << IPMI_SENSOR_FAN4_BIT;
			break;
		default:
			error = -EIO;
			goto exit;
	}
	present = !!(data->ipmi_resp_present & mask);   /* 0 = Not Present, 1 = Present */

	switch (attr->index) {
		case FAN1_PRESENT:
		case FAN2_PRESENT:
		case FAN3_PRESENT:
		case FAN4_PRESENT:
			value = present;
			break;
		case FAN1_DIR:
			VALIDATE_PRESENT_RETURN(fid);
			value = (data->ipmi_resp_dir & mask) >> IPMI_SENSOR_FAN1_BIT;
			break;
		case FAN2_DIR:
			VALIDATE_PRESENT_RETURN(fid);
			value = (data->ipmi_resp_dir & mask) >> IPMI_SENSOR_FAN2_BIT;
			break;
		case FAN3_DIR:
			VALIDATE_PRESENT_RETURN(fid);
			value = (data->ipmi_resp_dir & mask) >> IPMI_SENSOR_FAN3_BIT;
			break;
		case FAN4_DIR:
			VALIDATE_PRESENT_RETURN(fid);
			value = (data->ipmi_resp_dir & mask) >> IPMI_SENSOR_FAN4_BIT;
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	DEBUG_PRINT("7830_32ce_8de_fan show_fan: fid:%d, attr:%d, value:%d \n", fid, attr->index, value);

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
	int present = 0;
	u8 mask = 0;

	mutex_lock(&data->update_lock);

	/* get present value */
	data = extreme7830_32ce_8de_fan_update_device(da);
	if (!data->valid[fid]) {
		DEBUG_PRINT("show_fan failed: data->valid[fid]=1\n");
		error = -EIO;
		goto exit;
	}

	switch (fid) 
	{
		case FAN_1:
			mask = 0x1 << IPMI_SENSOR_FAN1_BIT;
			break;
		case FAN_2:
			mask = 0x1 << IPMI_SENSOR_FAN2_BIT;
			break;
		case FAN_3:
			mask = 0x1 << IPMI_SENSOR_FAN3_BIT;
			break;
		case FAN_4:
			mask = 0x1 << IPMI_SENSOR_FAN4_BIT;
			break;
		default:
			error = -EIO;
			goto exit;
	}
	present = !!(data->ipmi_resp_present & mask);   /* 0 = Not Present, 1 = Present */


	data = extreme7830_32ce_8de_fan_update_fan_speed(da);
	if (!data->fan_speed_valid[fid]) {
		error = -EIO;
		goto exit;
	}

	/* fan RPM and PWM */
	switch (attr->index) {
		case FAN1_INPUT_INLET:
		case FAN2_INPUT_INLET:
		case FAN3_INPUT_INLET:
		case FAN4_INPUT_INLET:
			VALIDATE_PRESENT_RETURN(fid);
			value = result_convert(data->ipmi_resp_fan[fid].rpm_inlet[0], fid, attr->index);
			DEBUG_PRINT("7830_32ce_8de_fan show_fan_speed: fid:%d, attr:%d, convert_value:%d, x=%d \n", fid, attr->index, value, data->ipmi_resp_fan[fid].rpm_inlet[0]);
			break;
		case FAN1_INPUT_OUTLET:
		case FAN2_INPUT_OUTLET:
		case FAN3_INPUT_OUTLET:
		case FAN4_INPUT_OUTLET:
			VALIDATE_PRESENT_RETURN(fid);
			value = result_convert(data->ipmi_resp_fan[fid].rpm_outlet[0], fid, attr->index);
			DEBUG_PRINT("7830_32ce_8de_fan show_fan_speed: fid:%d, attr:%d, convert_value:%d, x=%d \n", fid, attr->index, value, data->ipmi_resp_fan[fid].rpm_outlet[0]);
			break;
		case FAN1_PWM:
		case FAN2_PWM:
		case FAN3_PWM:
		case FAN4_PWM:
			value = data->ipmi_resp_fan_pwm;
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	DEBUG_PRINT("7830_32ce_8de_fan show_fan_speed: fid:%d, attr:%d, value:%d \n", fid, attr->index, value);

	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", value);

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static int extreme7830_32ce_8de_fan_probe(struct platform_device *pdev)
{
	int status = -1;

	/* Register sysfs hooks */
	status = sysfs_create_group(&pdev->dev.kobj, &extreme7830_32ce_8de_fan_group);
	if (status) {
		goto exit;
	}
    
	dev_info(&pdev->dev, "device created\n");

	return 0;

exit:
	return status;
}

static int extreme7830_32ce_8de_fan_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &extreme7830_32ce_8de_fan_group);

	return 0;
}

static int __init extreme7830_32ce_8de_fan_init(void)
{
	int ret;

	data = kzalloc(sizeof(struct extreme7830_32ce_8de_fan_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	mutex_init(&data->update_lock);
	data->valid[0] = 0;
	data->valid[1] = 0;
	data->valid[2] = 0;
	data->valid[3] = 0;
	data->fan_speed_valid[0] = 0;
	data->fan_speed_valid[1] = 0;
	data->fan_speed_valid[2] = 0;
	data->fan_speed_valid[3] = 0;

	ret = platform_driver_register(&extreme7830_32ce_8de_fan_driver);
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
	platform_driver_unregister(&extreme7830_32ce_8de_fan_driver);
dri_reg_err:
	kfree(data);
alloc_err:
	return ret;
}

static void __exit extreme7830_32ce_8de_fan_exit(void)
{
	ipmi_destroy_user(data->ipmi.user);
	platform_device_unregister(data->pdev);
	platform_driver_unregister(&extreme7830_32ce_8de_fan_driver);
	kfree(data);
}

module_init(extreme7830_32ce_8de_fan_init);
module_exit(extreme7830_32ce_8de_fan_exit);

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extremenetworks 7830-32ce-8de FAN driver");
MODULE_LICENSE("GPL");

