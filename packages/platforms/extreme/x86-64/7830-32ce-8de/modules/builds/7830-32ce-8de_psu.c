/*
 * An hwmon driver for extremenetworks 7830-32ce-8de AcBel Power Module
 *
 * Copyright (C) 2023 Alphanetworks Technology Corporation.
 * Ruby Wang <Ru-xin_Wang@alphanetworks.com>
 *
 * Based on:
 * Copyright (C) 2023 Alphanetworks Technology Corporation.
 * Anne Liou <anne_liou@alphanetworks.com>
 *
 * Based on:
 * Copyright (C) 2014 Accton Technology Corporation.
 * Brandon Chuang <brandon_chuang@accton.com.tw>
 *
 * Based on ad7414.c
 * Copyright 2006 Stefan Roese <sr at denx.de>, DENX Software Engineering
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

#define DRVNAME 								"7830_psu"

/* Base on Power CPLD spec v10 */
/* Get PSU Status */
#define IPMI_APP_NETFN							0x6
#define IPMI_READ_WRITE_CMD						0x52
#define IPMI_PWRCPLD_BUS						0x09
#define IPMI_PWRCPLD_ADDRESS					0xbc	/* 0x5e << 1 = 0xbc */
#define IPMI_READ_BYTE							0x01 	/* Read */
#define IPMI_SENSOR_OFFSET_PSU1_STAT          	0x11
#define IPMI_SENSOR_OFFSET_PSU2_STAT         	0x12

/* Get PSU Power, Temperature and PSU FAN RPM */
#define IPMI_SENSOR_NETFN           			0x4
#define IPMI_SENSOR_READ_CMD 					0x2d
#define IPMI_FACTORS_READ_CMD 					0x23
#define IPMI_FACTORS_READING_BYTE				0x00

#define SENSOR_PS1_IN_VOL						0xa0
#define SENSOR_PS2_IN_VOL						0xa1
#define SENSOR_PS1_OUT_VOL						0xa2
#define SENSOR_PS2_OUT_VOL						0xa3
#define SENSOR_PS1_IN_CUR						0xa4
#define SENSOR_PS2_IN_CUR						0xa5
#define SENSOR_PS1_OUT_CUR						0xa6
#define SENSOR_PS2_OUT_CUR						0xa7
#define SENSOR_PS1_IN_PWR						0xa8
#define SENSOR_PS2_IN_PWR						0xa9
#define SENSOR_PS1_OUT_PWR						0xaa
#define SENSOR_PS2_OUT_PWR						0xab
#define SENSOR_PS1_TEMP1						0x63
#define SENSOR_PS1_TEMP2						0x64
#define SENSOR_PS1_TEMP3						0x65
#define SENSOR_PS2_TEMP1						0x66
#define SENSOR_PS2_TEMP2						0x67
#define SENSOR_PS2_TEMP3						0x68
#define SENSOR_PS1_FAN1_PWM						0x38
#define SENSOR_PS2_FAN1_PWM						0x39

/* PMBus Protocol. */
#define IPMI_I2C_BUS_PSU1_MICRO_CTRLER			0x0
#define IPMI_I2C_CHAN_PSU1_MICRO_CTRLER			0x2
#define IPMI_I2C_BUS_PSU2_MICRO_CTRLER			0x1
#define IPMI_I2C_CHAN_PSU2_MICRO_CTRLER			0x2

#define IPMI_ENCODE_BUS_ID(_chan, _bus)			(((_chan) << 4) | ((_bus) << 1))

#define IPMI_PSU1_CTRL_ADDR  					0xb0	/* 0x58 << 1 */
#define IPMI_PSU2_CTRL_ADDR  					0xb2	/* 0x59 << 1 */

#define PSU_REG_MFR_MODEL           			0x9A
#define PSU_REG_MFR_SERIAL          			0x9E

#define IPMI_TIMEOUT							(20 * HZ)
#define IPMI_ERR_RETRY_TIMES					1

static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");

#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)

static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data);
static ssize_t show_psu_status(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_psu(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_string(struct device *dev, struct device_attribute *attr, char *buf);
static int extreme7830_32ce_8de_psu_probe(struct platform_device *pdev);
static int extreme7830_32ce_8de_psu_remove(struct platform_device *pdev);

enum psu_id {
	PSU_1,
	PSU_2,
	NUM_OF_PSU
};

enum psu_status_byte_index {
	PSU_PRESENT = 0,
	PSU_POWER_GOOD_CPLD
};

enum psu_power_value_byte_index {
    VIN_BYTE0,
	VIN_BYTE1,
	VIN_BYTE2,
	VIN_BYTE3,
    VOUT_BYTE0,
    VOUT_BYTE1,
    VOUT_BYTE2,
    VOUT_BYTE3,
    IIN_BYTE0,
    IIN_BYTE1,
    IIN_BYTE2,
    IIN_BYTE3,
    IOUT_BYTE0,
    IOUT_BYTE1,
    IOUT_BYTE2,
    IOUT_BYTE3,
    PIN_BYTE0,
    PIN_BYTE1,
    PIN_BYTE2,
    PIN_BYTE3,
    POUT_BYTE0,
    POUT_BYTE1,
    POUT_BYTE2,
    POUT_BYTE3
};

enum psu_temp_byte_index {
    TEMP0_BYTE0,
	TEMP0_BYTE1,
	TEMP0_BYTE2,
	TEMP0_BYTE3,
    TEMP1_BYTE0,
    TEMP1_BYTE1,
    TEMP1_BYTE2,
    TEMP1_BYTE3,
    TEMP2_BYTE0,
    TEMP2_BYTE1,
    TEMP2_BYTE2,
    TEMP2_BYTE3
};

enum psu_fan_byte_index {
    FAN0_BYTE0,
	FAN0_BYTE1,
	FAN0_BYTE2,
	FAN0_BYTE3,
    FAN1_BYTE0,
    FAN1_BYTE1,
    FAN1_BYTE2,
    FAN1_BYTE3
};

enum psu_fru_len_index {
    VENDOR_LEN,
	MODEL_LEN,
	SERIAL_LEN,
	NUM_OF_LEN
};

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

struct ipmi_psu_factors_data {
	unsigned char	ipmi_vin_factors[7];	/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
	unsigned char	ipmi_vout_factors[7];	/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
	unsigned char	ipmi_iin_factors[7];	/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
	unsigned char	ipmi_iout_factors[7];	/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
	unsigned char	ipmi_pin_factors[7];	/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
	unsigned char	ipmi_pout_factors[7];	/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
	unsigned char	ipmi_temp1_factors[7];	/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
	unsigned char	ipmi_temp2_factors[7];	/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
	unsigned char	ipmi_temp3_factors[7];	/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
	unsigned char	ipmi_fan1_factors[7];	/* 0: not use, 1: M-LS 8bits, 2: M-MS 2bits, 3: B-LS 8bits, 4: B-MS 2bits, 5: not use, 6: [7:4]R exponent(K2) [3:0]B exponent(K1) */
};

struct ipmi_psu_resp_data {
	char   serial[18];
	char   model[14];
};

struct ipmi_psu_resp_data_value{
	unsigned char	ipmi_resp_raw[3];	/* 0: raw value, 1: not use, 2: not use  */
};

struct extreme7830_32ce_8de_psu_data {
	struct platform_device *pdev;
	struct mutex	 update_lock;
	char			 valid[2]; /* 0: PSU1, 1: PSU2 */
	char			 valid_status[2]; /* != 0 if registers are valid */
							    	  /* 0: PSU1, 1: PSU2 */
	char			 valid_value[28];
	unsigned long	 last_updated[2];	 /* In jiffies, 0: PSU1, 1: PSU2 */
	unsigned long	 last_updated_status[2]; /* In jiffies, 0: PSU1, 1: PSU2 */
	unsigned long	 last_updated_value[28]; /* In jiffies */
	struct ipmi_data ipmi;
	struct ipmi_psu_factors_data ipmi_factors[2]; /* 0: PSU1, 1: PSU2 */
	struct ipmi_psu_resp_data ipmi_resp[2]; /* 0: PSU1, 1: PSU2 */
	struct ipmi_psu_resp_data_value ipmi_resp_value[28]; /* 0: PSU1_VIN, 1: PSU1_VOUT, 2: PSU1_IIN, 3: PSU1_IOUT, 4: PSU1_PIN, 5: PSU1_POUT, 6: PSU1_MODEL, 7: PSU1_SERIAL, 8: PSU1_TEMP1, 9: PSU1_TEMP2, 10: PSU1_TEMP3, 11: PSU1_FAN1, 12: PSU1_PRESENT, 13: PSU1_POWERGOOD */
							    	  					 /* 14: PSU2_VIN, 15: PSU2_VOUT, 16: PSU2_IIN, 17: PSU2_IOUT, 18: PSU2_PIN, 19: PSU2_POUT, 20: PSU2_MODEL, 21: PSU2_SERIAL, 22: PSU2_TEMP1, 23: PSU2_TEMP2, 24: PSU2_TEMP3, 25: PSU2_FAN1, 26: PSU1_PRESENT, 27: PSU1_POWERGOOD */
	unsigned char ipmi_resp_status[2]; /* 0: PSU1, 1: PSU2 */
	unsigned char ipmi_tx_data[5];
};

struct extreme7830_32ce_8de_psu_data *data = NULL;

static struct platform_driver extreme7830_32ce_8de_psu_driver = {
	.probe		= extreme7830_32ce_8de_psu_probe,
	.remove 	= extreme7830_32ce_8de_psu_remove,
	.driver 	= {
		.name	= DRVNAME,
		.owner	= THIS_MODULE,
	},
};

#define PSU_VIN_ATTR_ID(index)          PSU##index##_VIN
#define PSU_VOUT_ATTR_ID(index)         PSU##index##_VOUT
#define PSU_IIN_ATTR_ID(index)         	PSU##index##_IIN
#define PSU_IOUT_ATTR_ID(index)         PSU##index##_IOUT
#define PSU_PIN_ATTR_ID(index)          PSU##index##_PIN
#define PSU_POUT_ATTR_ID(index)         PSU##index##_POUT
#define PSU_MODEL_ATTR_ID(index)        PSU##index##_MODEL
#define PSU_SERIAL_ATTR_ID(index)       PSU##index##_SERIAL
#define PSU_TEMP1_INPUT_ATTR_ID(index)  PSU##index##_TEMP1_INPUT
#define PSU_TEMP2_INPUT_ATTR_ID(index)  PSU##index##_TEMP2_INPUT
#define PSU_TEMP3_INPUT_ATTR_ID(index)  PSU##index##_TEMP3_INPUT
#define PSU_FAN1_INPUT_ATTR_ID(index)   PSU##index##_FAN1_INPUT
#define PSU_PRESENT_ATTR_ID(index)      PSU##index##_PRESENT
#define PSU_POWERGOOD_ATTR_ID(index)    PSU##index##_POWER_GOOD

#define PSU_ATTR(psu_id) \
		PSU_VIN_ATTR_ID(psu_id),		\
		PSU_VOUT_ATTR_ID(psu_id),		\
		PSU_IIN_ATTR_ID(psu_id),		\
		PSU_IOUT_ATTR_ID(psu_id),		\
		PSU_PIN_ATTR_ID(psu_id),		\
		PSU_POUT_ATTR_ID(psu_id),		\
		PSU_MODEL_ATTR_ID(psu_id),		\
		PSU_SERIAL_ATTR_ID(psu_id), 	\
		PSU_TEMP1_INPUT_ATTR_ID(psu_id), \
		PSU_TEMP2_INPUT_ATTR_ID(psu_id), \
		PSU_TEMP3_INPUT_ATTR_ID(psu_id), \
		PSU_FAN1_INPUT_ATTR_ID(psu_id), \
		PSU_PRESENT_ATTR_ID(psu_id),	\
		PSU_POWERGOOD_ATTR_ID(psu_id)

enum extreme7830_32ce_8de_psu_sysfs_attrs {
	/* psu attributes */
	PSU_ATTR(1),
	PSU_ATTR(2),
	NUM_OF_PSU_ATTR,
	NUM_OF_PER_PSU_ATTR = (NUM_OF_PSU_ATTR/NUM_OF_PSU)
};

/* psu attributes */
#define DECLARE_PSU_SENSOR_DEVICE_ATTR(index) \
		static SENSOR_DEVICE_ATTR(psu##index##_vin, S_IRUGO, show_psu,	 NULL, PSU##index##_VIN); \
		static SENSOR_DEVICE_ATTR(psu##index##_vout, S_IRUGO, show_psu,  NULL, PSU##index##_VOUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_iin, S_IRUGO, show_psu,  NULL, PSU##index##_IIN); \
		static SENSOR_DEVICE_ATTR(psu##index##_iout, S_IRUGO, show_psu,  NULL, PSU##index##_IOUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_pin, S_IRUGO, show_psu,  NULL, PSU##index##_PIN); \
		static SENSOR_DEVICE_ATTR(psu##index##_pout, S_IRUGO, show_psu,  NULL, PSU##index##_POUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_model, S_IRUGO, show_string,  NULL, PSU##index##_MODEL); \
		static SENSOR_DEVICE_ATTR(psu##index##_serial, S_IRUGO, show_string,  NULL, PSU##index##_SERIAL);\
		static SENSOR_DEVICE_ATTR(psu##index##_temp1_input, S_IRUGO, show_psu,	NULL, PSU##index##_TEMP1_INPUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_temp2_input, S_IRUGO, show_psu,	NULL, PSU##index##_TEMP2_INPUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_temp3_input, S_IRUGO, show_psu,	NULL, PSU##index##_TEMP3_INPUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_fan1_input, S_IRUGO, show_psu,	NULL, PSU##index##_FAN1_INPUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_present, S_IRUGO, show_psu_status,	 NULL, PSU##index##_PRESENT); \
		static SENSOR_DEVICE_ATTR(psu##index##_power_good, S_IRUGO, show_psu_status,  NULL, PSU##index##_POWER_GOOD)
#define DECLARE_PSU_ATTR(index) \
		&sensor_dev_attr_psu##index##_vin.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_vout.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_iin.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_iout.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_pin.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_pout.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_model.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_serial.dev_attr.attr,\
		&sensor_dev_attr_psu##index##_temp1_input.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_temp2_input.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_temp3_input.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_fan1_input.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_present.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_power_good.dev_attr.attr

DECLARE_PSU_SENSOR_DEVICE_ATTR(1);
DECLARE_PSU_SENSOR_DEVICE_ATTR(2);

static struct attribute *extreme7830_32ce_8de_psu_attributes[] = {
	/* psu attributes */
	DECLARE_PSU_ATTR(1),
	DECLARE_PSU_ATTR(2),
	NULL
};

static const struct attribute_group extreme7830_32ce_8de_psu_group = {
	.attrs = extreme7830_32ce_8de_psu_attributes,
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

static struct extreme7830_32ce_8de_psu_data *extreme7830_32ce_8de_psu_update_device_status(struct device_attribute *da)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char pid = attr->index / NUM_OF_PER_PSU_ATTR;
	int status = 0;

	if (time_before(jiffies, data->last_updated_status[pid] + HZ * 5) && data->valid_status[pid]) {
		return data;
	}

	data->valid_status[pid] = 0;

	/* Get status from ipmi
	 * ipmitool raw 0x06 0x52 0x09 0xbc 0x1 0x11
	 * ipmitool raw 0x06 0x52 0x09 0xbc 0x1 0x12
	 */
	data->ipmi.tx_message.netfn = IPMI_APP_NETFN;
	data->ipmi_tx_data[0] = IPMI_PWRCPLD_BUS;
	data->ipmi_tx_data[1] = IPMI_PWRCPLD_ADDRESS;
	data->ipmi_tx_data[2] = IPMI_READ_BYTE;

	switch (attr->index)
	{
		case PSU1_PRESENT:
		case PSU1_POWER_GOOD:
			data->ipmi_tx_data[3] = IPMI_SENSOR_OFFSET_PSU1_STAT;
			break;
		case PSU2_PRESENT:
		case PSU2_POWER_GOOD:
			data->ipmi_tx_data[3] = IPMI_SENSOR_OFFSET_PSU2_STAT;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				data->ipmi_tx_data, 4,
				&data->ipmi_resp_status[pid],
				sizeof(data->ipmi_resp_status[pid]));

	if (unlikely(status != 0))
	{
		DEBUG_PRINT("%s:%d \n", __FUNCTION__, __LINE__);
		goto exit;
	}

	if (unlikely(data->ipmi.rx_result != 0))
	{
		status = -EIO;
		DEBUG_PRINT("%s:%d \n", __FUNCTION__, __LINE__);
		goto exit;
	}

	data->last_updated_status[pid] = jiffies;
	data->valid_status[pid] = 1;

exit:
	return data;
}

static struct extreme7830_32ce_8de_psu_data *extreme7830_32ce_8de_psu_update_device(struct device_attribute *da)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int status = 0;

	if (time_before(jiffies, data->last_updated_value[attr->index] + HZ * 5) && data->valid_value[attr->index]) {
		return data;
	}

	data->valid_value[attr->index] = 0;

	/* Get PSU Power, Temperature and PSU FAN RPM from ipmi */
	data->ipmi.tx_message.netfn = IPMI_SENSOR_NETFN;

	switch (attr->index)
	{
		case PSU1_VIN:
			data->ipmi_tx_data[0] = SENSOR_PS1_IN_VOL;
			break;
		case PSU2_VIN:
			data->ipmi_tx_data[0] = SENSOR_PS2_IN_VOL;
			break;
		case PSU1_VOUT:
			data->ipmi_tx_data[0] = SENSOR_PS1_OUT_VOL;
			break;
		case PSU2_VOUT:
			data->ipmi_tx_data[0] = SENSOR_PS2_OUT_VOL;
			break;
		case PSU1_IIN:
			data->ipmi_tx_data[0] = SENSOR_PS1_IN_CUR;
			break;
		case PSU2_IIN:
			data->ipmi_tx_data[0] = SENSOR_PS2_IN_CUR;
			break;
		case PSU1_IOUT:
			data->ipmi_tx_data[0] = SENSOR_PS1_OUT_CUR;
			break;
		case PSU2_IOUT:
			data->ipmi_tx_data[0] = SENSOR_PS2_OUT_CUR;
			break;
		case PSU1_PIN:
			data->ipmi_tx_data[0] = SENSOR_PS1_IN_PWR;
			break;
		case PSU2_PIN:
			data->ipmi_tx_data[0] = SENSOR_PS2_IN_PWR;
			break;
		case PSU1_POUT:
			data->ipmi_tx_data[0] = SENSOR_PS1_OUT_PWR;
			break;
		case PSU2_POUT:
			data->ipmi_tx_data[0] = SENSOR_PS2_OUT_PWR;
			break;
		case PSU1_TEMP1_INPUT:
			data->ipmi_tx_data[0] = SENSOR_PS1_TEMP1;
			break;
		case PSU1_TEMP2_INPUT:
			data->ipmi_tx_data[0] = SENSOR_PS1_TEMP2;
			break;
		case PSU1_TEMP3_INPUT:
			data->ipmi_tx_data[0] = SENSOR_PS1_TEMP3;
			break;
		case PSU2_TEMP1_INPUT:
			data->ipmi_tx_data[0] = SENSOR_PS2_TEMP1;
			break;
		case PSU2_TEMP2_INPUT:
			data->ipmi_tx_data[0] = SENSOR_PS2_TEMP2;
			break;
		case PSU2_TEMP3_INPUT:
			data->ipmi_tx_data[0] = SENSOR_PS2_TEMP3;
			break;
		case PSU1_FAN1_INPUT:
			data->ipmi_tx_data[0] = SENSOR_PS1_FAN1_PWM;
			break;
		case PSU2_FAN1_INPUT:
			data->ipmi_tx_data[0] = SENSOR_PS2_FAN1_PWM;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_SENSOR_READ_CMD,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp_value[attr->index].ipmi_resp_raw,
				   sizeof(data->ipmi_resp_value[attr->index].ipmi_resp_raw));

	DEBUG_PRINT("%s: attr=%d value=%d\n", __FUNCTION__, attr->index, data->ipmi_resp_value[attr->index].ipmi_resp_raw[0]);

	if (unlikely(status != 0)){
		DEBUG_PRINT("Get PSU data failed: status != 0\n");
		goto exit;
	}
	if (unlikely(data->ipmi.rx_result != 0)) {
		DEBUG_PRINT("Get PSU data failed: data->ipmi.rx_result != 0\n");
		status = -EIO;
		goto exit;
	}

	data->last_updated_value[attr->index] = jiffies;
	data->valid_value[attr->index] = 1;

exit:
	return data;
}

static struct extreme7830_32ce_8de_psu_data *extreme7830_32ce_8de_psu_get_factors(void)
{
	int pid, status = 0;

	data->ipmi.tx_message.netfn = IPMI_SENSOR_NETFN;
	data->ipmi_tx_data[1] = IPMI_FACTORS_READING_BYTE;


	/* Get vin factors from ipmi */
	for(pid = PSU_1; pid < NUM_OF_PSU; pid++)
	{
		switch (pid)
		{
			case PSU_1:
				data->ipmi_tx_data[0] = SENSOR_PS1_IN_VOL;
				break;
			case PSU_2:
				data->ipmi_tx_data[0] = SENSOR_PS2_IN_VOL;
				break;
			default:
				status = -EIO;
				goto exit;
		}

		status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD,
					   data->ipmi_tx_data, 2,
					   data->ipmi_factors[pid].ipmi_vin_factors,
					   sizeof(data->ipmi_factors[pid].ipmi_vin_factors));

		if (unlikely(status != 0)) {
			goto exit;
		}

		if (unlikely(data->ipmi.rx_result != 0)) {
			status = -EIO;
			goto exit;
		}
	}


	/* Get vout factors from ipmi */
	for(pid = PSU_1; pid < NUM_OF_PSU; pid++)
	{
		switch (pid)
		{
			case PSU_1:
				data->ipmi_tx_data[0] = SENSOR_PS1_OUT_VOL;
				break;
			case PSU_2:
				data->ipmi_tx_data[0] = SENSOR_PS2_OUT_VOL;
				break;
			default:
				status = -EIO;
				goto exit;
		}

		status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD,
					   data->ipmi_tx_data, 2,
					   data->ipmi_factors[pid].ipmi_vout_factors,
					   sizeof(data->ipmi_factors[pid].ipmi_vout_factors));

		if (unlikely(status != 0)) {
			goto exit;
		}

		if (unlikely(data->ipmi.rx_result != 0)) {
			status = -EIO;
			goto exit;
		}
	}


	/* Get iin factors from ipmi */
	for(pid = PSU_1; pid < NUM_OF_PSU; pid++)
	{
		switch (pid)
		{
			case PSU_1:
				data->ipmi_tx_data[0] = SENSOR_PS1_IN_CUR;
				break;
			case PSU_2:
				data->ipmi_tx_data[0] = SENSOR_PS2_IN_CUR;
				break;
			default:
				status = -EIO;
				goto exit;
		}

		status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD,
					   data->ipmi_tx_data, 2,
					   data->ipmi_factors[pid].ipmi_iin_factors,
					   sizeof(data->ipmi_factors[pid].ipmi_iin_factors));

		if (unlikely(status != 0)) {
			goto exit;
		}

		if (unlikely(data->ipmi.rx_result != 0)) {
			status = -EIO;
			goto exit;
		}
	}


	/* Get iout factors from ipmi */
	for(pid = PSU_1; pid < NUM_OF_PSU; pid++)
	{
		switch (pid)
		{
			case PSU_1:
				data->ipmi_tx_data[0] = SENSOR_PS1_OUT_CUR;
				break;
			case PSU_2:
				data->ipmi_tx_data[0] = SENSOR_PS2_OUT_CUR;
				break;
			default:
				status = -EIO;
				goto exit;
		}

		status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD,
					   data->ipmi_tx_data, 2,
					   data->ipmi_factors[pid].ipmi_iout_factors,
					   sizeof(data->ipmi_factors[pid].ipmi_iout_factors));

		if (unlikely(status != 0)) {
			goto exit;
		}

		if (unlikely(data->ipmi.rx_result != 0)) {
			status = -EIO;
			goto exit;
		}
	}


	/* Get pin factors from ipmi */
	for(pid = PSU_1; pid < NUM_OF_PSU; pid++)
	{
		switch (pid)
		{
			case PSU_1:
				data->ipmi_tx_data[0] = SENSOR_PS1_IN_PWR;
				break;
			case PSU_2:
				data->ipmi_tx_data[0] = SENSOR_PS2_IN_PWR;
				break;
			default:
				status = -EIO;
				goto exit;
		}

		status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD,
					   data->ipmi_tx_data, 2,
					   data->ipmi_factors[pid].ipmi_pin_factors,
					   sizeof(data->ipmi_factors[pid].ipmi_pin_factors));

		if (unlikely(status != 0)) {
			goto exit;
		}

		if (unlikely(data->ipmi.rx_result != 0)) {
			status = -EIO;
			goto exit;
		}
	}


	/* Get pout factors from ipmi */
	for(pid = PSU_1; pid < NUM_OF_PSU; pid++)
	{
		switch (pid)
		{
			case PSU_1:
				data->ipmi_tx_data[0] = SENSOR_PS1_OUT_PWR;
				break;
			case PSU_2:
				data->ipmi_tx_data[0] = SENSOR_PS2_OUT_PWR;
				break;
			default:
				status = -EIO;
				goto exit;
		}

		status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD,
					   data->ipmi_tx_data, 2,
					   data->ipmi_factors[pid].ipmi_pout_factors,
					   sizeof(data->ipmi_factors[pid].ipmi_pout_factors));

		if (unlikely(status != 0)) {
			goto exit;
		}

		if (unlikely(data->ipmi.rx_result != 0)) {
			status = -EIO;
			goto exit;
		}
	}


	/* Get temp1 factors from ipmi */
	for(pid = PSU_1; pid < NUM_OF_PSU; pid++)
	{
		switch (pid)
		{
			case PSU_1:
				data->ipmi_tx_data[0] = SENSOR_PS1_TEMP1;
				break;
			case PSU_2:
				data->ipmi_tx_data[0] = SENSOR_PS2_TEMP1;
				break;
			default:
				status = -EIO;
				goto exit;
		}

		status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD,
					   data->ipmi_tx_data, 2,
					   data->ipmi_factors[pid].ipmi_temp1_factors,
					   sizeof(data->ipmi_factors[pid].ipmi_temp1_factors));

		if (unlikely(status != 0)) {
			goto exit;
		}

		if (unlikely(data->ipmi.rx_result != 0)) {
			status = -EIO;
			goto exit;
		}
	}

	/* Get temp2 factors from ipmi */
	for(pid = PSU_1; pid < NUM_OF_PSU; pid++)
	{
		switch (pid)
		{
			case PSU_1:
				data->ipmi_tx_data[0] = SENSOR_PS1_TEMP2;
				break;
			case PSU_2:
				data->ipmi_tx_data[0] = SENSOR_PS2_TEMP2;
				break;
			default:
				status = -EIO;
				goto exit;
		}

		status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD,
					   data->ipmi_tx_data, 2,
					   data->ipmi_factors[pid].ipmi_temp2_factors,
					   sizeof(data->ipmi_factors[pid].ipmi_temp2_factors));

		if (unlikely(status != 0)) {
			goto exit;
		}

		if (unlikely(data->ipmi.rx_result != 0)) {
			status = -EIO;
			goto exit;
		}
	}

	/* Get temp3 factors from ipmi */
	for(pid = PSU_1; pid < NUM_OF_PSU; pid++)
	{
		switch (pid)
		{
			case PSU_1:
				data->ipmi_tx_data[0] = SENSOR_PS1_TEMP3;
				break;
			case PSU_2:
				data->ipmi_tx_data[0] = SENSOR_PS2_TEMP3;
				break;
			default:
				status = -EIO;
				goto exit;
		}

		status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD,
					   data->ipmi_tx_data, 2,
					   data->ipmi_factors[pid].ipmi_temp3_factors,
					   sizeof(data->ipmi_factors[pid].ipmi_temp3_factors));

		if (unlikely(status != 0)) {
			goto exit;
		}

		if (unlikely(data->ipmi.rx_result != 0)) {
			status = -EIO;
			goto exit;
		}
	}

	/* Get fan1 speed factors from ipmi */
	for(pid = PSU_1; pid < NUM_OF_PSU; pid++)
	{
		switch (pid)
		{
			case PSU_1:
				data->ipmi_tx_data[0] = SENSOR_PS1_FAN1_PWM;
				break;
			case PSU_2:
				data->ipmi_tx_data[0] = SENSOR_PS2_FAN1_PWM;
				break;
			default:
				status = -EIO;
				goto exit;
		}

		status = ipmi_send_message(&data->ipmi, IPMI_FACTORS_READ_CMD,
					   data->ipmi_tx_data, 2,
					   data->ipmi_factors[pid].ipmi_fan1_factors,
					   sizeof(data->ipmi_factors[pid].ipmi_fan1_factors));

		if (unlikely(status != 0)) {
			goto exit;
		}

		if (unlikely(data->ipmi.rx_result != 0)) {
			status = -EIO;
			goto exit;
		}
	}

exit:
	return data;
}

static struct extreme7830_32ce_8de_psu_data *extreme7830_32ce_8de_psu_update_string(struct device_attribute *da)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char pid = attr->index / NUM_OF_PER_PSU_ATTR;
	int status = 0;

	if (time_before(jiffies, data->last_updated[pid] + HZ * 5) && data->valid[pid]) {
		return data;
	}

	data->valid[pid] = 0;

	data->ipmi.tx_message.netfn = IPMI_APP_NETFN;


	/*
	 * Get model name from ipmi
	 * PSU1: ipmitool raw 0x06 0x52 0x20 0xb0 0xD 0x9A
	 * PSU2: ipmitool raw 0x06 0x52 0x22 0xb2 0xD 0x9A
	 */
	switch (pid)
	{
		case PSU_1:
			data->ipmi_tx_data[0] = IPMI_ENCODE_BUS_ID(IPMI_I2C_CHAN_PSU1_MICRO_CTRLER, IPMI_I2C_BUS_PSU1_MICRO_CTRLER);
			data->ipmi_tx_data[1] = IPMI_PSU1_CTRL_ADDR;
			break;

		case PSU_2:
			data->ipmi_tx_data[0] = IPMI_ENCODE_BUS_ID(IPMI_I2C_CHAN_PSU2_MICRO_CTRLER, IPMI_I2C_BUS_PSU2_MICRO_CTRLER);
			data->ipmi_tx_data[1] = IPMI_PSU2_CTRL_ADDR;
			break;

		default:
			status = -EIO;
			goto exit;
	}

	data->ipmi_tx_data[2] = sizeof(data->ipmi_resp[pid].model) - 1;
	data->ipmi_tx_data[3] = PSU_REG_MFR_MODEL;
	status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				data->ipmi_tx_data, 4,
				data->ipmi_resp[pid].model,
				sizeof(data->ipmi_resp[pid].model) - 1);

	if (unlikely(status != 0)) {
		goto exit;
	}

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	/*
	 * Get serial number from ipmi
	 * PSU1: ipmitool raw 0x06 0x52 0x20 0xb0 0x11 0x9E
	 * PSU2: ipmitool raw 0x06 0x52 0x22 0xb2 0x11 0x9E
	 */
	switch (pid)
	{
		case PSU_1:
			data->ipmi_tx_data[0] = IPMI_ENCODE_BUS_ID(IPMI_I2C_CHAN_PSU1_MICRO_CTRLER, IPMI_I2C_BUS_PSU1_MICRO_CTRLER);
			data->ipmi_tx_data[1] = IPMI_PSU1_CTRL_ADDR;
			break;

		case PSU_2:
			data->ipmi_tx_data[0] = IPMI_ENCODE_BUS_ID(IPMI_I2C_CHAN_PSU2_MICRO_CTRLER, IPMI_I2C_BUS_PSU2_MICRO_CTRLER);
			data->ipmi_tx_data[1] = IPMI_PSU2_CTRL_ADDR;
			break;

		default:
			status = -EIO;
			goto exit;
	}

	data->ipmi_tx_data[2] = sizeof(data->ipmi_resp[pid].serial) - 1;
	data->ipmi_tx_data[3] = PSU_REG_MFR_SERIAL;
	status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				data->ipmi_tx_data, 4,
				data->ipmi_resp[pid].serial,
				sizeof(data->ipmi_resp[pid].serial) - 1);

	if (unlikely(status != 0)) {
		goto exit;
	}

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated[pid] = jiffies;
	data->valid[pid] = 1;

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

static int result_convert(int x, u8 pid, u8 attr)
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
	int multiplier = 3; /* 1000 = 10 ^ 3 */
	int exponent1 = 0;
	int exponent2 = 0;
	int y1 = 0;
	int y2 = 0;
	int value = 0;

	switch (attr) {
			case PSU1_VIN:
			case PSU2_VIN:
				M = M_factors_get(data->ipmi_factors[pid].ipmi_vin_factors[1], data->ipmi_factors[pid].ipmi_vin_factors[2]);
				B = B_factors_get(data->ipmi_factors[pid].ipmi_vin_factors[3], data->ipmi_factors[pid].ipmi_vin_factors[4]);
				K1 = two_complement_to_int(data->ipmi_factors[pid].ipmi_vin_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_factors[pid].ipmi_vin_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_psu result_convert: x:%d, pid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, pid, attr, M, B, K1, K2);
				break;
			case PSU1_VOUT:
			case PSU2_VOUT:
				M = M_factors_get(data->ipmi_factors[pid].ipmi_vout_factors[1], data->ipmi_factors[pid].ipmi_vout_factors[2]);
				B = B_factors_get(data->ipmi_factors[pid].ipmi_vout_factors[3], data->ipmi_factors[pid].ipmi_vout_factors[4]);
				K1 = two_complement_to_int(data->ipmi_factors[pid].ipmi_vout_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_factors[pid].ipmi_vout_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_psu result_convert: x:%d, pid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, pid, attr, M, B, K1, K2);
				break;
			case PSU1_IIN:
			case PSU2_IIN:
				M = M_factors_get(data->ipmi_factors[pid].ipmi_iin_factors[1], data->ipmi_factors[pid].ipmi_iin_factors[2]);
				B = B_factors_get(data->ipmi_factors[pid].ipmi_iin_factors[3], data->ipmi_factors[pid].ipmi_iin_factors[4]);
				K1 = two_complement_to_int(data->ipmi_factors[pid].ipmi_iin_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_factors[pid].ipmi_iin_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_psu result_convert: x:%d, pid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, pid, attr, M, B, K1, K2);
				break;
			case PSU1_IOUT:
			case PSU2_IOUT:
				M = M_factors_get(data->ipmi_factors[pid].ipmi_iout_factors[1], data->ipmi_factors[pid].ipmi_iout_factors[2]);
				B = B_factors_get(data->ipmi_factors[pid].ipmi_iout_factors[3], data->ipmi_factors[pid].ipmi_iout_factors[4]);
				K1 = two_complement_to_int(data->ipmi_factors[pid].ipmi_iout_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_factors[pid].ipmi_iout_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_psu result_convert: x:%d, pid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, pid, attr, M, B, K1, K2);
				break;
			case PSU1_PIN:
			case PSU2_PIN:
				M = M_factors_get(data->ipmi_factors[pid].ipmi_pin_factors[1], data->ipmi_factors[pid].ipmi_pin_factors[2]);
				B = B_factors_get(data->ipmi_factors[pid].ipmi_pin_factors[3], data->ipmi_factors[pid].ipmi_pin_factors[4]);
				K1 = two_complement_to_int(data->ipmi_factors[pid].ipmi_pin_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_factors[pid].ipmi_pin_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_psu result_convert: x:%d, pid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, pid, attr, M, B, K1, K2);
				break;
			case PSU1_POUT:
			case PSU2_POUT:
				M = M_factors_get(data->ipmi_factors[pid].ipmi_pout_factors[1], data->ipmi_factors[pid].ipmi_pout_factors[2]);
				B = B_factors_get(data->ipmi_factors[pid].ipmi_pout_factors[3], data->ipmi_factors[pid].ipmi_pout_factors[4]);
				K1 = two_complement_to_int(data->ipmi_factors[pid].ipmi_pout_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_factors[pid].ipmi_pout_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_psu result_convert: x:%d, pid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, pid, attr, M, B, K1, K2);
				break;
			case PSU1_TEMP1_INPUT:
			case PSU2_TEMP1_INPUT:
				M = M_factors_get(data->ipmi_factors[pid].ipmi_temp1_factors[1], data->ipmi_factors[pid].ipmi_temp1_factors[2]);
				B = B_factors_get(data->ipmi_factors[pid].ipmi_temp1_factors[3], data->ipmi_factors[pid].ipmi_temp1_factors[4]);
				K1 = two_complement_to_int(data->ipmi_factors[pid].ipmi_temp1_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_factors[pid].ipmi_temp1_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_psu result_convert: x:%d, pid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, pid, attr, M, B, K1, K2);
				break;
			case PSU1_TEMP2_INPUT:
			case PSU2_TEMP2_INPUT:
				M = M_factors_get(data->ipmi_factors[pid].ipmi_temp2_factors[1], data->ipmi_factors[pid].ipmi_temp2_factors[2]);
				B = B_factors_get(data->ipmi_factors[pid].ipmi_temp2_factors[3], data->ipmi_factors[pid].ipmi_temp2_factors[4]);
				K1 = two_complement_to_int(data->ipmi_factors[pid].ipmi_temp2_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_factors[pid].ipmi_temp2_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_psu result_convert: x:%d, pid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, pid, attr, M, B, K1, K2);
				break;
			case PSU1_TEMP3_INPUT:
			case PSU2_TEMP3_INPUT:
				M = M_factors_get(data->ipmi_factors[pid].ipmi_temp3_factors[1], data->ipmi_factors[pid].ipmi_temp3_factors[2]);
				B = B_factors_get(data->ipmi_factors[pid].ipmi_temp3_factors[3], data->ipmi_factors[pid].ipmi_temp3_factors[4]);
				K1 = two_complement_to_int(data->ipmi_factors[pid].ipmi_temp3_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_factors[pid].ipmi_temp3_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_psu result_convert: x:%d, pid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, pid, attr, M, B, K1, K2);
				break;
			case PSU1_FAN1_INPUT:
			case PSU2_FAN1_INPUT:
				M = M_factors_get(data->ipmi_factors[pid].ipmi_fan1_factors[1], data->ipmi_factors[pid].ipmi_fan1_factors[2]);
				B = B_factors_get(data->ipmi_factors[pid].ipmi_fan1_factors[3], data->ipmi_factors[pid].ipmi_fan1_factors[4]);
				K1 = two_complement_to_int(data->ipmi_factors[pid].ipmi_fan1_factors[6] & 0x0f, 4, 0x0f);
				K2 = two_complement_to_int((data->ipmi_factors[pid].ipmi_fan1_factors[6] & 0xf0) >> 4, 4, 0x0f);
				DEBUG_PRINT("7830_32ce_8de_psu result_convert: x:%d, pid:%d, attr:%d, M:%d, B:%d, K1:%d, K2:%d \n", x, pid, attr, M, B, K1, K2);
				multiplier = 0;
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

	DEBUG_PRINT("7830_32ce_8de_psu result_convert: value:%d \n", value);

	return value;
}

static ssize_t show_psu_status(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char pid = attr->index / NUM_OF_PER_PSU_ATTR;
	int value = 0;
	int present = 0;
	int error = 0;
	u8 mask = 0;

	mutex_lock(&data->update_lock);

	data = extreme7830_32ce_8de_psu_update_device_status(da);
	if (!data->valid_status[pid]) {
		error = -EIO;
		goto exit;
	}

	mask = 0x1 << PSU_PRESENT;
	present = !(!!(data->ipmi_resp_status[pid] & mask));	/* 0 = Present, 1 = Not Present, need to convert */

	switch (attr->index) {
		case PSU1_PRESENT:
		case PSU2_PRESENT:
			value = present;
			break;
		case PSU1_POWER_GOOD:
		case PSU2_POWER_GOOD:
            VALIDATE_PRESENT_RETURN(pid);
			mask = 0x1 << PSU_POWER_GOOD_CPLD;
            value = (data->ipmi_resp_status[pid] & mask) >> PSU_POWER_GOOD_CPLD;
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

static ssize_t show_psu(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char pid = attr->index / NUM_OF_PER_PSU_ATTR;
	int value = 0;
	int error = 0;

	mutex_lock(&data->update_lock);

	data = extreme7830_32ce_8de_psu_update_device(da);
	if (!data->valid_value[attr->index]) {
		error = -EIO;
		goto exit;
	}

	switch (attr->index) {
		case PSU1_VIN:
		case PSU2_VIN:
		case PSU1_VOUT:
		case PSU2_VOUT:
		case PSU1_IIN:
		case PSU2_IIN:
		case PSU1_IOUT:
		case PSU2_IOUT:
		case PSU1_PIN:
		case PSU2_PIN:
		case PSU1_POUT:
		case PSU2_POUT:
		case PSU1_TEMP1_INPUT:
		case PSU2_TEMP1_INPUT:
		case PSU1_TEMP2_INPUT:
		case PSU2_TEMP2_INPUT:
		case PSU1_TEMP3_INPUT:
		case PSU2_TEMP3_INPUT:
		case PSU1_FAN1_INPUT:
		case PSU2_FAN1_INPUT:
			value = result_convert(data->ipmi_resp_value[attr->index].ipmi_resp_raw[0], pid, attr->index);
			DEBUG_PRINT("7830_32ce_8de_psu show_psu: pid:%d, attr:%d, convert_value:%d, x=%d \n", pid, attr->index, value, data->ipmi_resp_value[attr->index].ipmi_resp_raw[0]);
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	DEBUG_PRINT("7830_32ce_8de_psu show_psu: pid:%d, attr:%d, value:%d \n", pid, attr->index, value);

	mutex_unlock(&data->update_lock);
	return sprintf(buf, "%d\n", value);

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

/* trim tail(right) space */
char *rtrim(char *str)
{
    if (str == NULL || *str == '\0')
    {
        return str;
    }
    int len = strlen(str);
    char *p = str + len - 1;
    while (p >= str && (isspace(*p) || *p < 33 || *p > 122)) /* only print ascii 33~122 */
    {
        *p = '\0'; --p;
    }
    return str;
}

/* trim head(left) space */
char *ltrim(char *str)
{
    if (str == NULL || *str == '\0')
    {
        return str;
    }
    int len = 0;
    char *p = str;
    while (*p != '\0' && (isspace(*p) || *p < 33 || *p > 122)) /* only print ascii 33~122 */
    {
        ++p; ++len;
    }
    memmove(str, p, strlen(str) - len + 1);
    return str;
}

char *trim(char *str)
{
    str = rtrim(str);
    str = ltrim(str);
    return str;
}

static ssize_t show_string(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char pid = attr->index / NUM_OF_PER_PSU_ATTR;
	char *str = NULL;
	int error = 0;

	mutex_lock(&data->update_lock);

	data = extreme7830_32ce_8de_psu_update_string(da);
	if (!data->valid[pid]) {
		error = -EIO;
		goto exit;
	}

	switch (attr->index) {
		case PSU1_MODEL:
		case PSU2_MODEL:
			str = data->ipmi_resp[pid].model;
			str = trim(str);
			break;
		case PSU1_SERIAL:
		case PSU2_SERIAL:
			str = data->ipmi_resp[pid].serial;
			str = trim(str);
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	mutex_unlock(&data->update_lock);
	return sprintf(buf, "%s\n", str);

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static int extreme7830_32ce_8de_psu_probe(struct platform_device *pdev)
{
	int status = -1;

	/* Register sysfs hooks */
	status = sysfs_create_group(&pdev->dev.kobj, &extreme7830_32ce_8de_psu_group);
	if (status) {
		goto exit;
	}


	dev_info(&pdev->dev, "device created\n");

	return 0;

exit:
	return status;
}

static int extreme7830_32ce_8de_psu_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &extreme7830_32ce_8de_psu_group);
	return 0;
}

static int __init extreme7830_32ce_8de_psu_init(void)
{
	int ret;

	data = kzalloc(sizeof(struct extreme7830_32ce_8de_psu_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	mutex_init(&data->update_lock);
	data->valid[0] = 0;
	data->valid[1] = 0;

	ret = platform_driver_register(&extreme7830_32ce_8de_psu_driver);
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

	/* Get attributes factors */
	extreme7830_32ce_8de_psu_get_factors();

	return 0;

ipmi_err:
	platform_device_unregister(data->pdev);
dev_reg_err:
	platform_driver_unregister(&extreme7830_32ce_8de_psu_driver);
dri_reg_err:
	kfree(data);
alloc_err:
	return ret;
}

static void __exit extreme7830_32ce_8de_psu_exit(void)
{
	ipmi_destroy_user(data->ipmi.user);
	platform_device_unregister(data->pdev);
	platform_driver_unregister(&extreme7830_32ce_8de_psu_driver);
	kfree(data);
}

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extremenetworks 7830-32ce-8de PSU driver");
MODULE_LICENSE("GPL");

module_init(extreme7830_32ce_8de_psu_init);
module_exit(extreme7830_32ce_8de_psu_exit);
