/*
 * A VIM CPLD IPMI kernel dirver for extremenetworks 7830-32ce-8de
 *
 * Copyright (C) 2023 Alphanetworks Technology Corporation.
 * Ruby Wang <Ru-xin_Wang@alphanetworks.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
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

#define DRVNAME 								"7830_bmc_vim_pwr_cpld"
#define DEBUG_MODE                              0

/* Get VIM Status */
#define IPMI_APP_NETFN							0x6
#define IPMI_READ_WRITE_CMD						0x52
#define IPMI_VIM_PWRCPLD_BUS					0x09
#define IPMI_PCA9548_9_ADDRESS                  0xe0    /* 0x70 << 1 = 0xe0 */
#define IPMI_PCA9548_2_ADDRESS                  0xea    /* 0x75 << 1 = 0xea */
#define IPMI_VIM_PWRCPLD_ADDRESS				0xba	/* 0x5d << 1 = 0xba */
#define IPMI_READ_BYTE							0x01 	/* Read */
#define IPMI_WRITE_BYTE							0x00 	/* Write */
#define IPMI_CHANNEL_0_ADDRESS  				0x01	/* CH0 */
#define IPMI_CHANNEL_1_ADDRESS  				0x02	/* CH1 */

/* VIM CPLD1 0x5D*/
#define VIM_BOARD_ID_ADDRESS            	    0x01
#define VIM_CPLD_VERSION_ADDRESS            	0x00

#define IPMI_TIMEOUT							(20 * HZ)
#define IPMI_ERR_RETRY_TIMES					1

#define VIM_BOARD_ID_BITS_MASK 		   			0x07 	/* bit 0~2 */
#define VIM_CPLD_VERSION_BITS_MASK    			0x0f 	/* bit 0~3 */

static unsigned int debug = DEBUG_MODE;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");


#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)

static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data);
static ssize_t show_vim_board_id(struct device *dev, struct device_attribute *da, char *buf);
static ssize_t show_vim_cpld_version(struct device *dev, struct device_attribute *da, char *buf);
static int extreme7830_32ce_8de_vim_pwr_cpld_probe(struct platform_device *pdev);
static int extreme7830_32ce_8de_vim_pwr_cpld_remove(struct platform_device *pdev);

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
    VIM_24YE,
    VIM_NONE, 
    VIM_TYPE_ID_MAX 
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

struct extreme7830_32ce_8de_vim_pwr_cpld_data {
	struct platform_device 			*pdev;
	struct mutex                    update_lock;
	char                            valid[2];                   /* != 0 if registers are valid */
	unsigned long                   last_updated[2];            /* In jiffies */
	struct ipmi_data ipmi;   

    /* bit0: VIM1_BOARD_ID, bit1: VIM2_BOARD_ID */
    unsigned char   ipmi_resp[2];   
	unsigned char   ipmi_tx_data[5];
};

struct extreme7830_32ce_8de_vim_pwr_cpld_data *data = NULL;

static struct platform_driver extreme7830_32ce_8de_vim_pwr_cpld_driver = {
	.probe		= extreme7830_32ce_8de_vim_pwr_cpld_probe,
	.remove 	= extreme7830_32ce_8de_vim_pwr_cpld_remove,
	.driver 	= {
		.name	= DRVNAME,
		.owner	= THIS_MODULE,
	},
};


#define VIM_BOARD_ID_ATTR_ID(index)					                VIM_##index##_BOARD_ID
#define VIM_VERSION_ATTR_ID(index)					                VIM_##index##_VERSION

enum extreme7830_32ce_8de_vim_pwr_cpld_sysfs_attributes {
    /* VIM attributes */
	VIM_VERSION_ATTR_ID(1),
    VIM_VERSION_ATTR_ID(2), 
    VIM_BOARD_ID_ATTR_ID(1),
    VIM_BOARD_ID_ATTR_ID(2),  
};

/* VIM_BOARD_ID attributes */
#define DECLARE_VIM_VERSION_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(vim_##index##_version, S_IRUGO, show_vim_cpld_version, NULL, VIM_##index##_VERSION)
#define DECLARE_VIM_VERSION_ATTR(index)  &sensor_dev_attr_vim_##index##_version.dev_attr.attr

#define DECLARE_VIM_BOARD_ID_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(vim_##index##_board_id, S_IRUGO, show_vim_board_id, NULL, VIM_##index##_BOARD_ID)
#define DECLARE_VIM_BOARD_ID_ATTR(index)  &sensor_dev_attr_vim_##index##_board_id.dev_attr.attr

DECLARE_VIM_VERSION_SENSOR_DEVICE_ATTR(1);
DECLARE_VIM_VERSION_SENSOR_DEVICE_ATTR(2);
DECLARE_VIM_BOARD_ID_SENSOR_DEVICE_ATTR(1);
DECLARE_VIM_BOARD_ID_SENSOR_DEVICE_ATTR(2);


/* VIM#1 power CPLD */
static struct attribute *extreme7830_32ce_8de_vim_pwr_cpld_attributes[] = {
	DECLARE_VIM_VERSION_ATTR(1),
    DECLARE_VIM_VERSION_ATTR(2),
    DECLARE_VIM_BOARD_ID_ATTR(1),
    DECLARE_VIM_BOARD_ID_ATTR(2),
    NULL
};
static const struct attribute_group extreme7830_32ce_8de_vim_pwr_cpld_group = {
    .attrs = extreme7830_32ce_8de_vim_pwr_cpld_attributes,
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

static struct extreme7830_32ce_8de_vim_pwr_cpld_data *extreme7830_32ce_8de_vim_pwr_cpld_update_status_data(struct device_attribute *da)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    unsigned char vid = attr->index % VIM_ID_MAX;
	int status = 0;

	if (time_before(jiffies, data->last_updated[vid] + HZ * 5) && data->valid[vid]) {
		return data;
	}

	data->valid[vid] = 0;

	data->ipmi.tx_message.netfn = IPMI_APP_NETFN;
	data->ipmi_tx_data[0] = IPMI_VIM_PWRCPLD_BUS;
	
	switch (vid) 
	{
		case VIM_1:
			/* set PCA9548#9(0x70)'s channel to CH0(0x1) */
			data->ipmi_tx_data[1] = IPMI_PCA9548_9_ADDRESS;
			data->ipmi_tx_data[2] = IPMI_WRITE_BYTE;
			data->ipmi_tx_data[3] = 0x00;
			data->ipmi_tx_data[4] = IPMI_CHANNEL_0_ADDRESS;
			status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
					  data->ipmi_tx_data, 5,
					  NULL,
					  0);
			if (unlikely(status != 0)) {
				goto exit;
			}
	
			if (unlikely(data->ipmi.rx_result != 0)) {
				status = -EIO;
				goto exit;
			}
			break;

		case VIM_2:
			/* set PCA9548#9(0x70)'s channel to CH1(0x2) */
			data->ipmi_tx_data[1] = IPMI_PCA9548_9_ADDRESS;
			data->ipmi_tx_data[2] = IPMI_WRITE_BYTE;
			data->ipmi_tx_data[3] = 0x00;
			data->ipmi_tx_data[4] = IPMI_CHANNEL_1_ADDRESS;
			status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
					  data->ipmi_tx_data, 5,
					  NULL,
					  0);
			if (unlikely(status != 0)) {
				goto exit;
			}
	
			if (unlikely(data->ipmi.rx_result != 0)) {
				status = -EIO;
				goto exit;
			}
            break;
	}


    /* set PCA9548#2(0x75)'s channel to CH0(0x1) */
	data->ipmi_tx_data[1] = IPMI_PCA9548_2_ADDRESS;
	data->ipmi_tx_data[2] = IPMI_WRITE_BYTE;
	data->ipmi_tx_data[3] = 0x00;
	data->ipmi_tx_data[4] = IPMI_CHANNEL_0_ADDRESS;
	status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				data->ipmi_tx_data, 5,
				NULL,
				0);
	if (unlikely(status != 0)) {
		goto exit;
	}
	
	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}
	
    /* Get VIM Board ID and Version from VIM Power CPLD */
    data->ipmi_tx_data[1] = IPMI_VIM_PWRCPLD_ADDRESS;
	data->ipmi_tx_data[2] = IPMI_READ_BYTE;
    
	switch (attr->index) 
	{
		case VIM_1_VERSION:
        case VIM_2_VERSION:
			/* Get VIM Version from VIM Power CPLD */
			data->ipmi_tx_data[3] = VIM_CPLD_VERSION_ADDRESS;
			break;
		case VIM_1_BOARD_ID:
        case VIM_2_BOARD_ID:
			/* Get VIM Board ID from VIM Power CPLD */
            data->ipmi_tx_data[3] = VIM_BOARD_ID_ADDRESS;
            break;
        default:
			status = -EIO;
			goto exit;
	}
	
    status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				data->ipmi_tx_data, 4,
				&data->ipmi_resp[vid],
				sizeof(data->ipmi_resp[vid]));
	if (unlikely(status != 0)) {
		goto exit;
	}
	
	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated[vid] = jiffies;
	data->valid[vid] = 1;

exit:
	return data;
}

/*
 * Attributes function
 *		- show_vim_cpld_version	: VIM CPLD version
 *      - show_vim_board_id     : VIM board id
 */

static ssize_t show_vim_cpld_version(struct device *dev, struct device_attribute *da,
                           char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char vid = attr->index % VIM_ID_MAX;
	int value = 0;
    int mask = 0;
	int error = 0;
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
			return -EINVAL;
	}

	if (vim_present == -EIO)
	{
		return -EINVAL;
	}

	DEBUG_PRINT("7830_32ce_8de show_vim_cpld_version: vim%d_present=%d", vid, vim_present);

	switch (attr->index) {
		case VIM_1_VERSION:
        case VIM_2_VERSION:
			VALIDATE_PRESENT_RETURN(vid);
			data = extreme7830_32ce_8de_vim_pwr_cpld_update_status_data(da);
			if (!data->valid[vid]) {
				error = -EIO;
				goto exit;
			}
			
			value = data->ipmi_resp[vid] & VIM_CPLD_VERSION_BITS_MASK;
			break;
		default:
			return -EINVAL;
	}

	mutex_unlock(&data->update_lock);
	DEBUG_PRINT("7830_32ce_8de show_vim_cpld_version: data->ipmi_resp[%d]:0x%x", vid, data->ipmi_resp[vid]);
	return sprintf(buf, "%d\n", value);

exit:
	mutex_unlock(&data->update_lock);
	return error;    
}

static ssize_t show_vim_board_id(struct device *dev, struct device_attribute *da,
                           char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char vid = attr->index % VIM_ID_MAX;
	int value = 0;
    int mask = 0;
	int error = 0;
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

	if (vim_present == -EIO)
	{
		return -EINVAL;
	}

	DEBUG_PRINT("7830_32ce_8de show_vim_board_id: vim%d_present=%d", vid, vim_present);

	switch (attr->index) {
		case VIM_1_BOARD_ID:
		case VIM_2_BOARD_ID:
			VALIDATE_PRESENT_RETURN(vid);
			data = extreme7830_32ce_8de_vim_pwr_cpld_update_status_data(da);
			if (!data->valid[vid]) {
				error = -EIO;
				goto exit;
			}
			mask = 0x7; /* bit 0~2 */
            value = data->ipmi_resp[vid] & mask;
			break;

		default:
			error = -EINVAL;
			goto exit;
	}

	mutex_unlock(&data->update_lock);
	return sprintf(buf, "%d\n", value);

exit:
	mutex_unlock(&data->update_lock);
    value = VIM_NONE;
	return sprintf(buf, "%d\n", value);
}


static int extreme7830_32ce_8de_vim_pwr_cpld_probe(struct platform_device *pdev)
{
	int status = -1;

	/* Register sysfs hooks */
	status = sysfs_create_group(&pdev->dev.kobj, &extreme7830_32ce_8de_vim_pwr_cpld_group);
	if (status) {
		goto exit;
	}


	dev_info(&pdev->dev, "device created\n");

	return 0;

exit:
	return status;
}

static int extreme7830_32ce_8de_vim_pwr_cpld_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &extreme7830_32ce_8de_vim_pwr_cpld_group);
	return 0;
}

static int __init extreme7830_32ce_8de_vim_pwr_cpld_init(void)
{
	int ret;

	data = kzalloc(sizeof(struct extreme7830_32ce_8de_vim_pwr_cpld_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	mutex_init(&data->update_lock);
	data->valid[0] = 0;
	data->valid[1] = 0;

	ret = platform_driver_register(&extreme7830_32ce_8de_vim_pwr_cpld_driver);
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
	platform_driver_unregister(&extreme7830_32ce_8de_vim_pwr_cpld_driver);
dri_reg_err:
	kfree(data);
alloc_err:
	return ret;
}

static void __exit extreme7830_32ce_8de_vim_pwr_cpld_exit(void)
{
	ipmi_destroy_user(data->ipmi.user);
	platform_device_unregister(data->pdev);
	platform_driver_unregister(&extreme7830_32ce_8de_vim_pwr_cpld_driver);
	kfree(data);
}


MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extreme 7830_32ce_8de VIM power CPLD driver access from BMC");
MODULE_LICENSE("GPL");

module_init(extreme7830_32ce_8de_vim_pwr_cpld_init);
module_exit(extreme7830_32ce_8de_vim_pwr_cpld_exit);