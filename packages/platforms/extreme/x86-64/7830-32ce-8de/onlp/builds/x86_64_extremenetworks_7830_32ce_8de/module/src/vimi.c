/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2014 Accton Technology Corporation.
 *
 * Licensed under the Eclipse Public License, Version 1.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *        http://www.eclipse.org/legal/epl-v10.html
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 *
 * </bsn.cl>
 ************************************************************
 *
 * VIM Platform Implementation.
 *
 ***********************************************************/
#include <onlplib/file.h>
#include <onlplib/i2c.h>
#include <onlp/platformi/sfpi.h>
#include "platform_lib.h"
#include <string.h>
#include "vimi.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>

#define VIM_SEM_LOCK    do {sem_wait(&vim_i2c_info->mutex);} while(0)
#define VIM_SEM_UNLOCK  do {sem_post(&vim_i2c_info->mutex);} while(0)

static vim_callback_info_t vim_callbacks;
int extreme_update_vim_i2c_tree_sample(int vim_id, int operation, int vim_board_id);
int onlp_vimi_cpld_bus_id_get(int vim_id, int cpld_id);
onlp_vim_i2c_info_t *vim_i2c_info = NULL;
onlp_vim_i2c_info_t current_vim_i2c_info;

#if (DEBUG_FLAG_SAMPLE == 0)
/* ALPHA implements this function and only develops and tests it. */
int pca9548_i2c_bus_id_array[VIM_MAX_PCA_COUNT] = {58, 66, 74, 82, 90, 98, 106, 114};
char vim_pca9548_usage_status[VIM_MAX_PCA_COUNT];  /* 0 – VIM 1, 1 – VIM 2, 2 – NOT_USE */
int pca9548_ch0_bus_id_list[8] = {0};

vim_info vim_struct[VIM_TYPE_COUNT] =
{
    {
        /* 8DE */
        2,
        8,
        "optoe3"
    },
    {
        /* 16CE */
        3,
        16,
        "optoe1"
    },
    {
        /* 24CE */
        4,
        24,
        "optoe3"
    },
    {
        /* 24YE */
        4,
        24,
        "optoe2"
    },
    {
        /* None */
        0,
        0,
        "optoe"
    },
};
#endif /* DEBUG_FLAG_SAMPLE */

/**
 * @brief Create shared memory for struct onlp_vim_i2c_info_t.
 * @param id The shared memory id.
 * @returns 0 if create shared memory success, otherwise it failed.
 */
static int vimi_create_shm(key_t id)
{
    int rv;

    if (vim_i2c_info == NULL) {
        /* Create or retreive a shared memory region */
        rv = onlp_shmem_create(id, sizeof(onlp_vim_i2c_info_t),
                               (void**)&vim_i2c_info);
        if (rv >= 0) {
            /* If create shared memory success, initialize an unnamed semaphore */
            if(pltfm_create_sem(&vim_i2c_info->mutex) != 0) {
                AIM_DIE("%s(): mutex_init failed\n", __func__);
                return ONLP_STATUS_E_INTERNAL;
            }
        }
        else {
            AIM_DIE("Global %s created failed.", __func__);
        }
    }
    return ONLP_STATUS_OK;
}


/* Register callback function */
bool vim_register_callback(VIM_CALLBACK_FUNC callback)
{
	if (callback == NULL)
	{
		return FALSE;
	}

    if (vim_callbacks.valid == TRUE)
	{
		return FALSE;
	}

    vim_callbacks.valid = TRUE;
    vim_callbacks.callback = callback;
	return TRUE;

}

/* Deregister callback function */
bool vim_deregister_callback(void)
{
	vim_callbacks.valid = FALSE;
	vim_callbacks.callback = NULL;

	return TRUE;
}

/* Execute callback function */
bool execute_update_vim_i2c_tree(int vim_id, int operation, int vim_board_id)
{
    /* If the callback function is successfully registered */
	if (vim_callbacks.valid &&
	   (vim_callbacks.callback != NULL))
	{
        /* Execute the callback function and pass in relevant parameters */
		(*(vim_callbacks.callback))(
			vim_id,
			operation,
			vim_board_id);

		return TRUE;
	}

	return FALSE;
}

int new_i2c_device_for_vim(int vim_id, int vim_board_id)
{
#if DEBUG_FLAG_SAMPLE
    /*
     * Example only for this case: insert VIM 1 8DE, VIM 2 not present (Hard code: vim_id = 1(VIM 1), vim_board_id = 0(8DE))
     * Extreme can have your mechanism to create VIM I2C tree
     */

    /* This example only supports the operation of inserting VIM 1 8DE */
    if (vim_id != VIM1_ID || vim_board_id != VIM_8DE)
    {
        return -1;
    }

    /* Insert VIM 1 8DE */
    /* STEP1: Create I2C device node for pca9548#1 0x76 and CPLD0 0x5c */
    /* I2c address reference to VIM I2C block diagram in VIM HW spec.
     * Create I2C device node for pca9548#1 0x76
     *      0x76: I2c address for pca9548#1
     *      i2c-1: I2C bus id is 1
     * Create I2C device node for CPLD 0x5c
     *      0x5c: I2c address for VIM 1 CPLD0
     *      i2c-61: I2C bus id is 61
     */
    system("echo pca9548 0x76 > /sys/bus/i2c/devices/i2c-1/new_device");
    system("echo VIM1_CPLD0 0x5c > /sys/bus/i2c/devices/i2c-61/new_device");


    /* STEP2: Create I2C device node according to vim_board_id. */
    /* I2c address reference to VIM I2C block diagram in VIM HW spec.
     * Create I2C device node for pca9548#0 0x70
     *      0x70: I2c address for pca9548#0
     *      i2c-58: I2C bus id is 58
     */
    system("echo pca9548 0x70 > /sys/bus/i2c/devices/i2c-58/new_device");


    /* STEP3: Create I2C device node for optical transceiver (SFP/QSFP/CMIS) EEPROMs according to vim_board_id */
    /* I2c address reference to VIM I2C block diagram in VIM HW spec.
     * Create I2C device node for optical transceiver EEPROMs
     *      optoe3: I2C device name base on optical transceiver(SFP/QSFP/CMIS). Reference to optoe driver(optoe.c).
     *          - Use 'optoe1' to indicate this is a QSFP type device.
     *          - Use 'optoe2' to indicate this is an SFP type device.
     *          - Use 'optoe3' to indicate this is a CMIS type device. QSFP-DD is CMIS type device.
     *      0x50: I2c address for optical transceiver EEPROMs
     *      i2c-66~i2c-73: I2C bus id is 66~73
     */
    system("echo optoe3 0x50 > /sys/bus/i2c/devices/i2c-66/new_device");
    system("echo optoe3 0x50 > /sys/bus/i2c/devices/i2c-67/new_device");
    system("echo optoe3 0x50 > /sys/bus/i2c/devices/i2c-68/new_device");
    system("echo optoe3 0x50 > /sys/bus/i2c/devices/i2c-69/new_device");
    system("echo optoe3 0x50 > /sys/bus/i2c/devices/i2c-70/new_device");
    system("echo optoe3 0x50 > /sys/bus/i2c/devices/i2c-71/new_device");
    system("echo optoe3 0x50 > /sys/bus/i2c/devices/i2c-72/new_device");
    system("echo optoe3 0x50 > /sys/bus/i2c/devices/i2c-73/new_device");


    /* STEP4: Keep VIM I2C bus id for update VIM I2C tree db */
    current_vim_i2c_info.info[0].PCA9548_1_ch0_i2c_bus_id = 58;                 /* Channel 0 i2c bus id for VIM pca9548#1 */
    current_vim_i2c_info.info[0].PCA9548_2_ch0_i2c_bus_id = 66;                 /* Channel 0 i2c bus id for VIM pca9548#0 */
    current_vim_i2c_info.info[0].PCA9548_3_ch0_i2c_bus_id = PCA9548_NOT_USE;    /* Channel 0 i2c bus id for VIM pca9548#3 */
    current_vim_i2c_info.info[0].PCA9548_4_ch0_i2c_bus_id = PCA9548_NOT_USE;    /* Channel 0 i2c bus id for VIM pca9548#4 */

    return 0;
#else
    /* ALPHA implements this function and only develops and tests it. */
    int i = 0;
    int j = 0;
    int start_pca9548_id = -1;
    int pca9548_1_ch0_i2c_bus_id;
    int vim_index = vim_id - 1;     /* VIM index start at 0 (0 = VIM 1, 1 = VIM 2) */
    char cmd[CMD_SIZE] = {0};
    int count;

    /* STEP1: Create I2C device node for pca9548#1 0x76 and CPLD0 0x5c */
    /* Update PCA9548 usage status for pca9548#1 0x76 */
    for (i = 0; i < VIM_MAX_PCA_COUNT; i++)
    {
        if (vim_pca9548_usage_status[i] == NOT_USE)
        {
            start_pca9548_id = i;
            vim_pca9548_usage_status[i] = vim_index;
            break;
        }
    }

    if (start_pca9548_id < 0)
    {
        AIM_LOG_ERROR("Get start_pca9548_id from vim pca9548 usage status failed\r\n");
        return -1;
    }
    pca9548_1_ch0_i2c_bus_id = pca9548_i2c_bus_id_array[start_pca9548_id];

    /* Record I2C bus id for all PCA9548’s channel 0 information */
    current_vim_i2c_info.info[vim_index].PCA9548_1_ch0_i2c_bus_id = pca9548_1_ch0_i2c_bus_id;

    /* Create I2C device node for pca9548#1 0x76 */
    sprintf(cmd, "echo pca9548 0x%02X > /sys/bus/i2c/devices/i2c-%d/new_device", PCA9548_1_ADDR, ((vim_id == VIM1_ID) ? VIM1_PCA9548_1_BUS_ID : VIM2_PCA9548_1_BUS_ID));
    system(cmd);

    /* Create I2C device node for CPLD0 0x5c */
    sprintf(cmd, "echo VIM%d_CPLD0 0x%02X > /sys/bus/i2c/devices/i2c-%d/new_device", vim_id, VIM_CPLD0_ADDR, (pca9548_1_ch0_i2c_bus_id + VIM_PWR_CPLD_INDEX));
    system(cmd);


    /* STEP2: Create I2C device node according to vim_board_id. */
    /*
     * 1. Update PCA9548 usage status for pca9548#0 0x70, pca9548#3 0x71, pca9548#4 0x72.
     * 2. Record I2C bus id for PCA9548’s channel 0 information.
     */
    count = 1;
    for (i = 0; i < VIM_MAX_PCA_COUNT; i++)
    {
        if (vim_pca9548_usage_status[i] == NOT_USE)
        {
            if (count < vim_struct[vim_board_id].pca_count)
            {
                switch(count)
                {
                    case PCA9548_2:
                        current_vim_i2c_info.info[vim_index].PCA9548_2_ch0_i2c_bus_id = pca9548_i2c_bus_id_array[i];
                        break;

                    case PCA9548_3:
                        current_vim_i2c_info.info[vim_index].PCA9548_3_ch0_i2c_bus_id = pca9548_i2c_bus_id_array[i];
                         break;

                    case PCA9548_4:
                        current_vim_i2c_info.info[vim_index].PCA9548_4_ch0_i2c_bus_id = pca9548_i2c_bus_id_array[i];
                        break;

                     default:
                        AIM_LOG_ERROR("pca9548_%d does not exist\r\n", count);
                        break;
                }
                vim_pca9548_usage_status[i] = vim_index;
                count += 1;
            }
        }
    }

    switch (vim_board_id)
    {
        case VIM_8DE:
            /* 0x76 CH0 --> 0x70 */
            sprintf(cmd, "echo pca9548 0x%02X > /sys/bus/i2c/devices/i2c-%d/new_device", PCA9548_0_ADDR, pca9548_1_ch0_i2c_bus_id);
            system(cmd);
            break;

        case VIM_16CE:
            /* 0x76 CH0 --> 0x70,
             * 0x76 CH1 --> 0x71
             */
            sprintf(cmd, "echo pca9548 0x%02X > /sys/bus/i2c/devices/i2c-%d/new_device", PCA9548_0_ADDR, pca9548_1_ch0_i2c_bus_id);
            system(cmd);

            sprintf(cmd, "echo pca9548 0x%02X > /sys/bus/i2c/devices/i2c-%d/new_device", PCA9548_3_ADDR, (pca9548_1_ch0_i2c_bus_id + PCA9548_3_INDEX));
            system(cmd);
            break;

        case VIM_24CE:
        case VIM_24YE:
            /* 0x76 CH0 --> 0x70,
             * 0x76 CH1 --> 0x71,
             * 0x76 CH2 --> 0x72,
             * 0x76 CH4 --> 0x58 (CPLD1)
             */
            sprintf(cmd, "echo pca9548 0x%02X > /sys/bus/i2c/devices/i2c-%d/new_device", PCA9548_0_ADDR, pca9548_1_ch0_i2c_bus_id);
            system(cmd);

            sprintf(cmd, "echo pca9548 0x%02X > /sys/bus/i2c/devices/i2c-%d/new_device", PCA9548_3_ADDR, (pca9548_1_ch0_i2c_bus_id + PCA9548_3_INDEX));
            system(cmd);

            sprintf(cmd, "echo pca9548 0x%02X > /sys/bus/i2c/devices/i2c-%d/new_device", PCA9548_4_ADDR, (pca9548_1_ch0_i2c_bus_id + PCA9548_4_INDEX));
            system(cmd);

            sprintf(cmd, "echo VIM%d_CPLD1 0x%02X > /sys/bus/i2c/devices/i2c-%d/new_device", vim_id, VIM_CPLD1_ADDR, (pca9548_1_ch0_i2c_bus_id + VIM_PROT_CPLD_INDEX));
            system(cmd);
            break;

        default:
            AIM_LOG_ERROR("VIM board id(%d) does not exist\r\n", vim_board_id);
            break;
    }


    /* STEP3: Create optoe I2C device node according to vim_board_id */
    count = 0;
    for (i = 0; i < VIM_MAX_PCA_COUNT; i++)
    {
        if (vim_pca9548_usage_status[i] == vim_index)
        {
            /* Search VIM PCA9548 usage status, not include 0x76 */
            if ((count != 0) && (count < vim_struct[vim_board_id].pca_count))
            {
                for (j = pca9548_i2c_bus_id_array[i]; j < (pca9548_i2c_bus_id_array[i] + I2C_BUS_CHANNEL_COUNT); j++)
                {
                    sprintf(cmd, "echo %s 0x%02X > /sys/bus/i2c/devices/i2c-%d/new_device", vim_struct[vim_board_id].port_type, VIM_SFP_EEPROM_ADDR, j);
                    system(cmd);
                }
            }
            count += 1;
        }
    }

    /* Debug: Print VIM pca9548 usage status */
    AIM_SYSLOG_INFO("[SYSLOG_INFO]",
                    "[SYSLOG_INFO]",
                    "[SYSLOG_INFO] vim_pca9548_usage_status = [%d, %d, %d, %d, %d, %d, %d, %d])", \
                    vim_pca9548_usage_status[0], vim_pca9548_usage_status[1], \
                    vim_pca9548_usage_status[2], vim_pca9548_usage_status[3], \
                    vim_pca9548_usage_status[4], vim_pca9548_usage_status[5], \
                    vim_pca9548_usage_status[6], vim_pca9548_usage_status[7]);

    return 0;
#endif /* DEBUG_FLAG_SAMPLE */
}

int del_i2c_device_for_vim(int vim_id, int vim_board_id)
{
#if DEBUG_FLAG_SAMPLE
    /*
     * Example only for this case: remove VIM 1 8DE, VIM 2 not present (Hard code: vim_id = 1(VIM 1), vim_board_id = 0(8DE))
     * Extreme can have your mechanism to remove VIM I2C tree
     */

    /* This example only supports the operation of removing VIM 1 8DE */
    if (vim_id != VIM1_ID || vim_board_id != VIM_8DE)
    {
        return -1;
    }

    /* Remove VIM 1 8DE */
    /* STEP1: Delete I2C device node for optical transceiver (SFP/QSFP/CMIS) EEPROMs according to vim_board_id */
    /* I2c address reference to VIM I2C block diagram in VIM HW spec.
     * Delete I2C device node for optical transceiver EEPROMs
     *      0x50: I2c address for optoe I2C device
     *      i2c-66~i2c-73: I2C bus id is 66~73
     */
    system("echo 0x50 > /sys/bus/i2c/devices/i2c-66/delete_device");
    system("echo 0x50 > /sys/bus/i2c/devices/i2c-67/delete_device");
    system("echo 0x50 > /sys/bus/i2c/devices/i2c-68/delete_device");
    system("echo 0x50 > /sys/bus/i2c/devices/i2c-69/delete_device");
    system("echo 0x50 > /sys/bus/i2c/devices/i2c-70/delete_device");
    system("echo 0x50 > /sys/bus/i2c/devices/i2c-71/delete_device");
    system("echo 0x50 > /sys/bus/i2c/devices/i2c-72/delete_device");
    system("echo 0x50 > /sys/bus/i2c/devices/i2c-73/delete_device");


    /* STEP2: Delete I2C device node according to vim_board_id. */
    /* I2c address reference to VIM I2C block diagram in VIM HW spec.
     * Delete I2C device node for pca9548#0 0x70
     *      0x70: I2c address for pca9548#0
     *      i2c-58: I2C bus id is 58
     */
    system("echo 0x70 > /sys/bus/i2c/devices/i2c-58/delete_device");


    /* STEP3: Delete I2C device node for pca9548#1 0x76 and CPLD0 0x5c */
    /* I2c address reference to VIM I2C block diagram in VIM HW spec.
     * Delete I2C device node for CPLD 0x5c
     *      0x5c: I2c address for VIM 1 CPLD0
     *      i2c-61: I2C bus id is 61
     * Delete I2C device node for pca9548#1 0x76
     *      0x76: I2c address for pca9548#1
     *      i2c-1: I2C bus id is 1
     */
    system("echo 0x5c > /sys/bus/i2c/devices/i2c-61/delete_device");
    system("echo 0x76 > /sys/bus/i2c/devices/i2c-1/delete_device");


    /* STEP4: Keep VIM I2C bus id for update VIM I2C tree db */
    current_vim_i2c_info.info[0].PCA9548_1_ch0_i2c_bus_id = PCA9548_NOT_USE;
    current_vim_i2c_info.info[0].PCA9548_2_ch0_i2c_bus_id = PCA9548_NOT_USE;
    current_vim_i2c_info.info[0].PCA9548_3_ch0_i2c_bus_id = PCA9548_NOT_USE;
    current_vim_i2c_info.info[0].PCA9548_4_ch0_i2c_bus_id = PCA9548_NOT_USE;

    return 0;
#else
    /* ALPHA implements this function and only develops and tests it. */
    int i = 0;
    int j = 0;
    int pca9548_1_ch0_i2c_bus_id;
    int vim_index = vim_id - 1;     /* VIM index start at 0 (0 = VIM 1, 1 = VIM 2) */
    char cmd[CMD_SIZE] = {0};
    int count;

    /* STEP1: Delete optoe I2C device node according to vim_board_id */
    count = 0;
    for (i = 0; i < VIM_MAX_PCA_COUNT; i++)
    {
        if (vim_pca9548_usage_status[i] == vim_index)
        {
            /* Search VIM PCA9548 usage status, not include 0x76 */
            if ((count != 0) && (count < vim_struct[vim_board_id].pca_count))
            {
                for (j = pca9548_i2c_bus_id_array[i]; j < (pca9548_i2c_bus_id_array[i] + I2C_BUS_CHANNEL_COUNT); j++)
                {
                    sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", VIM_SFP_EEPROM_ADDR, j);
                    system(cmd);
                }
            }
            count += 1;
        }
    }


    /* STEP2: Delete I2C device node according to vim_board_id.  */
    /* Update PCA9548 usage status for pca9548#0 0x70, pca9548#3 0x71, pca9548#4 0x72. */
    count = 0;
    for (i = 0; i < VIM_MAX_PCA_COUNT; i++)
    {
        if (vim_pca9548_usage_status[i] == vim_index)
        {
            if (count == 0)
            {
                pca9548_1_ch0_i2c_bus_id = pca9548_i2c_bus_id_array[i];
            }
            else if ((count != 0) && (count < vim_struct[vim_board_id].pca_count))
            {
                vim_pca9548_usage_status[i] = NOT_USE;
            }
            count += 1;
        }
    }

    switch (vim_board_id)
    {
        case VIM_8DE:
            /* 0x76 CH0 --> 0x70 */
            sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", PCA9548_0_ADDR, pca9548_1_ch0_i2c_bus_id);
            system(cmd);
            break;

        case VIM_16CE:
            /* 0x76 CH0 --> 0x70,
             * 0x76 CH1 --> 0x71
             */
            sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", PCA9548_0_ADDR, pca9548_1_ch0_i2c_bus_id);
            system(cmd);

            sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", PCA9548_3_ADDR, pca9548_1_ch0_i2c_bus_id + PCA9548_3_INDEX);
            system(cmd);
            break;

        case VIM_24CE:
        case VIM_24YE:
            /* 0x76 CH0 --> 0x70,
             * 0x76 CH1 --> 0x71,
             * 0x76 CH2 --> 0x72,
             * 0x76 CH4 --> 0x58 (CPLD1)
             */
            sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", PCA9548_0_ADDR, pca9548_1_ch0_i2c_bus_id);
            system(cmd);

            sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", PCA9548_3_ADDR, pca9548_1_ch0_i2c_bus_id + PCA9548_3_INDEX);
            system(cmd);

            sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", PCA9548_4_ADDR, pca9548_1_ch0_i2c_bus_id + PCA9548_4_INDEX);
            system(cmd);

            sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", VIM_CPLD1_ADDR, pca9548_1_ch0_i2c_bus_id + VIM_PROT_CPLD_INDEX);
            system(cmd);
            break;

        default:
            AIM_LOG_ERROR("VIM board id(%d) does not exist\r\n", vim_board_id);
            break;
    }

    /* STEP3: Delete I2C device node for pca9548#1 0x76 and CPLD0 0x5c */
    /* Update PCA9548#1(0x76) usage status */
    for (i = 0; i < VIM_MAX_PCA_COUNT; i++)
    {
        if (vim_pca9548_usage_status[i] == vim_index)
        {
            vim_pca9548_usage_status[i] = NOT_USE;
            break;
        }
    }

    /* Delete PCA9548(0x76), CPLD0(0x5c) device on I2C tree */
    sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", VIM_CPLD0_ADDR, (pca9548_1_ch0_i2c_bus_id + VIM_PWR_CPLD_INDEX));
    system(cmd);

    if (vim_id == VIM1_ID)
    {
        sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", PCA9548_1_ADDR, VIM1_PCA9548_1_BUS_ID);
        system(cmd);
    }
    else
    {
        sprintf(cmd, "echo 0x%02X > /sys/bus/i2c/devices/i2c-%d/delete_device", PCA9548_1_ADDR, VIM2_PCA9548_1_BUS_ID);
        system(cmd);
    }

    /* STEP4: Record I2C bus id for all PCA9548’s channel 0 information*/
    current_vim_i2c_info.info[vim_index].PCA9548_1_ch0_i2c_bus_id = PCA9548_NOT_USE;
    current_vim_i2c_info.info[vim_index].PCA9548_2_ch0_i2c_bus_id = PCA9548_NOT_USE;
    current_vim_i2c_info.info[vim_index].PCA9548_3_ch0_i2c_bus_id = PCA9548_NOT_USE;
    current_vim_i2c_info.info[vim_index].PCA9548_4_ch0_i2c_bus_id = PCA9548_NOT_USE;

    /* Debug: Print VIM pca9548 usage status */
    AIM_SYSLOG_INFO("[SYSLOG_INFO]",
                    "[SYSLOG_INFO]",
                    "[SYSLOG_INFO] vim_pca9548_usage_status = [%d, %d, %d, %d, %d, %d, %d, %d]", \
                    vim_pca9548_usage_status[0], vim_pca9548_usage_status[1], \
                    vim_pca9548_usage_status[2], vim_pca9548_usage_status[3], \
                    vim_pca9548_usage_status[4], vim_pca9548_usage_status[5], \
                    vim_pca9548_usage_status[6], vim_pca9548_usage_status[7]);

    return 0;
#endif /* DEBUG_FLAG_SAMPLE */
}

int update_vim_i2c_tree_db(int vim_id, onlp_vim_i2c_info_t *current_vim_i2c_info)
{

    int res;
    int fd;
    int len;
    void *addr;
    char data[STORAGE_SIZE];

    AIM_SYSLOG_INFO("[SYSLOG_INFO]",
                    "[SYSLOG_INFO]",
                    "[SYSLOG_INFO] START update_vim_i2c_tree_db");

    /* Get shared memory file descriptor (not a file) */
    fd = shm_open(STORAGE_ID, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    if (fd == -1)
	{
        AIM_LOG_ERROR("[update_vim_i2c_tree_db] shm_open failed\r\n");
		return 10;
	}

    /* extend shared memory object as by default it's initialized with size 0 */
	res = ftruncate(fd, STORAGE_SIZE);
	if (res == -1)
	{
        AIM_LOG_ERROR("[update_vim_i2c_tree_db] ftruncate failed\r\n");
		return 20;
	}

    /* Map shared memory to process address space */
	addr = mmap(NULL, STORAGE_SIZE, PROT_WRITE, MAP_SHARED, fd, 0);
	if (addr == MAP_FAILED)
	{
        AIM_LOG_ERROR("[update_vim_i2c_tree_db] mmap failed\r\n");
		return 30;
	}

    /* Organize data into strings */
    sprintf(data, "%d %d %d %d %d %d %d %d", current_vim_i2c_info->info[VIM_1].PCA9548_1_ch0_i2c_bus_id,
                                             current_vim_i2c_info->info[VIM_1].PCA9548_2_ch0_i2c_bus_id,
                                             current_vim_i2c_info->info[VIM_1].PCA9548_3_ch0_i2c_bus_id,
                                             current_vim_i2c_info->info[VIM_1].PCA9548_4_ch0_i2c_bus_id,
                                             current_vim_i2c_info->info[VIM_2].PCA9548_1_ch0_i2c_bus_id,
                                             current_vim_i2c_info->info[VIM_2].PCA9548_2_ch0_i2c_bus_id,
                                             current_vim_i2c_info->info[VIM_2].PCA9548_3_ch0_i2c_bus_id,
                                             current_vim_i2c_info->info[VIM_2].PCA9548_4_ch0_i2c_bus_id);

    /* place data into memory */
	len = strlen(data) + 1;
	memcpy(addr, data, len);

    AIM_SYSLOG_INFO("[DEBUG]",
                    "[DEBUG]",
                    "[update_vim_i2c_tree_db] Write to shared memory: addr--(%s)", addr);

    AIM_SYSLOG_INFO("[DEBUG]",
                    "[DEBUG]",
                    "[update_vim_i2c_tree_db] Write to shared memory: data--(%s)", data);

    /* mmap cleanup */
	res = munmap(addr, STORAGE_SIZE);
	if (res == -1)
	{
        AIM_LOG_ERROR("[update_vim_i2c_tree_db] munmap failed\r\n");
		return 40;
	}

    return 0;
}

int extreme_update_vim_i2c_tree_sample(int vim_id, int operation, int vim_board_id)
{
    /* Implement callback function by Extreme */
    int rv;
    char cmd[CMD_SIZE] = {0};
    int cpld_bus_id;

    /*
     * 1. When VIM is inserted or removed, this register call back function will be called with vim_id,
     *    VIM board id and insert/remove information
     * 2. According to VIM id, VIM board id and inserted/removed information to update VIM I2C bus id
     *    and rebuild VIM I2C tree
     */
    /* STEP1: Extreme can follow your design to update VIM I2C tree */
    switch(operation)
    {
        case INSERT:
            /* Create VIM I2C tree according to vim_id and vim_board_id */
            rv = new_i2c_device_for_vim(vim_id, vim_board_id);
            break;

        case REMOVE:
            /* Delete VIM I2C tree according to vim_id and vim_board_id */
            rv = del_i2c_device_for_vim(vim_id, vim_board_id);
            break;

        default:
            rv = -1;
            AIM_LOG_ERROR("The operation is unknown.\r\n");
            break;
    }

    /* STEP2: Call Alpha VIM I2C function to update VIM I2C DB with I2C bus id information after I2C tree is updated */
    if (rv < 0)
    {
        AIM_LOG_ERROR("Updating the VIM %d I2C tree is not supported.\r\n", vim_id);
        return -1;
    }
    update_vim_i2c_tree_db(vim_id, &current_vim_i2c_info);


    cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_PORT_CPLD_ID);
    sprintf(cmd, "echo %d > /sys/bus/i2c/devices/%d-0058/vim_%d_board_id", vim_board_id, cpld_bus_id, vim_id);
    AIM_SYSLOG_INFO("[SYSLOG_INFO]",
                    "[SYSLOG_INFO]",
                    "[SYSLOG_INFO] Set VIM %d Board ID to %d. cmd: %s",vim_id, vim_board_id, cmd);
    system(cmd);


    return 0;
}

/*
 * This will be called to intiialize the vim subsystem.
 */
int
onlp_vimi_init(void)
{
    int vim1_present = -1;
    int vim2_present = -1;

    DIAG_PRINT("%s", __FUNCTION__);

    /* Init Share memory */
    if (vimi_create_shm(ONLP_VIMI_SHM_KEY) < 0) {
        AIM_DIE("%s::vimi_create_shm created failed.", __func__);
        return ONLP_STATUS_E_INTERNAL;
    }

#if (DEBUG_FLAG_SAMPLE == 0)
    /* Init vim_pca9548_usage_status */
    memset(vim_pca9548_usage_status, 2, 8*sizeof(char));  /* 0 = VIM 1 use, 1 = VIM 2 use, 2 = not use*/
#endif /* DEBUG_FLAG_SAMPLE */

    /* Extreme call vim_register_callback() to register VIM handler function */
    vim_register_callback(extreme_update_vim_i2c_tree_sample);

    /* Enable VIM power if VIM is present */
    vim1_present = onlp_vimi_present_get(VIM1_ID);
    vim2_present = onlp_vimi_present_get(VIM2_ID);
    if (vim1_present == VIM_PRESENT)
    {
        onlp_vimi_power_control(ON, VIM1_ID);
        sleep(1);
    }

    if (vim2_present == VIM_PRESENT)
    {
        onlp_vimi_power_control(ON, VIM2_ID);
        sleep(1);
    }

    return ONLP_STATUS_OK;
}


/*
 * -------------------------------------------
 * |                ONLPI API                |
 * -------------------------------------------
 */
int
onlp_vimi_present_get(int vim_id)
{
    int present;
    if (onlp_file_read_int(&present, VIM_PRESENT_PATH, vim_id))
    {
        AIM_LOG_ERROR("Unable to read present status from VIM(%d)\r\n", vim_id);
        return ONLP_STATUS_E_INTERNAL;
    }

    return present;
}


int
onlp_vimi_board_id_get(int vim_id)
{
    /* VIM Power CPLD(0x5D) accessed from BMC. */
    int vim_board_id = VIM_NONE;
    int vim_present = -1;

    vim_present = onlp_vimi_present_get(vim_id);
    if (vim_present == VIM_PRESENT)
    {
        if (onlp_file_read_int(&vim_board_id, VIM_BOARD_ID_PATH, vim_id))
        {
            AIM_LOG_ERROR("Unable to read vim_board_id status from VIM(%d)\r\n", vim_id);
            return ONLP_STATUS_E_INTERNAL;
        }
    }
    else if (vim_present == VIM_NOT_PRESENT)
    {
        vim_board_id = VIM_NONE;
    }

    return vim_board_id;
}

int
onlp_vimi_cpld_bus_id_get(int vim_id, int cpld_id)
{
    int cpld_bus_id;
    int fd;
    char data[STORAGE_SIZE];
    void *addr;
    int count = 0;
    int res;


    /* Get shared memory file descriptor (not a file) */
    fd = shm_open(STORAGE_ID, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    if (fd == -1)
    {
        AIM_LOG_ERROR("[onlp_vimi_cpld_bus_id_get] shm_open failed\r\n");
        return 10;
    }

    /* Map shared memory to process address space */
    addr = mmap(NULL, STORAGE_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED)
    {
        AIM_LOG_ERROR("[onlp_vimi_cpld_bus_id_get] mmap failed\r\n");
        return 30;
    }

    /* Copy memory from addr to data */
    memcpy(data, addr, STORAGE_SIZE);

    /* Data processing */
    count = 0;
    char *split_date = strtok(data, " ");
    if(split_date)
    {
        pca9548_ch0_bus_id_list[count] = atoi(split_date);
        count ++;
    }


    while((split_date=strtok(NULL, " ")))
    {
        /* Use the first parameter as NULL to extract a substring */
        pca9548_ch0_bus_id_list[count] = atoi(split_date);
        count ++;
    }

    /* Get VIM CPLD I2C bus id */
    switch (vim_id)
    {
        case VIM1_ID:
            cpld_bus_id = pca9548_ch0_bus_id_list[0];
            break;

        case VIM2_ID:
            cpld_bus_id = pca9548_ch0_bus_id_list[4];
            break;
    }


    switch (cpld_id)
    {
        case VIM_POWER_CPLD_ID:
            cpld_bus_id += VIM_PWR_CPLD_INDEX;
            break;

        case VIM_PORT_CPLD_ID:
            cpld_bus_id += VIM_PROT_CPLD_INDEX;
            break;
    }

    /* mmap cleanup */
	res = munmap(addr, STORAGE_SIZE);
	if (res == -1)
	{
        AIM_LOG_ERROR("[update_vim_i2c_tree_db] munmap failed\r\n");
		return 40;
	}

    return cpld_bus_id;
}

int
onlp_vimi_optoe_start_port_get(int vim_id)
{
    int start_port, vim1_board_id;
    int vim1_present, vim2_present;

    vim1_present = onlp_vimi_present_get(VIM1_ID);
    vim2_present = onlp_vimi_present_get(VIM2_ID);

    if ((vim1_present == VIM_PRESENT) && (vim2_present == VIM_PRESENT))
    {
        switch (vim_id)
        {
            case VIM1_ID:
                start_port = VIM_START_INDEX;
                break;

            case VIM2_ID:
                vim1_board_id = onlp_vimi_board_id_get(VIM1_ID);
                start_port = VIM_START_INDEX + vim_struct[vim1_board_id].port_count;
                break;

            default:
                break;
        }
    }
    else if ((vim1_present == VIM_PRESENT) && (vim2_present == VIM_NOT_PRESENT))
    {
        switch (vim_id)
        {
            case VIM1_ID:
                /* If VIM not present, start_port is -1 */
                start_port = VIM_START_INDEX;
                break;

            case VIM2_ID:
                start_port = -1;
                break;

            default:
                break;
        }
    }
    else if ((vim1_present == VIM_NOT_PRESENT) && (vim2_present == VIM_PRESENT))
    {
        switch (vim_id)
        {
            case VIM1_ID:
                /* If VIM not present, start_port is -1 */
                start_port = -1;
                break;

            case VIM2_ID:
                start_port = VIM_START_INDEX;
                break;

            default:
                break;
        }
    }
    else if ((vim1_present == VIM_NOT_PRESENT) && (vim2_present == VIM_NOT_PRESENT))
    {
        /* If VIM not present, start_port is -1 */
        start_port = -1;
    }

    return start_port;
}


int
onlp_vimi_get_vim_end_index(void)
{
    int vim_end_index = VIM_START_INDEX;
    int board_id, i;
    for (i = 1; i <= VIM_ID_MAX; i++ )
    {
        board_id = onlp_vimi_board_id_get(i);
        vim_end_index += vim_struct[board_id].port_count;
    }

    return vim_end_index;
}


int
onlp_vimi_index_map_to_vim_id(int index)
{
    int board_id;
    int vim1_num_port, vim2_num_port;
    int i, vim_id;

    /* Confirm which board_id the port index belongs to */
    for (i = 1; i <= VIM_ID_MAX; i++ )
    {
        board_id = onlp_vimi_board_id_get(i);

        if (i == VIM1_ID)
        {
            vim1_num_port = vim_struct[board_id].port_count;
        }
        else if (i == VIM2_ID)
        {
            vim2_num_port = vim_struct[board_id].port_count;
        }
    }

    if ((vim2_num_port == 0)) /* vim2 is not present */
    {
        vim_id = VIM1_ID;
    }
    else if ((vim1_num_port == 0)) /* vim1 is not present */
    {
        vim_id = VIM2_ID;
    }
    else /* vim1 and vim2 are present */
    {
        /* port index belongs to vim1 port */
        if ((index >= VIM_START_INDEX) && (index < (VIM_START_INDEX + vim1_num_port)))
        {
            vim_id = VIM1_ID;
        }
        /* port index belongs to vim2 port */
        else if ((index >= (VIM_START_INDEX + vim1_num_port)) && (index < (VIM_START_INDEX + vim1_num_port + vim2_num_port)))
        {
            vim_id = VIM2_ID;
        }
        else
        {
            DIAG_PRINT("%s, port:%d out of range.", __FUNCTION__, index);
            return ONLP_STATUS_E_INTERNAL;
        }
    }

    return vim_id;
}


int
onlp_vimi_get_list_index(int vim_id, int index)
{
    int vim1_board_id, list_index, start_index;

    vim1_board_id = onlp_vimi_board_id_get(VIM1_ID);

    switch(vim_id)
    {
        case VIM1_ID:
            start_index = VIM_START_INDEX;
            break;

        case VIM2_ID:
            start_index = VIM_START_INDEX + vim_struct[vim1_board_id].port_count;
            break;
    }

    /* list_index start from 1 */
    list_index = index - start_index + 1;

    return list_index;
}

int *
onlp_vimi_get_optoe_bus_id_list(int vim_id)
{
    int i, j;
    int * optoe_bus_id;
    int count = 0;
    optoe_bus_id = malloc(sizeof(int)*24);
    int fd;
    char data[STORAGE_SIZE];
    void *addr;
    int res;


    /* Get shared memory file descriptor (not a file) */
    fd = shm_open(STORAGE_ID, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    if (fd == -1)
    {
        AIM_LOG_ERROR("[onlp_vimi_get_optoe_bus_id_list] shm_open failed\r\n");
    }

    /* Map shared memory to process address space */
    addr = mmap(NULL, STORAGE_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED)
    {
        AIM_LOG_ERROR("[onlp_vimi_get_optoe_bus_id_list] mmap failed\r\n");
    }

    /* Copy memory from addr to data */
    memcpy(data, addr, STORAGE_SIZE);

    /* Data processing */
    count = 0;
    char *split_date = strtok(data, " ");
    if(split_date)
    {
        pca9548_ch0_bus_id_list[count] = atoi(split_date);
        count ++;
    }

    while((split_date=strtok(NULL, " ")))
    {
        /* Use the first parameter as NULL to extract a substring */
        pca9548_ch0_bus_id_list[count] = atoi(split_date);
        count ++;
    }

    /* Get optoe_bus_id list */
    count = 0;
    switch (vim_id)
    {
        case VIM1_ID:
            /* pca9548_ch0_bus_id_list index 0~3 for VIM 1, optoe_bus_id_list need index 1~3(0x70, 0x71, 0x72) */
            for (i = 1; i < 4; i++)
            {
                if (pca9548_ch0_bus_id_list[i] != 0)
                {
                    for (j = pca9548_ch0_bus_id_list[i]; j < pca9548_ch0_bus_id_list[i] + I2C_BUS_CHANNEL_COUNT; j++)
                    {
                        optoe_bus_id[count] = j;
                        count += 1;
                    }
                }
            }
            break;

        case VIM2_ID:
            /* pca9548_ch0_bus_id_list index 4~7 for VIM 2, optoe_bus_id_list need index 5~7(0x70, 0x71, 0x72) */
            for (i = 5; i < 8; i++)
            {
                if (pca9548_ch0_bus_id_list[i] != 0)
                {
                    for (j = pca9548_ch0_bus_id_list[i]; j < pca9548_ch0_bus_id_list[i] + I2C_BUS_CHANNEL_COUNT; j++)
                    {
                        optoe_bus_id[count] = j;
                        count += 1;
                    }
                }
            }
            break;

        default:
            AIM_LOG_ERROR("[onlp_vimi_get_optoe_bus_id_list] vim id is out of range\r\n");
    }

    /* mmap cleanup */
	res = munmap(addr, STORAGE_SIZE);
	if (res == -1)
	{
        AIM_LOG_ERROR("[update_vim_i2c_tree_db] munmap failed\r\n");
	}

    return optoe_bus_id;
}

int
onlp_vim_sfpi_denit(void)
{
    DIAG_PRINT("%s", __FUNCTION__);
    return ONLP_STATUS_OK;
}

int
onlp_vim_sfpi_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);
    /* Called at initialization time */
    return ONLP_STATUS_OK;
}

int
onlp_vimi_eeprom_read(int vim_id, uint8_t data[256])
{
    /*
     * Read the VIM eeprom into data[]
     *
     * Return MISSING if VIM is missing.
     * Return OK if VIM eeprom is read
     */
    int size = 0;
    int vim_present = 0;

    vim_present = onlp_vimi_present_get(vim_id);

    if(vim_present == VIM_NOT_PRESENT)
    {
        AIM_LOG_ERROR("vim(%d) not present\r\n", vim_id);
        return ONLP_STATUS_OK;
    }

    if(onlp_file_read(data, 256, &size, VIM_EEPROM_PATH,
                	vim_id) != ONLP_STATUS_OK)
    {
        AIM_LOG_ERROR("Unable to read eeprom from vim(%d)\r\n", vim_id);
        return ONLP_STATUS_E_INTERNAL;
    }


    if(size != 256)
    {
        return ONLP_STATUS_E_INTERNAL;
    }

    return ONLP_STATUS_OK;
}

int
onlp_vim_sfpi_dev_readb(int port, uint8_t devaddr, uint8_t addr)
{
    int ret = 0;
    int bus;
    int vim_id, start_port;
    int *optoe_bus_id;
    vim_id = onlp_vimi_index_map_to_vim_id(port);
    if (vim_id == ONLP_STATUS_E_INTERNAL)
    {
        DIAG_PRINT("%s, port:%d out of range.", __FUNCTION__, port);
        return ONLP_STATUS_E_INVALID;
    }
    start_port = onlp_vimi_optoe_start_port_get(vim_id);
    optoe_bus_id = onlp_vimi_get_optoe_bus_id_list(vim_id);

    bus = optoe_bus_id[port-start_port];
    ret = onlp_i2c_readb(bus, devaddr, addr, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, ret:%d(0x%02X)", __FUNCTION__, port, devaddr, addr, ret, ret);
    return ret;
}

int
onlp_vim_sfpi_dev_writeb(int port, uint8_t devaddr, uint8_t addr, uint8_t value)
{
    int ret = 0;
    int bus;
    int vim_id, start_port;
    int *optoe_bus_id;
    vim_id = onlp_vimi_index_map_to_vim_id(port);
    if (vim_id == ONLP_STATUS_E_INTERNAL)
    {
        DIAG_PRINT("%s, port:%d out of range.", __FUNCTION__, port);
        return ONLP_STATUS_E_INVALID;
    }
    start_port = onlp_vimi_optoe_start_port_get(vim_id);
    optoe_bus_id = onlp_vimi_get_optoe_bus_id_list(vim_id);
    bus = optoe_bus_id[port-start_port];

    ret = onlp_i2c_writeb(bus, devaddr, addr, value, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, value:%d(0x%02X), ret:%d", __FUNCTION__, port, devaddr, addr, value, value, ret);
    return ret;
}

int
onlp_vim_sfpi_dev_readw(int port, uint8_t devaddr, uint8_t addr)
{
    int ret = 0;
    int bus;
    int vim_id, start_port;
    int *optoe_bus_id;
    vim_id = onlp_vimi_index_map_to_vim_id(port);
    if (vim_id == ONLP_STATUS_E_INTERNAL)
    {
        DIAG_PRINT("%s, port:%d out of range.", __FUNCTION__, port);
        return ONLP_STATUS_E_INVALID;
    }
    start_port = onlp_vimi_optoe_start_port_get(vim_id);
    optoe_bus_id = onlp_vimi_get_optoe_bus_id_list(vim_id);

    bus = optoe_bus_id[port-start_port];

    ret = onlp_i2c_readw(bus, devaddr, addr, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, ret:%d(0x%04X)", __FUNCTION__, port, devaddr, addr, ret, ret);
    return ret;
}

int
onlp_vim_sfpi_dev_writew(int port, uint8_t devaddr, uint8_t addr, uint16_t value)
{
    int ret = 0;
    int bus;
    int vim_id, start_port;
    int *optoe_bus_id;
    vim_id = onlp_vimi_index_map_to_vim_id(port);
    if (vim_id == ONLP_STATUS_E_INTERNAL)
    {
        DIAG_PRINT("%s, port:%d out of range.", __FUNCTION__, port);
        return ONLP_STATUS_E_INVALID;
    }
    start_port = onlp_vimi_optoe_start_port_get(vim_id);
    optoe_bus_id = onlp_vimi_get_optoe_bus_id_list(vim_id);

    bus = optoe_bus_id[port-start_port];

    ret = onlp_i2c_writew(bus, devaddr, addr, value, ONLP_I2C_F_FORCE);
    DIAG_PRINT("%s, port:%d, devaddr:%d, addr:%d, value:%d(0x%04X), ret:%d", __FUNCTION__, port, devaddr, addr, value, value, ret);
    return ret;
}

int
onlp_vim_sfpi_control_set(int port, onlp_sfp_control_t control, int value)
{
    int rv;
    int supported = 0;
    int cpld_bus_id, list_index;
    int vim_id, board_id;
    char format[128];

    if ((onlp_sfpi_control_supported(port, control, &supported) == ONLP_STATUS_OK) &&
        (supported == 0))
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
        return ONLP_STATUS_E_UNSUPPORTED;
    }

    DIAG_PRINT("%s, port:%d, control:%d(%s), value:0x%X", __FUNCTION__, port, control, vim_sfp_control_to_str(control), value);

    vim_id = onlp_vimi_index_map_to_vim_id(port);
    if (vim_id == ONLP_STATUS_E_INTERNAL)
    {
        DIAG_PRINT("%s, port:%d out of range.", __FUNCTION__, port);
        return ONLP_STATUS_E_INVALID;
    }
    list_index = onlp_vimi_get_list_index(vim_id, port);
    board_id = onlp_vimi_board_id_get(vim_id);

    if ((board_id == VIM_24CE || board_id == VIM_24YE) && list_index > 12)
    {
        /* Access VIM Port CPLD from CPU */
        cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_PORT_CPLD_ID);

        switch(control)
        {
            case ONLP_SFP_CONTROL_RESET:
                strcpy(format, VIM_OPTOE_RESET_PORT_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_LP_MODE:
                strcpy(format, VIM_OPTOE_LP_MODE_PORT_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_TX_DISABLE:
                strcpy(format, VIM_OPTOE_TX_DIS_PORT_CPLD_PATH);
                break;
            default:
                rv = ONLP_STATUS_E_UNSUPPORTED;
                break;
        }
        if (onlp_file_write_int(value, format, cpld_bus_id, list_index) < 0)
        {
            AIM_LOG_ERROR("Unable to set reset status to port(%d). Path: %s\r\n", port, format);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        else
        {
            rv = ONLP_STATUS_OK;
        }
    }
    else
    {
        /* Access VIM Power CPLD from CPU */
        cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_POWER_CPLD_ID);

        switch(control)
        {
            case ONLP_SFP_CONTROL_RESET:
                strcpy(format, VIM_OPTOE_RESET_PWR_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_LP_MODE:
                strcpy(format, VIM_OPTOE_LP_MODE_PWR_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_TX_DISABLE:
                strcpy(format, VIM_OPTOE_TX_DIS_PWR_CPLD_PATH);
                break;

            default:
                rv = ONLP_STATUS_E_UNSUPPORTED;
                break;
        }

        if (onlp_file_write_int(value, format, cpld_bus_id, list_index) < 0)
        {
            AIM_LOG_ERROR("Unable to set reset status to port(%d). Path: %s\r\n", port, format);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        else
        {
            rv = ONLP_STATUS_OK;
        }
    }

    DIAG_PRINT("%s, Write VIM port:%d %s value:0x%x, ", __FUNCTION__, port, vim_sfp_control_to_str(control), value);

    return rv;
}

int
onlp_vim_sfpi_control_get(int port, onlp_sfp_control_t control, int* value)
{
    int rv;
    int val;
    int supported = 0;
    int cpld_bus_id, list_index;
    int vim_id, board_id;
    char format[128];

    if (value == NULL)
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_PARAM);
        return ONLP_STATUS_E_PARAM;
    }

    if ((onlp_sfpi_control_supported(port, control, &supported) == ONLP_STATUS_OK) &&
        (supported == 0))
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
        return ONLP_STATUS_E_UNSUPPORTED;
    }
    *value = 0;

    /* ONLP_SFP_CONTROL_RX_LOS , ONLP_SFP_CONTROL_TX_FAULT are read-only. */
    vim_id = onlp_vimi_index_map_to_vim_id(port);
    if (vim_id == ONLP_STATUS_E_INTERNAL)
    {
        DIAG_PRINT("%s, port:%d out of range.", __FUNCTION__, port);
        return ONLP_STATUS_E_INVALID;
    }

    list_index = onlp_vimi_get_list_index(vim_id, port);
    board_id = onlp_vimi_board_id_get(vim_id);

    if ((board_id == VIM_24CE || board_id == VIM_24YE) && list_index > 12)
    {
        /* Access VIM Port CPLD from CPU */
        cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_PORT_CPLD_ID);

        switch(control)
        {
            case ONLP_SFP_CONTROL_RESET:
                strcpy(format, VIM_OPTOE_RESET_PORT_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_LP_MODE:
                strcpy(format, VIM_OPTOE_LP_MODE_PORT_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_TX_DISABLE:
                strcpy(format, VIM_OPTOE_TX_DIS_PORT_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_RX_LOS:
                strcpy(format, VIM_OPTOE_RX_LOSS_PORT_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_TX_FAULT:
                strcpy(format, VIM_OPTOE_TX_FAULT_PORT_CPLD_PATH);
                break;

            default:
                rv = ONLP_STATUS_E_UNSUPPORTED;
                break;
        }

        if (onlp_file_read_int(&val, format, cpld_bus_id, list_index) < 0)
        {
            AIM_LOG_ERROR("Unable to read %s status from port(%d). path: %s\r\n", vim_sfp_control_to_str(control), port, format);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        else
        {
            rv = ONLP_STATUS_OK;
        }
    }
    else
    {
        /* Access VIM Power CPLD from CPU */
        cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_POWER_CPLD_ID);

        switch(control)
        {
            case ONLP_SFP_CONTROL_RESET:
                strcpy(format, VIM_OPTOE_RESET_PWR_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_LP_MODE:
                strcpy(format, VIM_OPTOE_LP_MODE_PWR_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_TX_DISABLE:
                strcpy(format, VIM_OPTOE_TX_DIS_PWR_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_RX_LOS:
                strcpy(format, VIM_OPTOE_RX_LOSS_PWR_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_TX_FAULT:
                strcpy(format, VIM_OPTOE_TX_FAULT_PWR_CPLD_PATH);
                break;

            default:
                rv = ONLP_STATUS_E_UNSUPPORTED;
                break;
        }

        if (onlp_file_read_int(&val, format, cpld_bus_id, list_index) < 0)
        {
            AIM_LOG_ERROR("Unable to read %s status from port(%d). path: %s\r\n", vim_sfp_control_to_str(control), port, format);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        else
        {
            rv = ONLP_STATUS_OK;
        }
    }
    *value = val;

    DIAG_PRINT("%s, port:%d, control:%d(%s), value:0x%X", __FUNCTION__, port, control, vim_sfp_control_to_str(control), *value);

    if (rv == ONLP_STATUS_E_UNSUPPORTED)
    {
        AIM_LOG_INFO("%s, port:%d, control:%d(%s) UNSUPPORTED(%d)\n", __FUNCTION__, port, control, vim_sfp_control_to_str(control), ONLP_STATUS_E_UNSUPPORTED);
    }

    return rv;
}

/* Private function based on VIM 24CE */
int
onlp_vim_sfpi_control_set_for_b_attr(int port, onlp_sfp_control_for_b_attr_t control, int value)
{
    int rv;
    int supported = 0;
    int cpld_bus_id, list_index;
    int vim_id, board_id;
    char format[128];

    if ((onlp_sfpi_control_supported_for_b_attr(port, control, &supported) == ONLP_STATUS_OK) &&
        (supported == 0))
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
        return ONLP_STATUS_E_UNSUPPORTED;
    }

    DIAG_PRINT("%s, port:%d, control:%d(%s), value:0x%X", __FUNCTION__, port, control, vim_sfp_control_for_b_attr_to_str(control), value);

    vim_id = onlp_vimi_index_map_to_vim_id(port);
    if (vim_id == ONLP_STATUS_E_INTERNAL)
    {
        DIAG_PRINT("%s, port:%d out of range.", __FUNCTION__, port);
        return ONLP_STATUS_E_INVALID;
    }
    list_index = onlp_vimi_get_list_index(vim_id, port);
    board_id = onlp_vimi_board_id_get(vim_id);

    if (board_id == VIM_24CE && list_index > 12)
    {
        /* Access VIM Port CPLD from CPU */
        cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_PORT_CPLD_ID);

        switch(control)
        {
            case ONLP_SFP_CONTROL_TX_DISABLE_B:
                strcpy(format, VIM_OPTOE_TX_DIS_B_PORT_CPLD_PATH);
                break;
            default:
                rv = ONLP_STATUS_E_UNSUPPORTED;
                break;
        }
        if (onlp_file_write_int(value, format, cpld_bus_id, list_index) < 0)
        {
            AIM_LOG_ERROR("Unable to set reset status to port(%d). Path: %s\r\n", port, format);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        else
        {
            rv = ONLP_STATUS_OK;
        }
    }
    else if (board_id == VIM_24CE && list_index <= 12)
    {
        /* Access VIM Power CPLD from CPU */
        cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_POWER_CPLD_ID);

        switch(control)
        {
            case ONLP_SFP_CONTROL_TX_DISABLE_B:
                strcpy(format, VIM_OPTOE_TX_DIS_B_PWR_CPLD_PATH);
                break;

            default:
                rv = ONLP_STATUS_E_UNSUPPORTED;
                break;
        }

        if (onlp_file_write_int(value, format, cpld_bus_id, list_index) < 0)
        {
            AIM_LOG_ERROR("Unable to set reset status to port(%d). Path: %s\r\n", port, format);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        else
        {
            rv = ONLP_STATUS_OK;
        }
    }
    else
    {
        rv = ONLP_STATUS_E_UNSUPPORTED;
    }

    DIAG_PRINT("%s, Write VIM port:%d %s value:0x%x, ", __FUNCTION__, port, vim_sfp_control_for_b_attr_to_str(control), value);

    return rv;
}

/* Private function based on VIM 24CE */
int
onlp_vim_sfpi_control_get_for_b_attr(int port, onlp_sfp_control_for_b_attr_t control, int* value)
{
    int rv;
    int val;
    int supported = 0;
    int cpld_bus_id, list_index;
    int vim_id, board_id;
    char format[128];

    if (value == NULL)
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_PARAM);
        return ONLP_STATUS_E_PARAM;
    }

    if ((onlp_sfpi_control_supported_for_b_attr(port, control, &supported) == ONLP_STATUS_OK) &&
        (supported == 0))
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
        return ONLP_STATUS_E_UNSUPPORTED;
    }
    *value = 0;

    /* ONLP_SFP_CONTROL_RX_LOS , ONLP_SFP_CONTROL_TX_FAULT are read-only. */
    vim_id = onlp_vimi_index_map_to_vim_id(port);
    if (vim_id == ONLP_STATUS_E_INTERNAL)
    {
        DIAG_PRINT("%s, port:%d out of range.", __FUNCTION__, port);
        return ONLP_STATUS_E_INVALID;
    }

    list_index = onlp_vimi_get_list_index(vim_id, port);
    board_id = onlp_vimi_board_id_get(vim_id);

    if (board_id == VIM_24CE && list_index > 12)
    {
        /* Access VIM Port CPLD from CPU */
        cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_PORT_CPLD_ID);

        switch(control)
        {
            case ONLP_SFP_CONTROL_TX_DISABLE_B:
                strcpy(format, VIM_OPTOE_TX_DIS_B_PORT_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_RX_LOS_B:
                strcpy(format, VIM_OPTOE_RX_LOSS_B_PORT_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_TX_FAULT_B:
                strcpy(format, VIM_OPTOE_TX_FAULT_B_PORT_CPLD_PATH);
                break;

            default:
                rv = ONLP_STATUS_E_UNSUPPORTED;
                break;
        }

        if (onlp_file_read_int(&val, format, cpld_bus_id, list_index) < 0)
        {
            AIM_LOG_ERROR("Unable to read %s status from port(%d). path: %s\r\n", vim_sfp_control_for_b_attr_to_str(control), port, format);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        else
        {
            rv = ONLP_STATUS_OK;
        }
    }
    else if (board_id == VIM_24CE && list_index <= 12)
    {
        /* Access VIM Power CPLD from CPU */
        cpld_bus_id = onlp_vimi_cpld_bus_id_get(vim_id, VIM_POWER_CPLD_ID);

        switch(control)
        {
            case ONLP_SFP_CONTROL_TX_DISABLE_B:
                strcpy(format, VIM_OPTOE_TX_DIS_B_PWR_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_RX_LOS_B:
                strcpy(format, VIM_OPTOE_RX_LOSS_B_PWR_CPLD_PATH);
                break;
            case ONLP_SFP_CONTROL_TX_FAULT_B:
                strcpy(format, VIM_OPTOE_TX_FAULT_B_PWR_CPLD_PATH);
                break;

            default:
                rv = ONLP_STATUS_E_UNSUPPORTED;
                break;
        }

        if (onlp_file_read_int(&val, format, cpld_bus_id, list_index) < 0)
        {
            AIM_LOG_ERROR("Unable to read %s status from port(%d). path: %s\r\n", vim_sfp_control_for_b_attr_to_str(control), port, format);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        else
        {
            rv = ONLP_STATUS_OK;
        }
    }
    else
    {
        rv = ONLP_STATUS_E_UNSUPPORTED;
    }
    *value = val;

    DIAG_PRINT("%s, port:%d, control:%d(%s), value:0x%X", __FUNCTION__, port, control, vim_sfp_control_for_b_attr_to_str(control), *value);

    if (rv == ONLP_STATUS_E_UNSUPPORTED)
    {
        AIM_LOG_INFO("%s, port:%d, control:%d(%s) UNSUPPORTED(%d)\n", __FUNCTION__, port, control, vim_sfp_control_for_b_attr_to_str(control), ONLP_STATUS_E_UNSUPPORTED);
    }

    return rv;
}

/* Private function based on VIM 24CE (sfp) */
int
onlp_sfpi_control_supported_for_b_attr(int port, onlp_sfp_control_for_b_attr_t control, int *supported)
{
    int vim_end_index = onlp_vimi_get_vim_end_index();
    int vim_id, board_id;

    if(IS_VIM_PORT(port, vim_end_index))
    {
        vim_id = onlp_vimi_index_map_to_vim_id(port);
        if (vim_id == ONLP_STATUS_E_INTERNAL)
        {
            DIAG_PRINT("%s, port:%d out of range.", __FUNCTION__, port);
            return ONLP_STATUS_E_INVALID;
        }
        board_id = onlp_vimi_board_id_get(vim_id);
    }
    else
    {
        return ONLP_STATUS_E_INVALID;
    }


    if (supported == NULL)
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_PARAM);
        return ONLP_STATUS_E_PARAM;
    }

    *supported = 0;
    switch (control)
    {
        case ONLP_SFP_CONTROL_RX_LOS_B:
        case ONLP_SFP_CONTROL_TX_FAULT_B:
        case ONLP_SFP_CONTROL_TX_DISABLE_B:
        {
            switch (board_id)
            {
                case VIM_8DE:
                case VIM_16CE:
                case VIM_24YE:
                    /* QSFP28 100G and QSFPDD 400G*/
                    *supported = 0;
                    break;
                case VIM_24CE:
                    /* SFP-DD and SFP28 (SFF-8472) */
                    *supported = 1;
                    break;
                default:
                    break;
            }
        }
            break;

        default:
            *supported = 0;
            break;
    }

    DIAG_PRINT("%s, port:%d, control:%d(%s), supported:%d", __FUNCTION__, port, control, vim_sfp_control_for_b_attr_to_str(control), *supported);

    return ONLP_STATUS_OK;
}


/* Private function based on VIM 24CE (sfp) */
int
onlp_sfpi_control_set_for_b_attr(int port, onlp_sfp_control_for_b_attr_t control, int value)
{
    int rv;
    int supported = 0;
    int vim_end_index = onlp_vimi_get_vim_end_index();

    if ((onlp_sfpi_control_supported_for_b_attr(port, control, &supported) == ONLP_STATUS_OK) &&
        (supported == 0))
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
        return ONLP_STATUS_E_UNSUPPORTED;
    }

    DIAG_PRINT("%s, port:%d, control:%d(%s), value:0x%X", __FUNCTION__, port, control, vim_sfp_control_for_b_attr_to_str(control), value);

    /* ONLP_SFP_CONTROL_RESET: write-only. */
    switch(control)
    {
        case ONLP_SFP_CONTROL_TX_DISABLE_B:
        {
            if(IS_VIM_PORT(port, vim_end_index))
            {
                rv = onlp_vim_sfpi_control_set_for_b_attr(port, control, value);
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }

        default:
            rv = ONLP_STATUS_E_UNSUPPORTED;
            break;
    }

    if (rv == ONLP_STATUS_E_UNSUPPORTED)
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
    }

    return rv;
}


/* Private function based on VIM 24CE (sfp) */
int
onlp_sfpi_control_get_for_b_attr(int port, onlp_sfp_control_for_b_attr_t control, int* value)
{
    int rv;
    int supported = 0;
    int vim_end_index = onlp_vimi_get_vim_end_index();

    if (value == NULL)
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_PARAM);
        return ONLP_STATUS_E_PARAM;
    }

    if ((onlp_sfpi_control_supported_for_b_attr(port, control, &supported) == ONLP_STATUS_OK) &&
        (supported == 0))
    {
        AIM_LOG_INFO("%s:%d fail[%d]\n", __FUNCTION__, __LINE__, ONLP_STATUS_E_UNSUPPORTED);
        return ONLP_STATUS_E_UNSUPPORTED;
    }
    *value = 0;

    /* ONLP_SFP_CONTROL_RX_LOS , ONLP_SFP_CONTROL_TX_FAULT are read-only. */
    switch(control)
    {
        case ONLP_SFP_CONTROL_RX_LOS_B:
        case ONLP_SFP_CONTROL_TX_FAULT_B:
        case ONLP_SFP_CONTROL_TX_DISABLE_B:
        {
            if(IS_VIM_PORT(port, vim_end_index))
            {
                rv = onlp_vim_sfpi_control_get_for_b_attr(port, control, value);
            }
            else
            {
                rv = ONLP_STATUS_E_UNSUPPORTED;
            }
            break;
        }

        default:
            rv = ONLP_STATUS_E_UNSUPPORTED;
    }

    DIAG_PRINT("%s, port:%d, control:%d(%s), value:0x%X", __FUNCTION__, port, control, vim_sfp_control_for_b_attr_to_str(control), *value);

    if (rv == ONLP_STATUS_E_UNSUPPORTED)
    {
        AIM_LOG_INFO("%s:%d port = %d, control = %d fail[%d]\n", __FUNCTION__, __LINE__, port, control, ONLP_STATUS_E_UNSUPPORTED);
    }

    return rv;
}

int
onlp_vimi_power_good_get(int vim_id)
{
    int vim_power_good = -1;

    if (onlp_file_read_int(&vim_power_good, VIM_POWER_GOOD_PATH, vim_id) < 0)
    {
        AIM_LOG_ERROR("Get vim power good failed\r\n");
        return ONLP_STATUS_E_INTERNAL;
    }

    return vim_power_good;
}

int
onlp_vimi_power_control(int vim_power_operation, int vim_id)
{
    int rv = 0;
    int vim_power_good = -1;
    vim_power_good = onlp_vimi_power_good_get(vim_id);

    /* VIM power on */
    if ((vim_power_good == VIM_POWER_FAIL) && (vim_power_operation == ON))
    {
        /* Chech VIM present == 0 */
        rv = onlp_vimi_present_get(vim_id);
        if (rv < 0){
            AIM_LOG_ERROR("Get vim %d present failed(%d)\r\n", vim_id, rv);
            return ONLP_STATUS_E_INTERNAL;
        }

        if (rv == VIM_NOT_PRESENT){
            AIM_LOG_ERROR("vim %d is not present, Enable VIM power failed\r\n", vim_id);
            return ONLP_STATUS_E_INVALID;
        }

        /*  Enable VIM's MP5990 */
        if (onlp_file_write_int(VIM_PWR_CTRL_ENA_MP5990, VIM_POWER_CONTROL_PATH, vim_id) < 0)
        {
            AIM_LOG_ERROR("Enable VIM%d's MP5990 failed\r\n", vim_id);
            return ONLP_STATUS_E_INTERNAL;
        }

        /*  Wait 50ms */
        usleep(50000);

        /* Enable VIM's DCDC power */
        if (onlp_file_write_int(VIM_PWR_CTRL_ENA_DCDC, VIM_POWER_CONTROL_PATH, vim_id) < 0)
        {
            AIM_LOG_ERROR("Enable VIM%d's DCDC power failed\r\n", vim_id);
            return ONLP_STATUS_E_INTERNAL;
        }

        return ONLP_STATUS_OK;
    }

    /* VIM power off */
    if ((vim_power_good == VIM_POWER_GOOD) && (vim_power_operation == OFF))
    {
        /* Disable VIM1's MP5990 and DCDC */
        if (onlp_file_write_int(VIM_PWR_CTRL_DIS_PWR, VIM_POWER_CONTROL_PATH, vim_id) < 0)
        {
            AIM_LOG_ERROR("Disable VIM%d's MP5990 and DCDC power failed\r\n", vim_id);
            return ONLP_STATUS_E_INTERNAL;
        }

        return ONLP_STATUS_OK;
    }

    if ( !(vim_power_operation == ON || vim_power_operation == OFF))
    {
        /* If vim_power_operation is not ON or OFF, return error code */
        AIM_LOG_ERROR("VIM power operation not support\r\n");
        return ONLP_STATUS_E_INTERNAL;
    }
    else
    {
        if (vim_power_good < 0)
        {
            return ONLP_STATUS_E_INTERNAL;
        }
        else
        {
            /* VIM does not need to enable/disable VIM power */
            return ONLP_STATUS_OK;
        }
    }

    /*  Wait 50 ms to VIM ready */
    usleep(50000);
}

