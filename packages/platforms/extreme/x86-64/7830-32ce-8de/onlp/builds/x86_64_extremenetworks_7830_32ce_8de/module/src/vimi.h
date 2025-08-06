/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *        Copyright 2014, 2015 Big Switch Networks, Inc.
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
 * VIM Platform Implementation Interface.
 *
 * DSCRIPTION
 *
 ***********************************************************/
#ifndef __ONLP_VIMI_H__
#define __ONLP_VIMI_H__

#include <onlp/onlp.h>
#include <onlp/onlp_config.h>
#include <onlp/sfp.h>
#include <sff/sff.h>

#define PORT_TYPE_SIZE                      7

#if (DEBUG_FLAG_SAMPLE == 0)
/*
 * VIM information for HW design
 * It contains how many PCA9548 have this VIM used,
 * the number of port supported and the type of Optoe
 */
typedef struct vim_info_s {
    int pca_count;
    int port_count;
    char port_type[PORT_TYPE_SIZE];
} vim_info;
#endif /* DEBUG_FLAG_SAMPLE */

/* Define callback function prototype */
typedef int (*VIM_CALLBACK_FUNC)(
    int vim_id,
    int operation,
    int vim_board_id);


/* Record data structure of callback function pointer */
typedef struct {
    bool valid;
    VIM_CALLBACK_FUNC callback;
} vim_callback_info_t;

enum vim_id
{
    VIM_1,
    VIM_2,
    VIM_ID_MAX
};

enum operation
{
    INSERT,
    REMOVE
};

enum vim_power_operation
{
    ON,
    OFF
};

enum boolean
{
    FALSE,
    TRUE
};

enum vim_use_status
{
    VIM_1_USE,
    VIM_2_USE,
    NOT_USE
};

enum pca9548_index
{
    PCA9548_1,
    PCA9548_2,
    PCA9548_3,
    PCA9548_4
};

/*
 * Extreme needs to record the VIM i2c tree information
 * after updating the i2c tree. VIM i2c tree information
 * include CH0 i2c bus id for each VIM i2c PCA9548. Alpha
 * needs this information to complete the ONLPI API.
*/
typedef struct onlp_vim_i2c_info {

    /* CH0 i2c bus id for all VIM PCA9548 */
    uint32_t PCA9548_1_ch0_i2c_bus_id;  /* (PCA9548#1) 0x76 */
    uint32_t PCA9548_2_ch0_i2c_bus_id;  /* (PCA9548#0) 0x70 */
    uint32_t PCA9548_3_ch0_i2c_bus_id;  /* (PCA9548#3) 0x71 */
    uint32_t PCA9548_4_ch0_i2c_bus_id;  /* (PCA9548#4) 0x72 */

} onlp_vim_i2c_info;


typedef struct {
    onlp_vim_i2c_info info[VIM_ID_MAX];
    sem_t mutex;
} onlp_vim_i2c_info_t;


/**
 * @brief Register callback function.
 * @param callback Callback function that needs to be registered.
 * @returns TRUE(1) if success
 * @returns FALSE(0) if failed
 */
bool vim_register_callback(VIM_CALLBACK_FUNC callback);

/**
 * @brief Deregister callback function.
 * @returns TRUE(1) if finish
 */
bool vim_deregister_callback(void);

/**
 * @brief Execute register callback function.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @param operation VIM is inserted/removed. (insert = 0, remove = 1)
 * @param vim_board_id The VIM board type. (8DE = 0, 16CE = 1, 24CE = 2, 24YE = 3)
 * @returns TRUE(1) if success
 * @returns FALSE(0) if failed
 */
bool execute_update_vim_i2c_tree(int vim_id, int operation, int vim_board_id);

/**
 * @brief Update VIM I2C tree when VIM is inserted/removed.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @param operation VIM is inserted/removed. (insert = 0, remove = 1)
 * @param vim_board_id The VIM board type. (8DE = 0, 16CE = 1, 24CE = 2, 24YE = 3)
 * @returns 0 if update VIM I2C tree success, otherwise it failed.
 */
int extreme_update_vim_i2c_tree_sample(int vim_id, int operation, int vim_board_id);

/**
 * @brief Update VIM I2C tree db after update VIM I2C tree.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @param current_vim_i2c_info VIM i2c tree information.
 * @returns 0 if create shared memory success, otherwise it failed.
 */
int update_vim_i2c_tree_db(int vim_id, onlp_vim_i2c_info_t *current_vim_i2c_info);

/**
 * @brief Initialize the VIMI subsystem.
 */
int onlp_vimi_init(void);

/**
 * @brief Get the VIM present.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @returns 1 if absent \
 * @returns 0 if present    \
 * @returns An error condition.
 */
int onlp_vimi_present_get(int vim_id);

/**
 * @brief Get the VIM board type.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @returns The VIM board type. (8DE = 0, 16CE = 1, 24CE = 2, 24YE = 3)
 */
int onlp_vimi_board_id_get(int vim_id);

/**
 * @brief [private] Get the VIM port CPLD i2c bus id.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @param cpld_id The VIM CPLD ID. (Power CPLD = 1, Port CPLD = 2)
 * @returns The VIM port CPLD i2c bus id.
 */
int onlp_vimi_cpld_bus_id_get(int vim_id, int cpld_id);

/**
 * @brief [private] optoe_bus_id_list array index need start_port.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @returns start port index
 */
int onlp_vimi_optoe_start_port_get(int vim_id);

/**
 * @brief [private] Using different types of VIM cards will change the scope of VIM SFP. \
 * @brief This function is used to calculate the effective range of the current VIM SFP.
 * @returns vim_end_index : VIM port max index.
 * @returns IN Athena, 0 ~ 41 is Front port index, 42 ~ (vim_end_index-1) is VIM port index
 */
int onlp_vimi_get_vim_end_index(void);

/**
 * @brief [private] Use index to map which VIM slot the port belongs to.
 * @param index index start from 0, front port(0-41), vim port(42-(vim_end_index-1))
 * @returns vim_id: The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 */
int onlp_vimi_index_map_to_vim_id(int index);

/**
 * @brief [private] Used vim_id and index to get the list_index.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @param index index start from 0, front port(0-41), vim port(42-(vim_end_index-1))
 * @returns list_index: rst_mod_%d, %d is list_index. list_index start from 1
 */
int onlp_vimi_get_list_index(int vim_id, int index);

/**
 * @brief [private] Get vim optoe bus id list according to vim id.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @returns optoe_bus_id: optoe bus id array list
 */
int * onlp_vimi_get_optoe_bus_id_list(int vim_id);

/**
 * @brief Deinitialize the VIM SFP driver.
 */
int onlp_vim_sfpi_denit(void);

/**
 * @brief Initialize the VIM SFPI subsystem.
 */
int onlp_vim_sfpi_init(void);

/**
 * @brief Read VIM EEPROM.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @param data VIM EEPRIM
 * @returns < 0 if read failed.
 */
int onlp_vimi_eeprom_read(int vim_id, uint8_t data[256]);

/**
 * @brief Read a byte from an address on the given VIM SFP port's bus.
 * @param port The port number.
 * @param devaddr The device address.
 * @param addr The address.
 * @returns return 0 if read a byte successful, error otherwise.
 */
int onlp_vim_sfpi_dev_readb(int port, uint8_t devaddr, uint8_t addr);

/**
 * @brief Write a byte to an address on the given VIM SFP port's bus.
 * @param port The port number.
 * @param devaddr The device address.
 * @param addr The address.
 * @param value Write value.
 * @returns The byte if successful, error otherwise.
 */
int onlp_vim_sfpi_dev_writeb(int port, uint8_t devaddr, uint8_t addr, uint8_t value);

/**
 * @brief Read a word from an address on the given VIM SFP port's bus.
 * @param port The port number.
 * @param devaddr The device address.
 * @param addr The address.
 * @returns return 0 if read a word successful, error otherwise.
 */
int onlp_vim_sfpi_dev_readw(int port, uint8_t devaddr, uint8_t addr);

/**
 * @brief Write a word to an address on the given VIM SFP port's bus.
 * @param port The port number.
 * @param devaddr The device address.
 * @param addr The address.
 * @param value Write value.
 * @returns The word if successful, error otherwise.
 */
int onlp_vim_sfpi_dev_writew(int port, uint8_t devaddr, uint8_t addr, uint16_t value);

/**
 * @brief Set an VIM SFP control.
 * @param port The port.
 * @param control The control.
 * @param value The value.
 * @returns The word if successful, error otherwise.
 */
int onlp_vim_sfpi_control_set(int port, onlp_sfp_control_t control, int value);

/**
 * @brief Get an VIM SFP control.
 * @param port The port.
 * @param control The control
 * @param value [out] Receives the current value.
 * @returns The word if successful, error otherwise.
 */
int onlp_vim_sfpi_control_get(int port, onlp_sfp_control_t control, int* value);

/**
 * @brief Set an VIM SFP control for _b attribute.
 * @param port The port.
 * @param control The control.
 * @param value The value.
 * @returns The word if successful, error otherwise.
 */
int onlp_vim_sfpi_control_set_for_b_attr(int port, onlp_sfp_control_for_b_attr_t control, int value);

/**
 * @brief Get an VIM SFP control for _b attribute.
 * @param port The port.
 * @param control The control
 * @param value [out] Receives the current value.
 * @returns The word if successful, error otherwise.
 */
int onlp_vim_sfpi_control_get_for_b_attr(int port, onlp_sfp_control_for_b_attr_t control, int* value);

int onlp_sfpi_control_supported_for_b_attr(int port, onlp_sfp_control_for_b_attr_t control, int *supported);
int onlp_sfpi_control_set_for_b_attr(int port, onlp_sfp_control_for_b_attr_t control, int value);
int onlp_sfpi_control_get_for_b_attr(int port, onlp_sfp_control_for_b_attr_t control, int* value);

/**
 * @brief On/Off VIM power.
 * @param vim_power_operation ON or OFF.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @returns return 0 if On/Off VIM power successful, error otherwise.
 */
int onlp_vimi_power_control(int vim_power_operation, int vim_id);

/**
 * @brief Get VIM power good.
 * @param vim_id The VIM ID. (VIM 1 = 1, VIM 2 = 2)
 * @returns return 1 if VIM power is good, return 0 if VIM power is failed, error otherwise.
 */
int onlp_vimi_power_good_get(int vim_id);
#endif /* __ONLP_SFPI_H__ */
