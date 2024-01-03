from onl.platform.base import *
from onl.platform.extreme import *

class OnlPlatform_x86_64_extremenetworks_7830_32ce_8de_r0(OnlPlatformExtremeNetworks,
                                              OnlPlatformPortConfig_8x400_33x100_1x10):
    PLATFORM='x86-64-extremenetworks-7830-32ce-8de-r0'
    MODEL="7830-32ce-8de"
    SYS_OBJECT_ID=".6040.8"

    def baseconfig(self):
    	for m in [ 'onie_eeprom', 'cpld', 'thermal', 'fan', 'psu', 'pwr_cpld', 'bmc_led', 'cpu_led', 'iobm_io_eeprom' ]:
            self.insmod("7830-32ce-8de_%s.ko" % m)

        ########### initialize I2C bus 0 ###########
        self.new_i2c_devices([            
            
            # System CPLD
            ('system_cpld', 0x6E, 0),

            # CPU Board G751 (Ambient)
            #('lm75', 0x4F, 0),

            # initialize multiplexer (PCA9548 #8)
            ('pca9548', 0x77, 0),

            # power CPLD
            ('power_cpld', 0x5f, 0),		

            ])

        ########### initialize I2C bus PCA9548 #8 ###########
        self.new_i2c_devices(
            [

            # reserved

            # initialize multiplexer (Front port PCA9548 #0)
            ('pca9548', 0x70, 4),

            # reserved STM32L562
            
            ])

        ########### initialize I2C bus PCA9548 #0 (Front port) ###########
        self.new_i2c_devices(
            [
            
            # reserved

            # initialize multiplexer (PCA9548 #2)
            ('pca9548', 0x72, 10),

            # initialize multiplexer (PCA9548 #3)
            ('pca9548', 0x73, 11),

            # Port CPLD 0
            ('7830_32ce_8de_cpld1', 0x57, 12),

            # initialize multiplexer (PCA9548 #4)
            ('pca9548', 0x74, 13),

            # initialize multiplexer (PCA9548 #5)
            ('pca9548', 0x75, 14),

            # Port CPLD 1
            ('7830_32ce_8de_cpld2', 0x57, 15),

            # initialize multiplexer (PCA9548 #6)
            ('pca9548', 0x76, 16),
            ])

        ########### initialize I2C bus PCA9548 #2 #3 #4 #5 ###########        

        # Initialize QSFP28 devices (Port EEPROM)
        for port in range(1, 33):
            self.new_i2c_device('optoe1', 0x50, port+16)

        ########### initialize I2C bus PCA9548 #6 ###########  

        # Initialize QSFP-DD devices (Port EEPROM)
        for port in range(33, 41):
            self.new_i2c_device('optoe3', 0x50, port+16)

        ########### modprobe i2c-ismt and initialize I2C bus SMB HOST ###########
        os.system("modprobe i2c-ismt")
        self.new_i2c_devices([

            # DPLL 8A34001
            ('8a34001', 0x5B, 57),
            
            # ONIE EEPROM
            ('7830_onie_eeprom', 0x55, 57),            		

            ])


        return True
