from onl.platform.base import *
from onl.platform.extreme import *

class OnlPlatform_x86_64_extremenetworks_8730_32d_r0(OnlPlatformExtremeNetworks,
                                              OnlPlatformPortConfig_32x400_1x100_2x10):
    PLATFORM='x86-64-extremenetworks-8730-32d-r0'
    MODEL="8730-32d"
    SYS_OBJECT_ID=".6040.8"

    def baseconfig(self):
        self.insmod('8730-32d_onie_eeprom')
        self.insmod('8730-32d_cpld')

        ########### initialize I2C bus 0 ###########
        self.new_i2c_devices([            
            
            # 1588 CPLD
            ('1588_cpld', 0x6E, 0),

            # CPU Board G751 (Ambient)
            #('lm75', 0x4F, 0),

            # initialize multiplexer (PCA9548 #0)
            ('pca9548', 0x70, 0),		

            ])

        ########### initialize I2C bus PCA9548 #0 ###########
        self.new_i2c_devices(
            [
            
            # reserved

            # initialize multiplexer (PCA9548 #2)
            ('pca9548', 0x72, 2),

            # initialize multiplexer (PCA9548 #3)
            ('pca9548', 0x73, 3),

            # Port CPLD 0
            ('8730_32d_cpld1' , 0x57, 4),

            # initialize multiplexer (PCA9548 #4)
            ('pca9548', 0x74, 5),

            # initialize multiplexer (PCA9548 #5)
            ('pca9548', 0x75, 6),

            # Port CPLD 1
            ('8730_32d_cpld2' , 0x57, 7),

            # reserved
            
            ])

        ########### initialize I2C bus PCA9548 #2 #3 #4 #5 ###########        

        # Initialize QSFP-DD devices (Port EEPROM)
        for port in range(1, 33):
            self.new_i2c_device('optoe3', 0x50, port+8)

        ########### modprobe i2c-ismt and initialize I2C bus SMB HOST ###########
        os.system("modprobe i2c-ismt")
        self.new_i2c_devices([

            # DPLL 8A34001
            ('8a34001', 0x5B, 41),
            
            # ONIE EEPROM
            ('8730_onie_eeprom', 0x55, 41),            		

            ])


        return True
