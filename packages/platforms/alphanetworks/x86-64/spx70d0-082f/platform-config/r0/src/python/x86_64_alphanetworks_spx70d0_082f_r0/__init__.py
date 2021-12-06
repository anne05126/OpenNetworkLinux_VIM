from onl.platform.base import *
from onl.platform.alphanetworks import *

class OnlPlatform_x86_64_alphanetworks_spx70d0_082f_r0(OnlPlatformAlphaNetworks,
                                              OnlPlatformPortConfig_8x10_2x100):
    PLATFORM='x86-64-alphanetworks-spx70d0-082f-r0'
    MODEL="SPX70D0-082F"
    SYS_OBJECT_ID=".6040.10"

    def baseconfig(self):
    	self.insmod('spx70d0-082f_onie_eeprom')       
        self.insmod('cns653mu')
        ########### initialize I2C bus 0 ###########
        self.new_i2c_devices([

            # TPS53647 @MB
            ('tps53647', 0x65, 0),

            # 8A34002 @MB
            ('8A34002', 0x5B, 0),

            # ONIE EEPROM @MB (FM24C128A)
            ('spx70d0_onie_eeprom', 0x56, 0),
			
            # PCA9539#2 @MB
            ('pca9539', 0x75, 0),
			
            # initialize multiplexer (PCA9548 #0)
            ('pca9548', 0x70, 0),

            ])

        ########### initialize I2C bus PCA9548 #0 ###########
        self.new_i2c_devices(
            [
            # CFG EEPROM @MB (AT24C02D)
            ('at24c02d', 0x51, 1),
                      
            # initialize multiplexer (PCA9548 #2)
            ('pca9548', 0x73, 3),
                      
            # MAC @MB
            ('bcm88470', 0x47, 4),

            # PSU #0
            ('cns653mu', 0x58, 5),
			
            # initialize multiplexer (PCA9548 #1)
            ('pca9548', 0x72, 7),
			
            # PCA9539#0 @MB
            ('pca9539', 0x76, 8),
			
            ])

        ########### initialize I2C bus PCA9548 #2 ###########
        self.new_i2c_devices(
            [
            # QSFP28 1 uplink
            ('optoe1', 0x50, 9),

            # QSFP28 2 uplink
            ('optoe1', 0x50, 10),

            # PCA9539#3 @MB
            ('pca9539', 0x77, 11),
			
            # XFP 7 downlink
            ('optoe2', 0x50, 12),

            # XFP 8 downlink
            ('optoe2', 0x50, 13),
            
            # XFP 9 downlink
            ('optoe2', 0x50, 14),

            # XFP 10 downlink
            ('optoe2', 0x50, 15),

            # PCA9539#4 @MB
            ('pca9539', 0x74, 16),
            
            ])

        ########### initialize I2C bus PCA9548 #1 ###########
        self.new_i2c_devices(
            [
            # XFP 1 downlink
            ('optoe2', 0x50, 17),

            # XFP 2 downlink
            ('optoe2', 0x50, 18),
            
            # XFP 3 downlink
            ('optoe2', 0x50, 19),

            # XFP 4 downlink
            ('optoe2', 0x50, 20),

            # PCA9539#1 @MB
            ('pca9539', 0x76, 21),
		
            # reserved
            
            ])
			
		# initialize sys led (PCA9539#2 @MB)
        subprocess.call('echo 496 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 497 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 498 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 499 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 500 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio496/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio497/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio498/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio499/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio500/direction', shell=True)

        subprocess.call('echo 1 > /sys/class/gpio/gpio496/value', shell=True)
        subprocess.call('echo 1 > /sys/class/gpio/gpio497/value', shell=True)
        subprocess.call('echo 1 > /sys/class/gpio/gpio498/value', shell=True)
        subprocess.call('echo 1 > /sys/class/gpio/gpio499/value', shell=True)
        subprocess.call('echo 1 > /sys/class/gpio/gpio500/value', shell=True)
			
		# PCA9539#3 @MB
        subprocess.call('echo 464 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 465 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 466 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 467 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 468 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 469 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 470 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 471 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 472 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 473 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 474 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 475 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 476 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 477 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 478 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 479 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio464/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio467/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio472/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio475/direction', shell=True)			
		
        subprocess.call('echo 0 > /sys/class/gpio/gpio464/value', shell=True)
        subprocess.call('echo 1 > /sys/class/gpio/gpio467/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio472/value', shell=True)
        subprocess.call('echo 1 > /sys/class/gpio/gpio475/value', shell=True)		
		
		# PCA9539#4 @MB            
        subprocess.call('echo 448 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 449 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 450 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 451 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 452 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 453 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 454 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 455 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 456 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 457 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 458 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 459 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 460 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 461 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 462 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 463 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio448/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio452/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio456/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio460/direction', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio448/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio452/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio456/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio460/value', shell=True)
		
		# PCA9539#1 @MB            
        subprocess.call('echo 432 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 433 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 434 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 435 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 436 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 437 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 438 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 439 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 440 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 441 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 442 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 443 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 444 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 445 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 446 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 447 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio432/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio436/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio440/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio444/direction', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio432/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio436/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio440/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio444/value', shell=True)
		
        return True
