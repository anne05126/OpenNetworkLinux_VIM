from onl.platform.base import *
from onl.platform.alphanetworks import *

class OnlPlatform_x86_64_alphanetworks_spx70d0_168f_r0(OnlPlatformAlphaNetworks,
                                              OnlPlatformPortConfig_16x10_6x25_2x100):
    PLATFORM='x86-64-alphanetworks-spx70d0-168f-r0'
    MODEL="SPX70D0-168F"
    SYS_OBJECT_ID=".6040.24"

    def baseconfig(self):
        self.insmod('spx70d0-168f_onie_eeprom')
    	self.insmod('spx70d0-168f_psu')
    	self.insmod('spx70d0-168f_pwr_cpld')
        self.insmod('spx70d0-168f_fan')
    	self.insmod('spx70d0-168f_led')
        self.insmod('spx70d0-168f_thermal')
    	
        ########### initialize I2C bus 0 ###########
        self.new_i2c_devices([
         
            # initialize multiplexer (PCA9548 #0)
            ('pca9548', 0x70, 0),

            ])

        ########### initialize I2C bus PCA9548 #0 ###########
        self.new_i2c_devices(
            [
			
            # PON Board ID EEPROM @PON Board (AT24C02D)
            ('at24c02d', 0x57, 1),
            
            # PCA9539#2 GPON/XGSPON SELECT @PON Board
            ('pca9539', 0x74, 1),

            # PCA9539#3 PON Ports effuse interrupt @PON Board
            ('pca9539', 0x75, 1),

            # 8T49N240's EEPROM (M24C02-WMN6TP)
            ('m24c02', 0x55, 1),

            # CLK GEN for 155.52MHz (8T49N240)
            ('8t49n240', 0x6C, 1),

            # CFG EEPROM @MB (AT24C02D)
            ('at24c02d', 0x51, 1),

            # TIMING SYNC FPGA I2C0 (LFD2NX-40-BG256)
            ('ldf2nx', 0x6E, 1),
                      
            # initialize multiplexer (PCA9548 #2)
            ('pca9548', 0x72, 2),

            # PCA9539#4 @MB
            ('pca9539', 0x74, 2),

            # PCA9539#5 @MB
            ('pca9539', 0x75, 2),

            # initialize multiplexer (PCA9548 #3)
            ('pca9548', 0x73, 2),

            # PCA9539#6 @MB
            ('pca9539', 0x76, 2),

            # PCA9539#7 @MB
            ('pca9539', 0x77, 2),

            # initialize multiplexer (PCA9548 #1)
            ('pca9548', 0x72, 3),

            # PCA9539#0 @MB
            ('pca9539', 0x74, 3),

            # PCA9539#1 @MB
            ('pca9539', 0x75, 3),
                      
            # MAC @MB
            ('bcm88470', 0x47, 4),

            # FAN EXT-Board EEPROM @FAN-EXTBD
            ('fan_extbd', 0x51, 5),

            # Jitter Attenuator (8V9N145)
            ('8v9n145', 0x6D, 8),

            # EEPROM for Jitter Attenuator (M24C02-WMN6TP)
            ('m24c02', 0x50, 8),
			
            ])

        ########### initialize I2C bus PCA9548 #2 ###########
        self.new_i2c_devices(
            [
            # SFP+ 9 downlink
            ('optoe2', 0x50, 9),

            # SFP+ 10 downlink
            ('optoe2', 0x50, 10),

            # SFP+ 11 downlink
            ('optoe2', 0x50, 11),

            # SFP+ 12 downlink
            ('optoe2', 0x50, 12),

            # SFP+ 13 downlink
            ('optoe2', 0x50, 13),

            # SFP+ 14 downlink
            ('optoe2', 0x50, 14),

            # SFP+ 15 downlink
            ('optoe2', 0x50, 15),

            # SFP+ 16 downlink
            ('optoe2', 0x50, 16),
            
            ])

        ########### initialize I2C bus PCA9548 #3 ###########
        self.new_i2c_devices(
            [
            # SFP+ 17 downlink
            ('optoe2', 0x50, 17),
            
            # SFP+ 18 downlink
            ('optoe2', 0x50, 18),

            # SFP+ 19 downlink
            ('optoe2', 0x50, 19),

            # SFP+ 20 downlink
            ('optoe2', 0x50, 20),

            # SFP+ 21 downlink
            ('optoe2', 0x50, 21),

            # SFP+ 22 downlink
            ('optoe2', 0x50, 22),

            # SFP+ 23 downlink
            ('optoe2', 0x50, 23),

            # SFP+ 24 downlink
            ('optoe2', 0x50, 24),
            
            ])

        ########### initialize I2C bus PCA9548 #1 ###########
        self.new_i2c_devices(
            [
            # QSFP28 1 downlink
            ('optoe1', 0x50, 25),

            # QSFP28 2 uplink
            ('optoe1', 0x50, 26),

            # SFP28 3 downlink
            ('optoe2', 0x50, 27),

            # SFP28 4 downlink
            ('optoe2', 0x50, 28),

            # SFP28 5 downlink
            ('optoe2', 0x50, 29),

            # SFP28 6 downlink
            ('optoe2', 0x50, 30),

            # SFP28 7 downlink
            ('optoe2', 0x50, 31),

            # SFP28 8 downlink
            ('optoe2', 0x50, 32),
            
            ])

        ########### modprobe i2c-ismt and initialize I2C bus SMB HOST ###########
        os.system("modprobe i2c-ismt")
        self.new_i2c_devices([

            # ONIE EEPROM @MB (FM24C128A)
            ('spx70d0_onie_eeprom', 0x55, 33),

            # 8A34002 @MB
            ('8a34002', 0x5B, 33),

            ])

		# PCA9539#2 GPON/XGSPON SELECT @PON Board
        subprocess.call('echo 342 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 343 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 344 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 345 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 346 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 347 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 348 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 349 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 350 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 351 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 352 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 353 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 354 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 355 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 356 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 357 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio342/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio343/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio344/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio345/direction', shell=True)
        
        subprocess.call('echo out > /sys/class/gpio/gpio346/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio347/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio348/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio349/direction', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio350/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio351/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio352/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio353/direction', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio354/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio355/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio356/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio357/direction', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio342/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio343/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio344/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio345/value', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio346/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio347/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio348/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio349/value', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio350/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio351/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio352/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio353/value', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio354/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio355/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio356/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio357/value', shell=True)

		# PCA9539#4 @MB
        subprocess.call('echo 310 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 311 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 312 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 313 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 314 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 315 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 316 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 317 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 318 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 319 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 320 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 321 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 322 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 323 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 324 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 325 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio310/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio314/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio318/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio322/direction', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio310/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio314/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio318/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio322/value', shell=True)

        # PCA9539#5 @MB
        subprocess.call('echo 294 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 295 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 296 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 297 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 298 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 299 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 300 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 301 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 302 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 303 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 304 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 305 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 306 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 307 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 308 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 309 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio294/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio298/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio302/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio306/direction', shell=True)	

        subprocess.call('echo 0 > /sys/class/gpio/gpio294/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio298/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio302/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio306/value', shell=True)
			
		# PCA9539#6 @MB
        subprocess.call('echo 278 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 279 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 280 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 281 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 282 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 283 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 284 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 285 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 286 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 287 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 288 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 289 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 290 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 291 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 292 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 293 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio278/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio282/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio286/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio290/direction', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio278/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio282/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio286/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio290/value', shell=True)

		# PCA9539#7 @MB            
        subprocess.call('echo 262 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 263 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 264 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 265 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 266 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 267 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 268 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 269 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 270 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 271 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 272 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 273 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 274 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 275 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 276 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 277 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio262/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio266/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio270/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio274/direction', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio262/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio266/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio270/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio274/value', shell=True)

		# PCA9539#0 @MB            
        subprocess.call('echo 246 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 247 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 248 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 249 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 250 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 251 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 252 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 253 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 254 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 255 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 256 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 257 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 258 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 259 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 260 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 261 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio246/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio249/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio250/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio253/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio254/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio258/direction', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio246/value', shell=True)
        subprocess.call('echo 1 > /sys/class/gpio/gpio249/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio250/value', shell=True)
        subprocess.call('echo 1 > /sys/class/gpio/gpio253/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio254/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio258/value', shell=True)

        # PCA9539#1 @MB            
        subprocess.call('echo 230 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 231 > /sys/class/gpio/export', shell=True) 
        subprocess.call('echo 232 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 233 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 234 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 235 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 236 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 237 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 238 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 239 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 240 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 241 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo 242 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 243 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 244 > /sys/class/gpio/export', shell=True)
        subprocess.call('echo 245 > /sys/class/gpio/export', shell=True)

        subprocess.call('echo out > /sys/class/gpio/gpio230/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio234/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio238/direction', shell=True)
        subprocess.call('echo out > /sys/class/gpio/gpio242/direction', shell=True)

        subprocess.call('echo 0 > /sys/class/gpio/gpio230/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio234/value', shell=True)
        subprocess.call('echo 0 > /sys/class/gpio/gpio238/value', shell=True)		
        subprocess.call('echo 0 > /sys/class/gpio/gpio242/value', shell=True)

        return True
