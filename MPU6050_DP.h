#ifndef MPU6050_DP_H
#define MPU6050_DP_H

/**
*MPU6050 Registers. 
**/
#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
#define YGOFFS_TC        0x01                                                                          
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E    
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68
//////////////////////////////////////////////////////

/**
* Using the GY-521 breakout board, I set ADO to 0 by grounding through a 4k7 resistor
* Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
**/
#define ADO 0
#if ADO
#define MPU6050_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68  // Device address when ADO = 0
#endif

// Configuration bits
#define BIT_SLEEP               0x40
#define BIT_H_RESET             0x80
#define BITS_CLKSEL             0x07
#define MPU_CLK_SEL_PLLGYROX    0x01
#define MPU_CLK_SEL_PLLGYROZ    0x03
#define MPU_EXT_SYNC_GYROX      0x02
#define BITS_FS_250DPS          0x00
#define BITS_FS_500DPS          0x08
#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
#define BITS_FS_MASK            0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR    0x10
#define BIT_RAW_RDY_EN          0x01
#define BIT_I2C_IF_DIS          0x10
#define BIT_INT_STATUS_DATA     0x01


class MPU6050 {

public:	
	// Set initial input parameters
	enum AScale {
	  AFS_2G = 0,
	  AFS_4G,
	  AFS_8G,
	  AFS_16G
	};

	enum GScale {
	  GFS_250DPS = 0,
	  GFS_500DPS,
	  GFS_1000DPS,
	  GFS_2000DPS
	};

	/*
	* Specify sensor full scale
	*/
	int Gscale;
	int Ascale;

	MPU6050() {
		Gscale = GFS_1000DPS;
		Ascale = AFS_4G;
		
		gyroScaleFactor = 0.0f;
		accelScaleFactor = 0.0f;
		
		getAres();
		getGres();
	}
	
	/**
	* Initialize the MPU6050.
	**/
	void initMPU6050()
	{  
	// wake up device-don't need this here if using calibration function below
	  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, BIT_H_RESET); // Clear sleep mode bit (6), enable all sensors 
	  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

	 // get stable time source
	 // Set clock source to be PLL with z-axis gyroscope reference, bits 2:0 = 0011
	  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x03);
	  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);

	 // Configure Gyro and Accelerometer
	 // DLPF_CFG = bits 2:0 = 000; this sets the sample rate at 1 kHz for both
	 //DLPF = 42 hz
	  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);  
	 
	 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);  // Use a 1kHz rate; the same rate set in CONFIG above
	 
	 // Set gyroscope full scale range
	 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	  uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
	  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
	   
	 // Set accelerometer configuration
	  c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
	  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

	  delay(500);
	  
	  // Configure Interrupts and Bypass Enable
	  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
	  // can join the I2C bus and all can be controlled by the Arduino as master
	   writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);    
	   writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	}
	
	void calibrateGyro() {
		static uint8_t retry = 0;
		uint8_t i, count = 128;
		int16_t xSum = 0, ySum = 0, zSum = 0;

		for (i = 0; i < count; i++) {
			readGyroRaw(gyroRaw);
			xSum += gyroRaw[XAXIS];
			ySum += gyroRaw[YAXIS];
			zSum += gyroRaw[ZAXIS];
			delay(10);
		}
		
		gyro_offset[0] = xSum / count;
		gyro_offset[1] = ySum / count;
		gyro_offset[2] = zSum / count; 
		
		// Calibration sanity check
		// if suitable offset couldn't be established, break out of the loop after 10 retries
		if ((abs(gyro_offset[XAXIS]) > 300 || abs(gyro_offset[YAXIS]) > 300 || abs(gyro_offset[ZAXIS]) > 300) && retry < 10) {
			// gyro calibration failed, run again
			retry++;
			
			// small delay before next gyro calibration
			delay(500);
			
			calibrate_gyro();
		}
	}
	
	void calibrateAccelerometer() {
		uint8_t i, count = 128;
		int32_t xSum = 0, ySum = 0, zSum = 0;

		for (i = 0; i < count; i++) {
			readAccelRaw(accelRaw);
			xSum += accelRaw[0];
			ySum += accelRaw[1];
			zSum += accelRaw[2];
			delay(10);
		}
		
		accel_offset[0] = xSum / count;
		accel_offset[1] = ySum / count;
		accel_offset[2] = zSum / count;
	}
	
	void readGyroSum() {
		readGyroRaw(gyroRaw);
		
		gyroSum[0] += gyroRaw[0];
		gyroSum[1] += gyroRaw[1];
		gyroSum[2] += gyroRaw[2]:
		
		gyroSamples++;
	}
	
	void readAccelSum() {
		readAccelRaw(accelRaw);
		
		accelSum[0] += accelRaw[0];
		accelSum[1] += accelRaw[1];
		accelSum[2] += accelRaw[2];
		
		accelSamples++;
	}
	
	void evaluateGyro(float * data) {
		data[0] = gyroSum[0]/gyroSamples;
		data[1] = gyroSum[1]/gyroSamples;
		data[2] = gyroSum[2]/gyroSamples;
		
		data[0] += gyro_offset[0];
		data[1] += gyro_offset[1];
		data[2] += gyro_offset[2];
	
		data[0] *= gyroScaleFactor;
		data[1] *= gyroScaleFactor;
		data[2] *= gyroScaleFactor;
		
		gyroSum[0] = 0;
		gyroSum[1] = 0;
		gyroSum[2] = 0;
		
		gyroSamples = 0;
	}
	
	void evaluateAccel(float * data) {
		data[0] = accelSum[0]/accelSamples;
		data[1] = accelSum[1]/accelSamples;
		data[2] = accelSum[2]/accelSamples;
		
		data[0] += accel_offset[0];
		data[1] += accel_offset[1];
		data[2] += accel_offset[2];
		
		data[0] *= accelScaleFactor;
		data[1] *= accelScaleFactor;
		data[2] *= accelScaleFactor;
		
		accelSum[0] = 0;
		accelSum[1] = 0;
		accelSum[2] = 0;
		
		accelSamples = 0;
	}

	/**
	* Read the raw accelerometer data.
	* @param *destination array of 16 bit integers that the three axis values are written to.
	*/
	void readAccelRaw(int16_t * destination) {
	  uint8_t rawData[6];  // x/y/z accel register data stored here
	  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;  
	  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ; 
	}

	/**
	* Read the raw gyroscope data. 
	* @param *destination array of 16 bit integers that the 3 axis values are written to.
	*/
	void readGyroRaw(int16_t * destination) {
	  uint8_t rawData[6];  // x/y/z gyro register data stored here
	  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;  
	  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ; 
	}

	/**
	* Reads the temperature data from the sensor.
	* @return int16_t 16 bit integer representing the sensor temperature.
	*/
	int16_t readTempData() {
	  uint8_t rawData[2];  //temp register data stored here
	  readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
	  return ((int16_t)rawData[0]) << 8 | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
	}
	
private:
	
	/**
	* Sets the gyroscope scale.
	*/
	void getGres() {
	  switch (Gscale)
	  {
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
		// Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
		case GFS_250DPS:
			  gyroScaleFactor = 250.0/32768.0;
			  break;
		case GFS_500DPS:
			  gyroScaleFactor = 500.0/32768.0;
			  break;
		case GFS_1000DPS:
			  gyroScaleFactor = 1000.0/32768.0;
			  break;
		case GFS_2000DPS:
			  gyroScaleFactor = 2000.0/32768.0;
			  break;
	  }
	}

	/**
	* Sets the accelerometer scale.
	*/
	void getAres() {
	  switch (Ascale)
	  {
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
			// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
			  accelScaleFactor = 2.0/32768.0;
			  break;
		case AFS_4G:
			  accelScaleFactor = 4.0/32768.0;
			  break;
		case AFS_8G:
			  accelScaleFactor = 8.0/32768.0;
			  break;
		case AFS_16G:
			  accelScaleFactor = 16.0/32768.0;
			  break;
	  }
	}
	
	/**
	* Helper function for writing bytes to the wire. 
	* @param address the address to write to. 
	* @param subaddress the sub address to write to in the address.
	* @param data the data to write.
	*/
	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
		Wire.beginTransmission(address);  // Initialize the Tx buffer
		Wire.write(subAddress);           // Put slave register address in Tx buffer
		Wire.write(data);                 // Put data in Tx buffer
		Wire.endTransmission();           // Send the Tx buffer
	}

	/**
	* Helper function to read a byte from the wire. 
	* @param address the address to read from.
	* @param subAddress to read from the address.
	* @return uint_8 the byte that was read.
	*/
	uint8_t readByte(uint8_t address, uint8_t subAddress) {
		uint8_t data; // `data` will store the register data	 
		Wire.beginTransmission(address);         // Initialize the Tx buffer
		Wire.write(subAddress);	                 // Put slave register address in Tx buffer
		Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
		Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
		data = Wire.read();                      // Fill Rx buffer with result
		return data;                             // Return data read from slave register
	}

	/**
	* Helper function to read multiple bytes from the Wire.
	* @param address the address to read from.
	* @param subAddress the subAddress to read from.
	* @param count the number of bytes to read.
	* @param *dest the destination of the read bytes.
	*/
	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {  
		// Initialize the Tx buffer
		Wire.beginTransmission(address); 
		// Put slave register address in Tx buffer	
		Wire.write(subAddress); 
		// Send the Tx buffer, but send a restart to keep connection alive
		Wire.endTransmission(false);       
		uint8_t i = 0;
		// Read bytes from slave register address 
		Wire.requestFrom(address, count);	
		while (Wire.available()) {
			dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
	}
	
	int16_t gyro_offset[3];
	int16_t accel_offset[3];
	
	float gyroScaleFactor;
	float accelScaleFactor; 

	int16_t gyroRaw[3];
	float gyroSum[3];

	int16_t accelRaw[3];
	float accelSum[3];
	
	uint8_t gyroSamples;
	uint8_t accelSamples;  

};
#endif