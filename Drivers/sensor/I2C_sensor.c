/**
  ******************************************************************************
  * @file    
  * @author  Dragino
  * @version 
  * @date    
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_delay.h"
#include "log.h"
#include "I2C_sensor.h"
#include "I2C_A.h"
#include "bsp.h"

#define ADS1115_CONFIG 0x8B83  // Single-shot mode, differential AIN0-AIN1, ±0.256V, 128 SPS, comparator disabled

static float displacementFiltered = 0.0f;
static bool firstRun = true;
static float damping = 0.95f;

extern bool bh1750flags;
extern bool iic_noack;
uint8_t mlx_flag = 0;  // Global variable definition
	
uint8_t SHT31_CheckSum_CRC8(uint8_t* Result,uint8_t num) 
{
	uint8_t data[2];
	uint8_t index=0;
	index=num;
	
	data[0] = Result[index];
	data[1] = Result[index+1];

	uint32_t POLYNOMIAL = 0x131;
	uint8_t crc = 0xFF;
	uint8_t bit = 0;
	uint8_t byteCtr = 0;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < 2; ++byteCtr)
	{
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	if (crc == Result[index+2]) {
		return 1;
	}
	else {
		return 0;
	}
}

void SHT31_Read(sht31_t *sht31_data)
{
	uint8_t rxdatas[6];
	uint8_t data[2] = {0x2C, 0x06};	
  bool read_status=1;	
	uint8_t check_number=0;	

	I2C_GPIO_MODE_Config();
	
	do
	{
		read_status=1;
		check_number++;
	  if(I2C_Write_Len(0x44,0x01,2,data)==1)
		{
			read_status=0;
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	if(read_status==1)
	{
		delay_ms(60);
	  check_number=0;
		do
		{
			read_status=1;
			check_number++;
			if(I2C_Read_Len(0x44,0x01,6,rxdatas)==1)
			{
				read_status=0;
				delay_ms(20);
			}
			
			if( SHT31_CheckSum_CRC8(rxdatas,0)==0 && SHT31_CheckSum_CRC8(rxdatas,3)==0)
			{
				read_status=0;
				delay_ms(20);
			}
		}while(read_status==0&&check_number<4);
	}

	if(read_status==1)
	{	
		sht31_data->temp_sht=((rxdatas[0]<<8)+rxdatas[1])*175.0/(65536-1)-45.0;
		sht31_data->hum_sht=((rxdatas[3]<<8)+rxdatas[4])*100.0/(65536-1);
		if(sht31_data->hum_sht>100)	
		{
			sht31_data->hum_sht=100;
		}
		else if(sht31_data->hum_sht<0)
		{		
		  sht31_data->hum_sht=0;
		}	
			
		if(sht31_data->temp_sht>125)
		{
			sht31_data->temp_sht=125;
		}
		else if(sht31_data->temp_sht<-40)
		{
			sht31_data->temp_sht=-40;
		}		
	}
	else
	{
		sht31_data->temp_sht=3276.7;
		sht31_data->hum_sht=6553.5;
	}	
}

uint8_t check_sht31_connect(void)
{
	uint8_t rxdata[3];
	uint8_t data[2] = {0xF3, 0x2D};		
	I2C_GPIO_MODE_Config();
	I2C_Write_Len(0x44,0x01,2,data);
	delay_ms(50);
	I2C_Read_Len(0x44,0x01,3,rxdata);
	if(SHT31_CheckSum_CRC8(rxdata,0)==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t SHT20_CheckSum_CRC8(uint8_t* Result) 
{
	uint8_t data[2];
	data[0] = Result[0];
	data[1] = Result[1];

	uint32_t POLYNOMIAL = 0x131;
	uint8_t crc = 0;
	uint8_t bit = 0;
	uint8_t byteCtr = 0;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < 2; ++byteCtr)
	{
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	if (crc == Result[2]) {
		return 1;
	}
	else {
		return 0;
	}
}

float SHT20_RH(void)
{
	uint8_t txdata[1]={0xf5};//Humidity measurement
  uint8_t rxdata[3];
	uint16_t AD_code;
	float hum;
	uint8_t check_number=0;	
  bool read_status=1;	

	I2C_GPIO_MODE_Config();
	
	do
	{
		read_status=1;
		check_number++;
	  if(I2C_Write_Len(0x40,0x01,1,txdata)==1)
		{
			read_status=0;
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	if(read_status==1)
	{
		delay_ms(30);
	  check_number=0;
		do
		{
			read_status=1;
			check_number++;
			if(I2C_Read_Len(0x40,0x01,3,rxdata)==1)
			{
				read_status=0;
				delay_ms(20);
			}
			
			if(SHT20_CheckSum_CRC8(rxdata)==0)
			{
				read_status=0;
				delay_ms(20);
			}
		}while(read_status==0&&check_number<4);
	}

	if(read_status==1)
	{	
		AD_code=(rxdata[0]<<8)+rxdata[1];
		AD_code &=~0x0003;
		hum=(AD_code*125.0/65536.0)-6.0;
		
		if(hum>100)	
		{
			hum=100;
		}
		else if(hum<0)
		{		
		  hum=0;
		}		
	}
	else
	{
		hum=6553.5;
	}	
  return hum;	
}

float SHT20_RT(void)
{
	uint8_t txdata[1]={0xf3};//Humidity measurement
  uint8_t rxdata[3];
	uint16_t AD_code;
	float tem;
	uint8_t check_number=0;	
  bool read_status=1;	

	I2C_GPIO_MODE_Config();
	
	do
	{
		read_status=1;
		check_number++;
	  if(I2C_Write_Len(0x40,0x01,1,txdata)==1)
		{
			read_status=0;
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	if(read_status==1)
	{
		delay_ms(90);
	  check_number=0;
		do
		{
			read_status=1;
			check_number++;
			if(I2C_Read_Len(0x40,0x01,3,rxdata)==1)
			{
				read_status=0;
				delay_ms(20);
			}
			
			if(SHT20_CheckSum_CRC8(rxdata)==0)
			{
				read_status=0;
				delay_ms(20);
			}
		}while(read_status==0&&check_number<4);
	}
	
	if(read_status==1)
	{	
		AD_code=(rxdata[0]<<8)+rxdata[1];
		AD_code &=~0x0003;
		tem=(AD_code*175.72/65536.0)-46.85;
			
		if(tem>125)
		{
			tem=125;
		}
		else if(tem<-40)
		{
			tem=-40;
		}		
	}
	else
	{
		tem=3276.7;
	}	
  return tem;
}

uint8_t check_sht20_connect(void)
{
	uint8_t rxdata[3];
	uint8_t data[1] = {0xF3};		
	I2C_GPIO_MODE_Config();
	I2C_Write_Len(0x40,0x01,1,data);
	delay_ms(90);
	I2C_Read_Len(0x40,0x01,3,rxdata);
	if(SHT20_CheckSum_CRC8(rxdata)==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint16_t bh1750_read(void)
{
	uint8_t rxdata[2];	
	uint16_t AD_code;
	uint16_t luminance;	
	
	I2C_GPIO_MODE_Config();
	
	delay_ms(10);//Required	
	I2C_Write_Byte(0x23,0x01); 
	I2C_Write_Byte(0x23,0x20);  
  delay_ms(200);	
  I2C_Read_Len(0x23,0x00,2,rxdata);
	I2C_Write_Byte(0x23,0x07);	

	if(iic_noack==1)
	{
		luminance=0;	
	}
	else
	{
		AD_code=(rxdata[0]<<8)+rxdata[1];
		luminance=AD_code/1.2;
  }
	
	iic_noack=0;
	
	return luminance;
}

void I2C_read_data(sensor_t *sensor_data,uint8_t flag_temp, uint8_t message)
{	
	I2C_GPIO_MODE_Config();
	if(flag_temp==0)
	{
		sensor_data->temp_sht=6553.5;
		sensor_data->hum_sht=6553.5;
	} 
	else if(flag_temp==1)
	{
		sensor_data->temp_sht=SHT20_RT();
		sensor_data->hum_sht=SHT20_RH(); 
		if(message==1)
		{
			LOG_PRINTF(LL_DEBUG,"SHT2x_temp:%.1f,SHT2x_hum:%.1f\r\n",sensor_data->temp_sht,sensor_data->hum_sht);
			delay_ms(20);
		}
	}
	else if(flag_temp==2)
	{			       	
		sht31_t temphum_data;		
		SHT31_Read(&temphum_data);				
		sensor_data->temp_sht=temphum_data.temp_sht;
		sensor_data->hum_sht=temphum_data.hum_sht;	
		if(message==1)
		{
			LOG_PRINTF(LL_DEBUG,"SHT3x_temp:%.1f,SHT3x_hum:%.1f\r\n",sensor_data->temp_sht,sensor_data->hum_sht);
			delay_ms(20);
		}
	}
	else if(flag_temp==3)
	{		
    bh1750flags=1;		
		sensor_data->illuminance=bh1750_read();		
		if(message==1)
		{			
			LOG_PRINTF(LL_DEBUG,"BH1750_lum:%d\r\n",sensor_data->illuminance);	
      delay_ms(20);			
		}				
	}		
  I2C_GPIO_MODE_ANALOG();	
}

void LidarLite_init(void)
{
   uint8_t sigCountMax[1]={0x80};
   uint8_t acqConfigReg[1]={0x08};
   uint8_t refCountMax[1]={0x05};
   uint8_t thresholdBypass[1]={0x00};	
   waitbusy(1); 
	 I2C_Write_reg_Len(0x62,0x02,1,sigCountMax);
   waitbusy(1); 
	 I2C_Write_reg_Len(0x62,0x04,1,acqConfigReg);
   waitbusy(1);
   I2C_Write_reg_Len(0x62,0x12,1,refCountMax);	 
   waitbusy(1); 
	 I2C_Write_reg_Len(0x62,0x1c,1,thresholdBypass);	 
   waitbusy(1);	
}

uint16_t LidarLite(void)
{
	uint8_t dataByte[1]={0x04};
  uint8_t rxdata1[1];
	uint8_t rxdata2[1];
	uint16_t distance;
  waitbusy(1);
  I2C_Write_reg_Len(0x62,0x00,1,dataByte);
  if(waitbusy(2)<999)
	{		
		I2C_Read_reg_Len(0x62,0x0f,1,rxdata1);
		I2C_Read_reg_Len(0x62,0x10,1,rxdata2);
		distance=(rxdata1[0]<<8)+rxdata2[0];
		if(distance>4000)
		{
			distance=0;			
			return distance;			
		}
		else
		{
			return distance*10;	
		}			
	}
	else
	{
	 distance=65535;
	 return distance;			
	}	
}

uint16_t waitbusy(uint8_t mode)
{
  uint16_t busyCounter = 0;
	uint8_t busy[1]={0x01};
  while (busy[0])      
  {
   if((mode==1)&&(busyCounter > 99))
   {
		return busyCounter;			 
   }
	 
   if((mode==2)&&(busyCounter > 999))
   {
		return busyCounter;			 
   }	 
	 I2C_Read_reg_Len(0x62,0x01,1,busy);
	 busy[0] &=0x01;
	 busyCounter++;
  }
	return busyCounter;	
}
float MLX90614_ReadTemp(uint8_t devAddr, uint8_t regAddr)
{
    uint8_t rxdata[3];
    uint16_t temp;
    float temperature;
    bool read_status = 1;
    uint8_t check_number = 0;

    do {
        read_status = 1;
        check_number++;
        if(I2C_Read_reg_Len(devAddr, regAddr, 3, rxdata) == 1) {
            read_status = 0;
            delay_ms(20);
        }
    } while(read_status == 0 && check_number < 4);

    if(read_status == 1) {
        temp = (rxdata[1] << 8) | rxdata[0];
        temperature = (temp * 0.02) - 273.15;  // Convert to Celsius
        
        // Validate temperature range
        if(temperature > 125) {
            temperature = 125;
        } else if(temperature < -40) {
            temperature = -40;
        }
    } else {
        temperature = 3276.7;  // Error value
    }
    
    return temperature;
}

uint8_t check_mlx90614_connect(uint8_t devAddr)
{
    uint8_t rxdata[3];
    
    I2C_GPIO_MODE_Config();
    if(I2C_Read_reg_Len(devAddr, MLX90614_TA_REG, 3, rxdata) == 0) {
        return 1;  // Device responded
    }
    return 0;  // No response
}

void MLX90614_read_data(sensor_t *sensor_data, uint8_t message)
{
    // Read from two MLX90614 sensors with different addresses
    I2C_GPIO_MODE_Config();
    
    // Read from first MLX90614 (default address 0x5A)
    sensor_data->temp2 = MLX90614_ReadTemp(0x5A, MLX90614_TOBJ1_REG);
    
    // Read from second MLX90614 (assumed address 0x5B)
    sensor_data->temp3 = MLX90614_ReadTemp(0x5B, MLX90614_TOBJ1_REG);
    
    if(message == 1) {
        LOG_PRINTF(LL_DEBUG, "MLX90614_1 temp:%.1f, MLX90614_2 temp:%.1f\r\n", 
                  sensor_data->temp2, sensor_data->temp3);
        delay_ms(20);
    }
    
    I2C_GPIO_MODE_ANALOG();
}
// Check if ADS1115 is connected by reading from the configuration register.
uint8_t check_ads1115_connect(void) {
    uint8_t rxdata[2];
    uint8_t check_number = 0;
    bool read_status = false; // Start with failure

    I2C_GPIO_MODE_Config();
    
    do {
        check_number++;
        // Read config register; assume success returns 0.
        if (I2C_Read_reg_Len(ADS1115_ADDR, ADS1115_REG_POINTER_CONFIG, 2, rxdata) == 0) {
            read_status = true; // Connection works
            break;
        }
        delay_ms(20);
    } while (check_number < 4);

    return (uint8_t)read_status;
}
//---------------------------------------------------------------------------
// Read a differential ADC sample from the ADS1115 and average multiple samples.
int16_t ads1115_read_differential(void) {
    uint8_t txdata[3];
    uint8_t rxdata[2];
    const int NUM_SAMPLES = 20;  // Increase averaging for noise reduction
    int32_t sum = 0;
    int successful_samples = 0;
    
    // Configure ADS1115 by writing the configuration register.
    // txdata[0] is the register pointer (config register).
    // txdata[1] and txdata[2] are the configuration high and low bytes.
    txdata[0] = ADS1115_REG_POINTER_CONFIG;
    txdata[1] = (uint8_t)(ADS1115_CONFIG >> 8);
    txdata[2] = (uint8_t)(ADS1115_CONFIG & 0xFF);

    for (int i = 0; i < NUM_SAMPLES; i++) {
        // Trigger conversion by writing configuration.
        if (I2C_Write_Len(ADS1115_ADDR, txdata[0], 3, txdata) != 0) {
            // Write failed, skip this sample.
            continue;
        }
        
        // Wait for conversion to complete by polling the OS bit.
        uint8_t timeout = 20; // timeout in milliseconds
        uint8_t rx_config[2];
        bool conversion_ready = false;
        while (timeout--) {
            delay_ms(1);
            if (I2C_Read_reg_Len(ADS1115_ADDR, ADS1115_REG_POINTER_CONFIG, 2, rx_config) != 0) {
                // I2C read error: abandon this sample.
                break;
            }
            // OS bit is the MSB (bit 15, found in rx_config[0] bit 7).
            if (rx_config[0] & 0x80) {
                conversion_ready = true;
                break;
            }
        }
        if (!conversion_ready) {
            LOG_PRINTF(LL_DEBUG, "Conversion timeout sample %d\r\n", i);
            continue;
        }
        
        // Read the conversion result.
        if (I2C_Read_reg_Len(ADS1115_ADDR, ADS1115_REG_POINTER_CONVERT, 2, rxdata) == 0) {
            int16_t reading = (int16_t)((rxdata[0] << 8) | rxdata[1]);
            
            // Optional: Discard outlier readings if necessary.
            if (reading > 10000 || reading < -10000) {
                LOG_PRINTF(LL_DEBUG, "Outlier: 0x%04X\r\n", reading);
                continue;
            }
            
            sum += reading;
            successful_samples++;
        }
        delay_ms(5);  // Short delay between samples
    }
    
    if (successful_samples == 0) {
        LOG_PRINTF(LL_ERR, "All samples failed\r\n");
        return 32767;  // Indicate error (choose an error code that does not conflict with valid readings)
    }
    return (int16_t)(sum / successful_samples);
}

//---------------------------------------------------------------------------
// Convert the raw ADC reading to displacement (in mm).
//
// Important notes:
// - The ADS1115 conversion factor (mv_per_bit) depends on your PGA setting.
//   Here we assume that ADS1115_CONFIG is set to a full–scale range of ±0.128 V,
//   yielding an LSB of 3.90625 µV per count (i.e. 0.00390625 mV per bit).
//
// - Our dendrometer sensor produces a voltage that decreases with increasing growth.
//   Let V_baseline be the sensor output (in mV) when the sensor is unstrained.
//   Then for a measured voltage V_measured, the displacement is computed as:
//       displacement_mm = (V_baseline - V_measured) / 0.65
//   where 0.65 mV/mm is the (absolute) sensor slope.
//   Calibrate V_baseline using a known unstrained measurement.
float ads1115_convert_to_mm(int16_t raw_adc) {
    // For a ±0.256 V full–scale setting of the ADS1115:
    const float mv_per_bit = 0.0078125f;  // in mV per bit

    // Convert the raw ADC reading to a voltage in mV
    float measured_voltage_mV = raw_adc * mv_per_bit;

    // Define the baseline voltage when the sensor is unstrained.
    // According to the sensor documentation, this should be ~50 mV.
    const float baseline_voltage_mV = 50.0f;
    
    // Sensor’s sensitivity: -0.65 mV per mm (negative means the voltage drops as growth increases)
    // We use the absolute value here because we want a positive displacement.
    const float sensor_slope = 0.65f;  // mV per mm

    // Compute the displacement. When the sensor is unstrained:
    // measured_voltage_mV == baseline_voltage_mV, so displacement = 0.
    float displacement_mm = (baseline_voltage_mV - measured_voltage_mV) / sensor_slope;
    
    LOG_PRINTF(LL_DEBUG,
               "Raw ADC: %d, Measured Voltage: %.3f mV, Displacement: %.3f mm\r\n",
               raw_adc, measured_voltage_mV, displacement_mm);
    
    return displacement_mm;
}


uint8_t check_ads122c04_connect(void) {
    uint8_t check_number = 0;
    uint8_t cmd = 0x06;

    while (check_number++ < 4) {
        LOG_PRINTF(LL_DEBUG, "Attempt %d: Writing 0x%02X to addr 0x%02X\r\n", check_number, cmd, ADS122C04_ADDR);
        if (I2C_Write_Len(ADS122C04_ADDR, 0x00, 1, &cmd) == 0) {
            LOG_PRINTF(LL_DEBUG, "ADS122C04 detected on attempt %d\r\n", check_number);
            return 1;
        } else {
            LOG_PRINTF(LL_DEBUG, "I2C write failed on attempt %d\r\n", check_number);
        }
        delay_ms(20);
    }

    return 0;
}


void ads122c04_write_register(uint8_t reg, uint8_t value) {
    uint8_t cmd = 0x40 | (reg << 2);  // WREG command

    I2C_Start();
    I2C_SendByte((ADS122C04_ADDR << 1) | 0);  // Write mode
    if (I2C_WaitAck()) { I2C_Stop(); return; }

    I2C_SendByte(cmd);
    if (I2C_WaitAck()) { I2C_Stop(); return; }

    I2C_SendByte(value);
    if (I2C_WaitAck()) { I2C_Stop(); return; }

    I2C_Stop();
}



uint8_t ads122c04_read_register(uint8_t reg) {
    uint8_t cmd = 0x20 | (reg << 2);  // RREG command
    uint8_t value = 0;

    I2C_Start();
    I2C_SendByte((ADS122C04_ADDR << 1) | 0);  // Write
    if (I2C_WaitAck()) { I2C_Stop(); return 0; }

    I2C_SendByte(cmd);
    if (I2C_WaitAck()) { I2C_Stop(); return 0; }

    I2C_Start();
    I2C_SendByte((ADS122C04_ADDR << 1) | 1);  // Read
    if (I2C_WaitAck()) { I2C_Stop(); return 0; }

    value = I2C_ReadByte(0);  // Read one byte, NACK
    I2C_Stop();

    return value;
}


void ads122c04_configure(void) {
    uint8_t reset_cmd = 0x06;
    I2C_Write_Len(ADS122C04_ADDR, 0x00, 1, &reset_cmd);
    delay_ms(10);  // wait after reset

    ads122c04_write_register(0x00, 0x0C);  // CONFIG0: Gain 32
    ads122c04_write_register(0x01, 0x04);  // CONFIG1: 20 SPS, normal mode
    ads122c04_write_register(0x02, 0x00);  // CONFIG2: default
    ads122c04_write_register(0x03, 0x00);  // CONFIG3: default
}




int32_t ads122c04_read_raw_exact(void)
{
    uint8_t cmd;
    uint8_t data[3] = {0};
    int32_t adc = 0;
    
    // First, verify device is responsive
    uint8_t reg0 = ads122c04_read_register(0);
    if (reg0 != 0x0C) {
        LOG_PRINTF(LL_DEBUG, "ADS122C04: Device not properly configured. Reg0=0x%02X\r\n", reg0);
        ads122c04_configure();  // Try to reconfigure
        delay_ms(10);
    }

    // Step 1: Send START/SYNC command
    cmd = 0x08;
    if (I2C_Write_Len(ADS122C04_ADDR, 0x00, 1, &cmd) != 0) {
        LOG_PRINTF(LL_DEBUG, "ADS122C04: START/SYNC failed\r\n");
        return 0x7FFFFF;  // Error value
    }

    delay_ms(100);  // Wait for conversion to complete (increased from 60ms)

    // Step 2: Send RDATA command
    cmd = 0x10;
    if (I2C_Write_Len(ADS122C04_ADDR, 0x00, 1, &cmd) != 0) {
        LOG_PRINTF(LL_DEBUG, "ADS122C04: RDATA cmd failed\r\n");
        return 0x7FFFFF;  // Error value
    }

    // Step 3: Direct read after RDATA (no register address needed)
    I2C_Start();
    I2C_SendByte((ADS122C04_ADDR << 1) | 1);  // Read mode
    if (I2C_WaitAck()) {
        LOG_PRINTF(LL_DEBUG, "ADS122C04: No ACK on read\r\n");
        I2C_Stop();
        return 0x7FFFFF;  // Error value
    }
    
    data[0] = I2C_ReadByte(1);  // Read byte 1 with ACK
    data[1] = I2C_ReadByte(1);  // Read byte 2 with ACK
    data[2] = I2C_ReadByte(0);  // Read byte 3 with NACK
    I2C_Stop();

    LOG_PRINTF(LL_DEBUG, "ADS122C04 raw bytes: 0x%02X 0x%02X 0x%02X\r\n", 
               data[0], data[1], data[2]);

    adc = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
    if (adc & 0x800000) {
        adc |= 0xFF000000;  // Sign extend negative 24-bit value
    }

    return adc;
}


int32_t ads122c04_read_filtered(uint8_t samples) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += ads122c04_read_raw();
    }
    return sum / samples;
}

void ads122c04_read_displacement(int32_t* raw_out, float* voltage_mv_out, float* filtered_disp_out)
{
    int32_t raw = 0;
		for (int i = 0; i < 8; i++) {
			raw += ads122c04_read_raw_exact();
		}
		raw /= 8;
    float voltage_mv = (float)raw * 2.048f / (8388608.0f * 32) * 1000.0f;
    float displacement = -1.538f * voltage_mv;

    if (firstRun) {
        displacementFiltered = displacement;
        firstRun = false;
    } else {
        displacementFiltered = damping * displacementFiltered + (1.0f - damping) * displacement;
    }

    if (raw_out) *raw_out = raw;
    if (voltage_mv_out) *voltage_mv_out = voltage_mv;
    if (filtered_disp_out) *filtered_disp_out = displacementFiltered;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
