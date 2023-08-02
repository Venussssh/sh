#include "LSM6DS3.h"
#include "stdio.h"
#include "spi.h"
#include "stdbool.h"


// template
// void SetRegister(uint8_t register, uint8_t value)       
// {

// }

// #define Bit(x)  
// (0x01  << 4) 


//加速度速率
void Set_Acc_Rate(uint8_t rate)
{
	SPI2_ReadCommand(CTRL1_XL);                 // x 读 改 写
    uint8_t send_text[1] = {0};
	send_text[0] |= rate;
	SPI2_WriteCommand(CTRL1_XL, send_text);
}



//加速度量程
void Set_Accelerometer_Fullscale(uint8_t value)
{
	
	SPI2_ReadCommand(CTRL1_XL);
    uint8_t send_text[1] = {0};
	send_text[0] |= value;
	SPI2_WriteCommand(CTRL1_XL, send_text);
}
//角速度速率
void Set_Gyroscope_Rate(uint8_t rate)
{
	
	SPI2_ReadCommand(CTRL2_G);
    uint8_t send_text[1] = {0};
	send_text[0] |= rate;
	SPI2_WriteCommand(CTRL2_G, send_text);
}


//角速度量程
void Set_Gyroscope_Fullscale(uint8_t value)
{
	SPI2_ReadCommand(CTRL2_G);
    uint8_t send_text[1] = {0};
	send_text[0] |= value;
	SPI2_WriteCommand(CTRL2_G, send_text);
}
//带宽     角速度计
void Set_Accelerometer_Bandwidth(uint8_t BW0XL, uint8_t ODR)
{
	uint8_t send_text1[1] = {0};
    uint8_t send_text2[1] = {0};
	SPI2_ReadCommand(CTRL1_XL);
	send_text1[0] |= BW0XL;
	SPI2_WriteCommand(CTRL1_XL, send_text1);

	SPI2_ReadCommand(CTRL8_XL);
	send_text2[0] |= ODR;
	SPI2_WriteCommand(CTRL8_XL, send_text2);
}

//带宽     陀螺仪
void Set_Gyroscope_Bandwidth(uint8_t reg7, uint8_t reg6, uint8_t reg4)
{
    uint8_t send_text1[1] = {0};
    SPI2_ReadCommand(CTRL7_G);
    send_text1[0] |= reg7;
    SPI2_WriteCommand(CTRL7_G, send_text1);

    uint8_t send_text2[1] = {0};
    SPI2_ReadCommand(CTRL6_C);
    send_text2[0] |=reg6;
    SPI2_WriteCommand(CTRL6_C, send_text2);

    uint8_t send_text3[1] = {0};
    SPI2_ReadCommand(CTRL6_C);
    send_text3[0] |=reg4;
    SPI2_WriteCommand(CTRL6_C, send_text2);

}




//其他寄存器配置
void Set_Register(uint8_t reg_addr,uint8_t Target_text)
{
    SPI2_ReadCommand(reg_addr);
    uint8_t send_text[1] = {0};
    send_text[0] |= Target_text;
    SPI2_WriteCommand(reg_addr, send_text);
}



// uint8_t
uint8_t GetChipID(void)
{
	uint8_t reg_addr = 0x0F;

	uint8_t buf = SPI2_Read(reg_addr);
	if (buf == 0x6a)
		return 1;           
	else
		return 0;         
}   


void LSM6DS3_Reset(void)
{
    uint8_t buf[1] = {0};
    buf[0] = 0x80;
    SPI2_WriteCommand(CTRL3_C, buf);
    HAL_Delay(15);
    SPI2_ReadCommand(CTRL3_C);
    buf[0] |= 0x01;
    SPI2_WriteCommand(CTRL3_C, buf);
    while (buf[0] & 0x01)
    {
        SPI2_ReadCommand(CTRL3_C);
    }
}

//LSM6DS3 Init
uint8_t LSM6DS3_Init(void)
{
    
    //rate
    Set_Acc_Rate(ACC_RATE_13HZ); 
    Set_Gyroscope_Rate(GYR_RATE_13HZ);
    //full-scale selection.
    Set_Accelerometer_Fullscale(ACC_FSXL_2G);
    Set_Gyroscope_Fullscale(GYR_FSG_2000);
    //xyz使能
    SPI2_WriteCommand(CTRL9_XL,0x38);
    SPI2_WriteCommand(CTRL10_C, 0x38);


    Set_Accelerometer_Bandwidth(ACC_BW0XL_400HZ, ACC_HIGH_PASS_ODR_100);
    Set_Gyroscope_Bandwidth((CTRL7_G_HPM_32MHZ4 | CTRL7_G_HP_EN_ENABLE | CTRL7_G_HM_MODE_DISABLE), CTRL6_C_XL_HM_MODE_DISABLE, CTRL4_DEN_XL_EN_ENABLE);
    


    Set_Register(CTRL3_C, 0x40);//BDU  open 
    


}

void Start_Status(void)
{
    uint8_t buf[1] = {0};
    SPI2_ReadCommand(STATUS_REG);
    buf[0] = 0x08;
    SPI2_WriteCommand(STATUS_REG, buf); 
}


uint8_t Get_Status(void)
{
	uint8_t buf[1] = {0};
	buf[0] = SPI2_Read(STATUS_REG);
	return buf[0];
}


void Get_Acceleration(float *acc_float)
{
    uint8_t buf[7];
	int16_t acc[3];
    SPI_Read_Mul_Bite(OUTX_L_XL, buf);
    acc[0] = buf[2] << 8 | buf[1];
	acc[1] = buf[4] << 8 | buf[3];
	acc[2] = buf[6] << 8 | buf[5];

    acc_float[0] = ((float)acc[0] * 0.061f);
	acc_float[1] = ((float)acc[1] * 0.061f);
	acc_float[2] = ((float)acc[2] * 0.061f);


}


void Get_Gyroscope(float *gry_float)
{
    uint8_t buf[7];
	int16_t gry[3];
	SPI_Read_Mul_Bite(OUTX_L_G, buf);
	gry[0] = buf[2] << 8 | buf[1];
	gry[1] = buf[4] << 8 | buf[3];
	gry[2] = buf[6] << 8 | buf[5];
    gry_float[0] = ((float)gry[0] * 70.00f);
	gry_float[1] = ((float)gry[1] * 70.00f);
	gry_float[2] = ((float)gry[2] * 70.00f);  
}


