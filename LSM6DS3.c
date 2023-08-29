#include "LSM6DS3.h"
#include "stdio.h"
#include "spi.h"
#include "math.h"


void Get_Acceleration(float *acc_float)
{
    uint8_t buf[6];
	int16_t acc[3];
    SPI_Read_Mul_Bite(buf, OUTX_L_XL_REG, 6);
    acc[0] = buf[1] << 8 | buf[0];
	acc[1] = buf[3] << 8 | buf[2];
	acc[2] = buf[5] << 8 | buf[4];

    acc_float[0] = ((float)acc[0] / 2049.18);
	acc_float[1] = ((float)acc[1] / 2049.18);
	acc_float[2] = ((float)acc[2] / 2049.18);
    

}


void Get_Gyroscope(float *gry_float)
{
    uint8_t buf[6];
	int16_t gry[3];
	SPI_Read_Mul_Bite(buf,OUTX_L_G_REG, 6);
	gry[0] = buf[1] << 8 | buf[0];
	gry[1] = buf[3] << 8 | buf[2];
	gry[2] = buf[5] << 8 | buf[4];
    gry_float[0] = ((float)gry[0] / 14.29);
	gry_float[1] = ((float)gry[1] / 14.29);
	gry_float[2] = ((float)gry[2] / 14.29);  
    
}

// void Get_Data(float *Gyr_DMA_Data, float *Acc_DMA_Data)
// {
// 	uint8_t buf[12];
// 	uint16_t gry[3];
// 	uint16_t acc[3];
// 	SPI_Read_Mul_Bite(buf,OUTX_L_G_REG, 12);
// 	gry[0] = buf[1] << 8 | buf[0];
// 	gry[1] = buf[3] << 8 | buf[2];
// 	gry[2] = buf[5] << 8 | buf[4];
// 	acc[0] = buf[7] << 8 | buf[6];
// 	acc[1] = buf[9] << 8 | buf[8];
// 	acc[2] = buf[11] << 8 | buf[10];

// 	Acc_DMA_Data[0] = (float)(acc[0] / 2049.18);
// 	Acc_DMA_Data[1] = (float)(acc[1] / 2049.18);
// 	Acc_DMA_Data[2] = (float)(acc[2] / 2049.18);
// 	Gyr_DMA_Data[0] = (float)(gry[0] / 14.29);
// 	Gyr_DMA_Data[1] = (float)(gry[1] / 14.29);
// 	Gyr_DMA_Data[2] = (float)(gry[2] / 14.29);  



// }



//float Acc_Temp[3] = {0};
void Get_Acc_Compensate(float *Acc_Temp)
{
	uint8_t i;
	float x = 0;
	float y = 0;
	float z = 0;
	for(i = 0; i < 200; i++)
	{
		float Acc[3] = {0};
		Get_Acceleration(&Acc);
		x += Acc[0];
		y += Acc[1];
		z += Acc[2];

	}
	Acc_Temp[0] = (float)(x / 200);
	Acc_Temp[1] = (float)(y / 200);
	Acc_Temp[2] = (float)(z / 200);
}

//float Gyro_Temp[3] = {0};
void Get_Gyro_Compensate(float *Gyro_Temp)
{
	uint8_t i;
	float x = 0;
	float y = 0;
	float z = 0;
	for(i = 0; i <200; i++) 
	{
		float Gyro[3] = {0};
		Get_Gyroscope(&Gyro);
		x += Gyro[0];//sum of x , y , z
		y += Gyro[1];
		z += Gyro[2];
	}
	Gyro_Temp[0] = (float)(x / 200);
	Gyro_Temp[1] = (float)(y / 200);
	Gyro_Temp[2] = (float)(z / 200);
}



//test
void LSM6DS3_init(void)
{
	uint8_t sw = 0x00;
	sw = (uint8_t)(0x01);
	SPI2_Write(&sw,CTRL3_C_REG);



	LSM6DS3Gyro_InitTypeDef LSM6DS3Gyro_InitStructure;  
	LSM6DS3Gyro_InitStructure.Output_DataRate = ODR_G_1k66Hz;
	LSM6DS3Gyro_InitStructure.Full_Scale = FS_G_2000dps;
	LSM6DS3Gyro_InitStructure.BlockData_Update = BlockDataUpdate_Continous;  
	LSM6DS3Gyro_InitStructure.Endianness = MSB_FIRST;  
	LSM6DS3Gyro_InitStructure.Register_read = Register_read_continuous;  


	LSM6DS3Gyro_Cmd(&LSM6DS3Gyro_InitStructure);


	LSM6DS3XL_InitTypeDef   LSM6DS3XL_InitStructure;
	LSM6DS3XL_InitStructure.Full_Scale = FS_XL_16g;
	LSM6DS3XL_InitStructure.Output_DataRate = ODR_XL_1k66Hz;
	LSM6DS3XL_InitStructure.Band_Width = BW_XL_400Hz;
	LSM6DS3XL_InitStructure.Endianness = MSB_FIRST;
	LSM6DS3XL_InitStructure.BlockData_Update = BlockDataUpdate_Continous;
	LSM6DS3XL_InitStructure.BandWidth_Source = BandWidth_Default;  
	LSM6DS3XL_InitStructure.Register_read = Register_read_continuous;  

	

	LSM6DS3XL_Cmd(&LSM6DS3XL_InitStructure);
//high pass filter
	LSM6DS3_InitfilterTypeDef    LSM6DS3_InitFilter;
	LSM6DS3_InitFilter.CTRL6_Filter = Filter_6;
	LSM6DS3_InitFilter.CTRL7_Filter = Filter_7a;
	LSM6DS3_InitFilter.CTRL8_Filter = Filter_8;
	LSM6DS3Filter_Cmd(&LSM6DS3_InitFilter);

}


//打开中断
void LSM6DS3_INT1(void)
{
	
	LSM6DS3_InitINT1TypeDef   LSM6DS3_InitINT1;
	LSM6DS3_InitINT1.INT1_CTRL = INT1_CTRL_ACC;
	LSM6DS3INT1_Cmd(&LSM6DS3_InitINT1);

	// float acc_float[3] = {0};
	// Get_Acceleration(acc_float);
	
}
void LSM6DS3_INT2(void)
{
	LSM6DS3_InitINT2TypeDef   LSM6DS3_InitINT2;
	LSM6DS3_InitINT2.INT2_CTRL = INT2_CTRL_GYRO;
	LSM6DS3INT2_Cmd(&LSM6DS3_InitINT2);
	float gry_float[3] = {0};
    Get_Gyroscope(gry_float);
}
//关闭中断
void LSM6DS3_INT1_Close(void)
{
	
	LSM6DS3_InitINT1TypeDef   LSM6DS3_InitINT1;
	LSM6DS3_InitINT1.INT1_CTRL = 0x00;
	LSM6DS3INT1_Cmd(&LSM6DS3_InitINT1);
	
}
void LSM6DS3_INT2_Close(void)
{
	LSM6DS3_InitINT2TypeDef   LSM6DS3_InitINT2;
	LSM6DS3_InitINT2.INT2_CTRL = 0x00;
	LSM6DS3INT2_Cmd(&LSM6DS3_InitINT2);

}





/*set Gyro*/
void LSM6DS3Gyro_Cmd(LSM6DS3Gyro_InitTypeDef* LSM6DS3Gyro_InitStructure)
{
	uint8_t ctrl2 = 0x00;
	uint8_t ctrl3 = 0x00;
	

	ctrl2 |= (uint8_t)(LSM6DS3Gyro_InitStructure->Output_DataRate|LSM6DS3Gyro_InitStructure->Full_Scale);
	ctrl3 |= (uint8_t)(LSM6DS3Gyro_InitStructure->BlockData_Update|LSM6DS3Gyro_InitStructure->Endianness);
	ctrl3 |= (uint8_t)LSM6DS3Gyro_InitStructure->Register_read;
	
	
	SPI2_Write(&ctrl2,CTRL2_G_REG);
	SPI2_Write(&ctrl3,CTRL3_C_REG);
	

}
void LSM6DS3XL_Cmd(LSM6DS3XL_InitTypeDef* LSM6DS3XL_InitStructure)
{
	uint8_t ctrl1 = 0x00;
	uint8_t ctrl3 = 0x00;
	uint8_t ctrl4 = 0x00;
	

	ctrl1 |= (uint8_t)(LSM6DS3XL_InitStructure->Band_Width|LSM6DS3XL_InitStructure->Full_Scale|LSM6DS3XL_InitStructure->Output_DataRate);
	ctrl3 |= (uint8_t)(LSM6DS3XL_InitStructure->Endianness|LSM6DS3XL_InitStructure->BlockData_Update);
	ctrl4 |= (uint8_t)(LSM6DS3XL_InitStructure->BandWidth_Source);
	ctrl3 |= (uint8_t)LSM6DS3XL_InitStructure->Register_read;
	

	SPI2_Write(&ctrl1,CTRL1_XL_REG);
	SPI2_Write(&ctrl3,CTRL3_C_REG);
	

}
void LSM6DS3INT1_Cmd(LSM6DS3_InitINT1TypeDef* LSM6DS3_InitINT1)
{
	
	uint8_t ctr1 = 0x00;

	
	ctr1 |= (uint8_t)LSM6DS3_InitINT1->INT1_CTRL;

	
	SPI2_Write(&ctr1,INT1_CTRL_REG);
}
void LSM6DS3INT2_Cmd(LSM6DS3_InitINT2TypeDef* LSM6DS3_InitINT2)
{
	uint8_t ctrl = 0x00;
	ctrl |= (uint8_t)LSM6DS3_InitINT2->INT2_CTRL;
	SPI2_Write(&ctrl,INT2_CTRL_REG);
}









/*initialize LSM6DS3*/
// void LSM6DS3_init(void)
// {
// 	uint8_t sw = 0x00;
// 	sw = (uint8_t)(0x01);
// 	SPI2_Write(&sw,CTRL3_C_REG);



// 	LSM6DS3Gyro_InitTypeDef LSM6DS3Gyro_InitStructure;  
// 	LSM6DS3Gyro_InitStructure.Output_DataRate = ODR_G_1k66Hz;
// 	LSM6DS3Gyro_InitStructure.Full_Scale = FS_G_2000dps;
// 	LSM6DS3Gyro_InitStructure.BlockData_Update = BlockDataUpdate_Single;  
// 	LSM6DS3Gyro_InitStructure.Endianness = MSB_FIRST;  
// 	LSM6DS3Gyro_InitStructure.Register_read = Register_read_continuous;  

// 	LSM6DS3_InitINT2TypeDef   LSM6DS3_InitINT2;
// 	LSM6DS3_InitINT2.INT2_CTRL = INT2_CTRL_GYRO;
	
// 	LSM6DS3Gyro_Cmd(&LSM6DS3Gyro_InitStructure, &LSM6DS3_InitINT2);


// 	LSM6DS3XL_InitTypeDef   LSM6DS3XL_InitStructure;
// 	LSM6DS3XL_InitStructure.Full_Scale = FS_XL_16g;
// 	LSM6DS3XL_InitStructure.Output_DataRate = ODR_XL_1k66Hz;
// 	LSM6DS3XL_InitStructure.Band_Width = BW_XL_400Hz;
// 	LSM6DS3XL_InitStructure.Endianness = MSB_FIRST;
// 	LSM6DS3XL_InitStructure.BlockData_Update = BlockDataUpdate_Single;
// 	LSM6DS3XL_InitStructure.BandWidth_Source = BandWidth_Default;  
// 	LSM6DS3XL_InitStructure.Register_read = Register_read_continuous;  

// 	LSM6DS3_InitINT1TypeDef   LSM6DS3_InitINT1;
// 	LSM6DS3_InitINT1.INT1_CTRL = INT1_CTRL_ACC;

// 	LSM6DS3XL_Cmd(&LSM6DS3XL_InitStructure, &LSM6DS3_InitINT1);
// //high pass filter
// 	LSM6DS3_InitfilterTypeDef    LSM6DS3_InitFilter;
// 	LSM6DS3_InitFilter.CTRL6_Filter = Filter_6;
// 	LSM6DS3_InitFilter.CTRL7_Filter = Filter_7a;
// 	LSM6DS3_InitFilter.CTRL8_Filter = Filter_8;
// 	LSM6DS3Filter_Cmd(&LSM6DS3_InitFilter);


// 	float	acc_float[3] = {0};
// 	Get_Acceleration(acc_float);
// 	float gry_float[3] = {0};
//     Get_Gyroscope(gry_float);

// }







// /*set Gyro*/
// void LSM6DS3Gyro_Cmd(LSM6DS3Gyro_InitTypeDef* LSM6DS3Gyro_InitStructure, LSM6DS3_InitINT2TypeDef* LSM6DS3_InitINT2)
// {
// 	uint8_t ctrl2 = 0x00;
// 	uint8_t ctrl3 = 0x00;
// 	uint8_t ctrl = 0x00;

// 	ctrl2 |= (uint8_t)(LSM6DS3Gyro_InitStructure->Output_DataRate|LSM6DS3Gyro_InitStructure->Full_Scale);
// 	ctrl3 |= (uint8_t)(LSM6DS3Gyro_InitStructure->BlockData_Update|LSM6DS3Gyro_InitStructure->Endianness);
// 	ctrl3 |= (uint8_t)LSM6DS3Gyro_InitStructure->Register_read;
// 	ctrl |= (uint8_t)LSM6DS3_InitINT2->INT2_CTRL;
	
// 	SPI2_Write(&ctrl2,CTRL2_G_REG);
// 	SPI2_Write(&ctrl3,CTRL3_C_REG);
// 	SPI2_Write(&ctrl,INT2_CTRL_REG);

// }
// void LSM6DS3XL_Cmd(LSM6DS3XL_InitTypeDef* LSM6DS3XL_InitStructure, LSM6DS3_InitINT1TypeDef* LSM6DS3_InitINT1)
// {
// 	uint8_t ctrl1 = 0x00;
// 	uint8_t ctrl3 = 0x00;
// 	uint8_t ctrl4 = 0x00;
// 	uint8_t ctrl = 0x00;

// 	ctrl1 |= (uint8_t)(LSM6DS3XL_InitStructure->Band_Width|LSM6DS3XL_InitStructure->Full_Scale|LSM6DS3XL_InitStructure->Output_DataRate);
// 	ctrl3 |= (uint8_t)(LSM6DS3XL_InitStructure->Endianness|LSM6DS3XL_InitStructure->BlockData_Update);
// 	ctrl4 |= (uint8_t)(LSM6DS3XL_InitStructure->BandWidth_Source);
// 	ctrl3 |= (uint8_t)LSM6DS3XL_InitStructure->Register_read;
// 	ctrl |= (uint8_t)LSM6DS3_InitINT1->INT1_CTRL;

// 	SPI2_Write(&ctrl1,CTRL1_XL_REG);
// 	SPI2_Write(&ctrl3,CTRL3_C_REG);
// 	SPI2_Write(&ctrl,INT1_CTRL_REG);

// }

void LSM6DS3Filter_Cmd(LSM6DS3_InitfilterTypeDef* LSM6DS3_InitFilter)
{
	uint8_t ctrl6 = 0x00;
	uint8_t ctrl7 = 0x00;
	uint8_t ctrl8 = 0x00;
	ctrl6 |= (uint8_t)LSM6DS3_InitFilter->CTRL6_Filter;
	ctrl7 |= (uint8_t)LSM6DS3_InitFilter->CTRL7_Filter;
	ctrl8 |= (uint8_t)LSM6DS3_InitFilter->CTRL8_Filter;
	SPI2_Write(&ctrl6,CTRL6_C_REG);
	SPI2_Write(&ctrl7,CTRL7_G_REG);
	SPI2_Write(&ctrl8,CTRL8_XL_REG);

}



// void LSM6DS3XL_OFF(void)
// {
// 	uint8_t ACC_OFF_CTRL1 = 0x00;
// 	uint8_t ACC_OFF_CTRL3 = 0x04;
// 	uint8_t ACC_OFF_CTRL_REG = 0x00;
// 	SPI2_Write(&ACC_OFF_CTRL1,CTRL1_XL_REG);
// 	SPI2_Write(&ACC_OFF_CTRL3,CTRL3_C_REG);
// 	SPI2_Write(&ACC_OFF_CTRL_REG,INT1_CTRL_REG);
// }

// void LSM6DS3Gyro_OFF(void)
// {
// 	uint8_t GYRO_OFF_CTRL2 = 0x00;
// 	uint8_t GYRO_OFF_CTRL3 = 0x04;
// 	uint8_t GYRO_OFF_CTRL_REG = 0x00;
// 	SPI2_Write(&GYRO_OFF_CTRL2,CTRL2_G_REG);
// 	SPI2_Write(&GYRO_OFF_CTRL3,CTRL3_C_REG);
// 	SPI2_Write(&GYRO_OFF_CTRL_REG,INT2_CTRL_REG);
// }













 //加速度单位g，陀螺仪rad/s
void IMUupdate(float *Angle_Data)
{
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;  
		float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
		float exInt = 0, eyInt = 0, ezInt = 0;        // 按比例缩小积分误差
 		float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻滚角

		float gx = 0;
		float gy = 0;
		float gz = 0;
		float ax = 0;
		float ay = 0;
		float az = 0;
		float acc_data[3] = {0};
		float gyro_data[3] = {0};
		Get_Acceleration(&acc_data);
		Get_Gyroscope(&gyro_data);
		gx = gyro_data[0];
		gy = gyro_data[1];
		gz = gyro_data[2];
		ax = acc_data[0];
		ay = acc_data[0];
		az = acc_data[0];

        // 测量正常化
        norm = sqrt(ax*ax + ay*ay + az*az);      
        ax = ax / norm;                   //单位化
        ay = ay / norm;
        az = az / norm;      
 
        // 估计方向的重力
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
 
        // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);
 
        // 积分误差比例积分增益
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
 
        // 调整后的陀螺仪测量
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
 
        // 整合四元数率和正常化
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
 
        // 正常化四元
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
		
        Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch ,转换为度数
        Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
        //Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;   //偏移太大
		Angle_Data[0] = Pitch;
		Angle_Data[1] = Roll;

}










