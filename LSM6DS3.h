#ifndef __LSM6DS_H
#define __LSM6DS_H

#include "stm32f3xx_hal.h"



#define LSM6DS3TRC_WHO_AM_I		0x0F
#define CTRL1_XL  0x10//线性加速度传感器控制寄存器1 (r/w)
/*  bit7						~						  bit0
	ODR_XL3 ODR_XL2 ODR_XL1 ODR_XL0 FS_XL1 FS_XL0 BW_XL1 BW_XL0
	其中:ODR_XL3 ODR_XL2 ODR_XL1 ODR_XL0 输出数据速率和电源模式选择
	FS_XL1 FS_XL0：ACC满量程选择
	BW_XL1 BW_XL0：抗混叠滤波器带宽选择。
*/
//初始化设置定义
//速率
#define ACC_RATE_0	0x00
#define ACC_RATE_1HZ6	0xB0
#define ACC_RATE_13HZ	0x10
#define ACC_RATE_26HZ	0x20
#define ACC_RATE_52HZ	0x30
#define ACC_RATE_104HZ	0x40
#define ACC_RATE_208HZ	0x50
#define ACC_RATE_416HZ	0x60
#define ACC_RATE_833HZ	0x70
#define ACC_RATE_1660HZ	0x80
#define ACC_RATE_3330HZ	0x90
#define ACC_RATE_6660HZ	0xA0

//量程
#define ACC_FSXL_2G	0x00
#define ACC_FSXL_16G 0x04
#define ACC_FSXL_4G	0x08
#define ACC_FSXL_8G	0x0C

//带宽

#define ACC_BW0XL_400HZ	0x01
//Accelerometer bandwidth selection

#define ACC_HIGH_PASS_ODR_100 0x24


//11010000    0xd0
#define CTRL7_G_HM_MODE_DISABLE	0x80
#define CTRL7_G_HP_EN_ENABLE 0x40
#define CTRL7_G_HPM_32MHZ4	0x10


#define CTRL6_C_XL_HM_MODE_DISABLE	0x10
#define CTRL4_DEN_XL_EN_ENABLE	0x80      //10000000

#define CTRL2_G  0x11
/*  bit7						~			   bit0
	ODR_G3  ODR_G2 ODR_G1 ODR_G0 FS_G1  FS_G0 FS_125
	其中:ODR_G3  ODR_G2 ODR_G1 ODR_G0:陀螺仪输出速率选择
	FS_G1  FS_G0：陀螺仪满量程选择
	FS_125：陀螺仪满量程为125dps；	
*/
//角速度速率
#define GYR_RATE_0	0x00
#define GYR_RATE_13HZ	0x10
#define GYR_RATE_26HZ	0x20
#define GYR_RATE_52HZ	0x30
#define GYR_RATE_104HZ	0x40
#define GYR_RATE_208HZ	0x50
#define GYR_RATE_416HZ	0x60
#define GYR_RATE_833HZ	0x70
#define GYR_RATE_1660HZ	0x80

//角速度量程
#define GYR_FSG_245	0x00
#define GYR_FSG_500	0x04
#define GYR_FSG_1000	0x08
#define GYR_FSG_2000	0x0C


#define CTRL3_C  0x12
/*  bit7						~			   bit0
	BooT BDU H_LACTIVE PP_OD SIM IF_INC BLE SW_RESET
	BooT:重启内存里的内容
	BDU：更新块数据
	H_LACTIVE：中断活跃水平
	PP_OD：INT1和INT2衬垫上的推拉/开式排水选择。
	SIM：SPI Serial interface mode select
	IF_INC:寄存器地址在进行串行多字节访问时自动增加
	BLE：大/小端数据选择
	SW_RESET：软件复位
*/
#define CTRL4_C  0X13
/*  bit7						~			   bit0
	XL_BW_SCAL_ODR:加速度计带宽的选择;
	SLEEP_G:陀螺仪睡眠模式使能;
	INT2_on_INT1:所有中断信号在INT1上可用
	FIFO_TEMP_EN:使能稳如数据作为4th FIFO数据设置
	DRDY_MASK:数据读掩码使能;
	I2C_disable:disable i2c接口
	0
	STOP_ON_FTH:启用FIFO阈值级别使用;
*/
#define LSM6DS3_CTRL5_C  0X14
/*
	ROUNDING2 ROUNDING1 ROUNDING0 0 ST1_G  ST0_G ST1_XL ST0_XL
	ROUNDING2 ROUNDING1 ROUNDING0:从输出寄存器中读取循环突发模式(舍入)。
	ST1_G  ST0_G:角速率传感器自检使能。
	ST1_XL ST0_XL:线性加速度传感器自检使能。
*/
#define CTRL6_C  0x15
/*
	角速率传感器控制寄存器
	TRIG_EN LVLen LVL2_EN XL_HM_MODE 0 0 0 0
	TRIG_EN:陀螺仪数据边缘敏感触发启用。
	LVLen:陀螺仪数据电平敏感触发启用。
	LVL2_EN:陀螺仪电平敏感锁存使能。
	XL_HM_MODE:加速计高性能工作模式禁用.
*/
#define CTRL7_G  0x16
/*
	角速率传感器控制寄存器7
	G_HM_MODE HP_G_EN HPCF_G1 HPCF_G0 HP_G_RST ROUNDING_STATUE 0 0
	G_HM_MODE:陀螺仪的高性能工作模式禁用
	HP_G_EN:陀螺数字高通滤波器使能。只有当陀螺仪处于HP模式时，才启用过滤器。
	HPCF_G1 HPCF_G0:陀螺仪高通滤波器截止频率选择。
	HP_G_RST:陀螺数字HP滤波器复位
	ROUNDING_STATUE:源寄存器四舍五入功能使能在1E,53,1B	
*/
#define CTRL8_XL  0x17
/*
	线性加速度传感器控制寄存器8 (r/w)。
	LPF2_XL_EN	HPCF_XL1 HPCF_XL0 0 0 HP_SLOPE_XL_EN 0 LOW_PASS_ON_6D
	LPF2_XL_EN: 加速度计低通滤波器LPF2选择。
	HPCF_XL1 HPCF_XL0:加速度计斜率滤波器和高通滤波器配置和截止设置。
		见表68。它还用于选择LPF2滤波器的截止频率，如表69所示。通过将CTRL8XL (17h)的LOW PASS ON 6D位设置为1，这种低通滤波器也可以用于6D/4D功能。
	HP_SLOPE_XL_EN:加速度计斜率滤波器/高通滤波器选择。
	LOW_PASS_ON_6D:低通滤波器对6D功能的选择。
*/
#define CTRL9_XL  0x18
/*
	0 0 Zen_XL Yen_XL Xen_XL SOFT_EN 0 0
	Zen_XL:加速计z轴输出启用。
	Yen_XL:加速计y轴输出启用。
	Xen_XL：加速计x轴输出启用.
	SOFT_EN:启用磁强计软铁校正算法
*/
#define CTRL10_C  0x19
/*
	0 0 Zen_G Yen_G Xen_G FUNC_EN FEDO_RST_STEP SIGN_MOTION_EN
	Zen_G:陀螺偏航轴(Z)输出使能。
	Yen_G:陀螺滚轴(Y)输出使能。
	Xen_G:陀螺螺距轴(X)输出使能。
	FUNC_EN:启用嵌入式功能(计步器、倾斜、显著运动、传感器轮毂和熨烫)和加速度计HP和LPF2滤波器
		(参见图6)。默认值:0
	FEDO_RST_STEP:重置计步器步长计数器。
	SIGN_MOTION_EN:使能重要运动功能
*/
#define MASTER_CONFIG  0X1A

//interrupts register
#define WAKE_UP_SRC  0X1B
/*
	0 0 FF_IA SLEEP_STATE_IA WU_IA X_WU Y_WU Z_WU
	FF_IA:自由落体时间检测状态
	SLEEP_STATE_IA:睡眠时间状态
	WU_IA:唤醒时间检测状态
	X_WU：x轴上的唤醒事件检测状态。
	Y_WU: y轴上的唤醒事件检测状态。
	Z_WU: z轴上的唤醒事件检测状态。
*/
#define TAP_SRC                 0X1C
/*
	0 TAP_IA SIGLE_TAP DOUBLE_TAP TAP_SIGN X_TAP Y_TAP Z_TAP
	TAP_IA:轻击事件检测状态
	SIGLE_TAP:单击事件检测状态
	DOUBLE_TAP:双击事件检测状态
	TAP_SIGN：轻击事件检测到的加速标志。
	X_TAP Y_TAP Z_TAP：点击x/y/z轴上的事件检测状态
*/

#define D6D_SRC                 0X1D
/*
	纵向、横向、面朝上和面朝下源寄存器(r)
	0 D6D_IA ZH ZL YH YL XH XL 
	D6D_IA：激活中断以改变位置纵向，横向，正面向上，正面向下。
	ZH: Z轴高事件(高于阈值)
	ZL：Z轴低事件(低于阈值)
	...
*/
//status data register
#define STATUS_REG              0X1E
/*
	- - - - - TDE GDA XLDA
	TDE:温度可用的新数据
	GDA:陀螺仪可用的新数据
	XLDA:加速度计温度可用的新数据
*/
#define STATUS_GYROSCOPE 0x02
#define STATUS_ACCELEROMETER 0x01


#define TAP_CFG  0x58
#define TAP_THS_6D  0x59
#define WAKE_UP_THS  0x5B
#define WAKE_UP_DUR  0x5C
#define MD1_CFG  0x5E
#define MD2_CFG  0x5F
#define INT1_CTRL  0x0D
#define INT2_CTRL  0x0E



//gyroscope output register
#define OUTX_L_G  0X22
#define OUTX_H_G  0X23
#define OUTY_L_G  0X24
#define OUTY_H_G  0X25
#define OUTZ_L_G  0X26
#define OUTZ_H_G  0X27

//acc output register
#define OUTX_L_XL  0X28
#define OUTX_H_XL  0X29
#define OUTY_L_XL  0X2A
#define OUTY_H_XL  0X2B
#define OUTZ_L_XL  0X2C
#define OUTZ_H_XL  0X2D


uint8_t GetChipID(void);
void Set_Acc_Rate(uint8_t rate);
void Set_Accelerometer_Fullscale(uint8_t value);
void Set_Gyroscope_Rate(uint8_t rate);
void Set_Gyroscope_Fullscale(uint8_t value);
void Set_Accelerometer_Bandwidth(uint8_t BW0XL, uint8_t ODR);
void Set_Gyroscope_Bandwidth(uint8_t reg7, uint8_t reg6, uint8_t reg4);
void Set_Register(uint8_t reg_addr,uint8_t Target_text);
uint8_t Get_Status(void);
void Start_Status(void);
void Get_Acceleration(float *acc_float);
void Get_Gyroscope(float *gry_float);
uint8_t LSM6DS3_Init(void);



#endif
