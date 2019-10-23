#ifndef __U_IIC_H
#define __U_IIC_H


#include "include.h"


#define SCL_PORT  PTD    //SCL使用D端口     1.修改模拟IIC引脚时需要修改
#define SCL_INDEX 8       //SCL使用D8引脚
#define SDA_PORT  PTD    //SDA使用D端口     
#define SDA_INDEX 9      //SDA使用D9引脚
//驱动MPU6050接口，GPIO模拟IIC

#define SDA_IN()  {GPIO_PDDR_REG(SDA_PORT) &= ~(1<<SDA_INDEX);}	//输入
#define SDA_OUT() {GPIO_PDDR_REG(SDA_PORT) |= (1<<SDA_INDEX);} //输出

//IO操作函数	 
#define IIC_SCL    PTD8_OUT //SCL             2.修改模拟iic引脚时需要修改
#define IIC_SDA    PTD9_OUT //SDA	 
#define READ_SDA   PTD9_IN  //输入SDA 


//IIC_1所有操作函数
void delay_nop(int n);
void IIC_Init(void);        //初始化IIC的IO口3.修改模拟iic引脚时需要修改			 
void IIC_Start(void);			  //发送IIC开始信号
void IIC_Stop(void);	  	  //发送IIC停止信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
uint8_t IIC_WaitAck(void); 		 //IIC等待ACK信号
void IIC_SendByte(uint8_t data);  //IIC发送一个字节
uint8_t IIC_ReadByte(uint8_t ack);//IIC读取一个字节
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);



#endif