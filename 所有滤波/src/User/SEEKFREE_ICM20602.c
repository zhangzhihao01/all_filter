/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		ICM20602
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看common.h内VERSION宏定义
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 * @note		
					接线定义：
					------------------------------------ 
						SCL                 查看SEEKFREE_IIC文件内的SEEKFREE_SCL宏定义
						SDA                 查看SEEKFREE_IIC文件内的SEEKFREE_SDA宏定义
					------------------------------------ 
 ********************************************************************************************************************/


#include "include.h"
#include "math.h"
extern short T_tim;
int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16 icm_acc_x,icm_acc_y,icm_acc_z;



/**************
四元法参数
**************/
#define halfT    (T_tim/2000.0)             // 采样周期的一半
float Kp= 20.0f;                          // proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki =0.01f;                          // integral gain governs rate of convergence of gyroscope biases
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
float Q_ANGLE_Yaw,Q_ANGLE_Y,Q_ANGLE_X;


/****************
矩阵卡尔曼滤波法
****************/

#define dt (T_tim/1000.0)


float Angle,Gyro_y;                     //滤波后的角度，角速度
float Q_angle=0.001;                    //预测（过程）噪声方差,角度置信度
float Q_gyro=0.003;                     //预测（过程）噪声方差,角速度置信度
float R_angle=0.5;                      //测量（观测）噪声方差

char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
float w_kp=1;    


/**************
互补滤波法
**************/
float GYRO_Integration;                        //互补滤波角度
float turth_satio = 1.0f;                      //加速度计置信度，越大越相信加速度计


/****************
非矩阵卡尔曼滤波法
*****************/
float  Q_Gyro = 1;                    //陀螺仪不确定度
float  Q_Acc = 0.1;                   //加速度计不确定度
float  nf_Angle;                      //滤波角度


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602自检函数
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				调用该函数前，请先调用模拟IIC的初始化
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self1_check(void)
{
    while(0x12 != simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_WHO_AM_I,IIC)) //读取ICM20602 ID
    {
        //卡在这里原因有以下几点
        //1 MPU6050坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V
    }
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				调用该函数前，请先调用模拟IIC的初始化
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init(void)
{
    time_delay_ms(10);  //上电延时
    
    //检测
    icm20602_self1_check();
    
    //复位
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x80);               //复位设备
    time_delay_ms(2);                                                        //延时
    while(0x80 & simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,IIC));//等待复位完成
    
    //配置参数
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x01);               //时钟设置
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_2,0x00);               //开启陀螺仪和加速度计
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_CONFIG,0x01);                   //176HZ 1KHZ
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_SMPLRT_DIV,0x07);               //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_GYRO_CONFIG,0x18);              //±2000 dps
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG,0x10);             //±8g
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG_2,0x23);           //Average 8 samples   44.8HZ
}





//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata(void)
{
    uint8 dat[6];

    simiic_read_regs(ICM20602_DEV_ADDR, ICM20602_ACCEL_XOUT_H, dat, 6, IIC);  
    icm_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    icm_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    icm_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
    
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro(void)
{
    uint8 dat[6];

    simiic_read_regs(ICM20602_DEV_ADDR, ICM20602_GYRO_XOUT_H, dat, 6, IIC);  
    icm_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]))-11;
    icm_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]))-4;
    icm_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]))+1;

}



/**************************
加速度计低通滤波
输入：截至频率，原加速度
输出：低通滤波后的加速度
***************************/
float low_RC_cut(int16 V_input,int f_cut)
{
   float RC;
   float Cof1,Cof2;
   static float V_output;
   static float V_output_ago;
   
   V_output_ago=V_output;
   RC = 1/(2.0*pi*f_cut);
   Cof1 = 1/(1+RC/(T_tim*0.001));
   Cof2 = RC/(T_tim*0.001)*Cof1;
   V_output = V_input*Cof1+V_output_ago*Cof2;
   
   return V_output;
   
}



/*************************
陀螺仪高通滤波
输入：截至频率，原角速度
输出：高通滤波后的角速度
**************************/
float hight_RC_cut(int16 V_input,int f_cut)
{
   float RC;
   float Cof;
   static float V_output;
   static float V_output_ago;
   static float V_input_ago;
   
   V_output_ago=V_output;
   RC = 1/(2.0*pi*f_cut);
   Cof = RC/(RC+T_tim*0.001);
   V_output=(V_input-V_input_ago+V_output_ago)*Cof;
   V_input_ago=V_input;
      
   return V_output;
}


/*****************************
四元数求欧拉角
输入：三轴角速度
      三轴角加速度
结果：三轴欧拉角
*****************************/
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  // 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q1q1 = q1*q1;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
  if(ax*ay*az==0)
     return;
		
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc数据归一化
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
  vx = 2*(q1q3 - q0q2);							//四元素中xyz的表示
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           			 //向量外积在相减得到差分就是误差
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;							  //对误差进行积分
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   		//将误差PI后补偿到陀螺仪，即补偿零点漂移
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   		//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

  // integrate quaternion rate and normalise						   //四元素的微分方程
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  //Q_ANGLE_Yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
  Q_ANGLE_Y  = -asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  Q_ANGLE_X = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
  Q_ANGLE_Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
	
}




/*********************
卡尔曼滤波求偏角
输入：角度，角速度
输出：滤波角度
*********************/
float Kalman_Filter(float Accel,float Gyro)
{ 
    
  Angle+=(Gyro - Q_bias) * dt;                        //先验估计
  Pdot[0]=Q_angle - PP[0][1] - PP[1][0];              // Pk-先验估计误差协方差的微分
  Pdot[1]=- PP[1][1];
  Pdot[2]=- PP[1][1];
  Pdot[3]=Q_gyro;
  
  PP[0][0] += Pdot[0] * dt;                           // Pk-先验估计误差协方差微分的积分
  PP[0][1] += Pdot[1] * dt;                           // =先验估计误差协方差
  PP[1][0] += Pdot[2] * dt;
  PP[1][1] += Pdot[3] * dt;
  
  Angle_err = Accel - Angle;                          //zk-先验估计    
  PCt_0 = C_0 * PP[0][0];
  PCt_1 = C_0 * PP[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * PP[0][1];
  
  PP[0][0] -= K_0 * t_0;                              //后验估计误差协方差 
  PP[0][1] -= K_0 * t_1;
  PP[1][0] -= K_1 * t_0;
  PP[1][1] -= K_1 * t_1;

  Angle+= K_0 * Angle_err;                            //后验估计
  Q_bias+= K_1 * Angle_err;                          //后验估计
  Gyro_y   = Gyro - Q_bias;                          //输出值(后验估计)的微分=角速度   
  return  Angle; 

}



/*******************************
互补滤波
输入：加速度计反正切角度
      陀螺仪角速度
输出：滤波角度
********************************/
float Complement_Filter(float angle_m,float gyro_m)
{ 
    float Angle_Filter_Temp;                                                         //互补融合滤波中间量
    float Angle_Difference_Value;                                                   //融合后的角度与加速度静态角度差值，作为反馈量加入积分回路，让静差为0
    
    Angle_Filter_Temp=GYRO_Integration;
    Angle_Difference_Value=(angle_m-Angle_Filter_Temp)*turth_satio;                  //融合后的角度与加速度静态角度差值
    GYRO_Integration=GYRO_Integration+(gyro_m+Angle_Difference_Value )*dt;
    return  GYRO_Integration;
} 
 



/***********************
卡尔曼滤波非矩阵形式
************************/

float  N_Kalman_Filter(float Accel,float Gyro)
{
   static float fore_date;        //预测值
   static float fore_noise_date;  //预测值噪声偏差
   static float noise_square;     //噪声均方差
   static float real_angle;       //滤波后角度
   static float best_error;       //最优误差
   
   
   fore_date = real_angle+Gyro*dt;                                       //预测值=上一次最优值加陀螺仪积分
   fore_noise_date = sqrt(Q_Gyro*Q_Gyro+best_error*best_error);          //预测值噪声偏差
   noise_square = sqrt((fore_noise_date*fore_noise_date)/(fore_noise_date*fore_noise_date+Q_Acc*Q_Acc));         //噪声均方差
   real_angle = fore_date+noise_square*(Accel-fore_date);                //最优角度=预测角度+噪声均方差*（测量值-预测值）
   best_error = sqrt((1-noise_square)*fore_noise_date*fore_noise_date);  //更新最优误差
   
   nf_Angle = real_angle;
     
   return real_angle;
   
}



