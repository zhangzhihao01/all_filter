/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		ICM20602
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴common.h��VERSION�궨��
 * @Software 		IAR 7.8 or MDK 5.24a
 * @Target core		LPC54606J512BD100
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 * @note		
					���߶��壺
					------------------------------------ 
						SCL                 �鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SCL�궨��
						SDA                 �鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SDA�궨��
					------------------------------------ 
 ********************************************************************************************************************/


#include "include.h"
#include "math.h"
extern short T_tim;
int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16 icm_acc_x,icm_acc_y,icm_acc_z;



/**************
��Ԫ������
**************/
#define halfT    (T_tim/2000.0)             // �������ڵ�һ��
float Kp= 20.0f;                          // proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki =0.01f;                          // integral gain governs rate of convergence of gyroscope biases
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
float Q_ANGLE_Yaw,Q_ANGLE_Y,Q_ANGLE_X;


/****************
���󿨶����˲���
****************/

#define dt (T_tim/1000.0)


float Angle,Gyro_y;                     //�˲���ĽǶȣ����ٶ�
float Q_angle=0.001;                    //Ԥ�⣨���̣���������,�Ƕ����Ŷ�
float Q_gyro=0.003;                     //Ԥ�⣨���̣���������,���ٶ����Ŷ�
float R_angle=0.5;                      //�������۲⣩��������

char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
float w_kp=1;    


/**************
�����˲���
**************/
float GYRO_Integration;                        //�����˲��Ƕ�
float turth_satio = 1.0f;                      //���ٶȼ����Ŷȣ�Խ��Խ���ż��ٶȼ�


/****************
�Ǿ��󿨶����˲���
*****************/
float  Q_Gyro = 1;                    //�����ǲ�ȷ����
float  Q_Acc = 0.1;                   //���ٶȼƲ�ȷ����
float  nf_Angle;                      //�˲��Ƕ�


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602�Լ캯��
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				���øú���ǰ�����ȵ���ģ��IIC�ĳ�ʼ��
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self1_check(void)
{
    while(0x12 != simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_WHO_AM_I,IIC)) //��ȡICM20602 ID
    {
        //��������ԭ�������¼���
        //1 MPU6050���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
    }
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				���øú���ǰ�����ȵ���ģ��IIC�ĳ�ʼ��
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init(void)
{
    time_delay_ms(10);  //�ϵ���ʱ
    
    //���
    icm20602_self1_check();
    
    //��λ
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x80);               //��λ�豸
    time_delay_ms(2);                                                        //��ʱ
    while(0x80 & simiic_read_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,IIC));//�ȴ���λ���
    
    //���ò���
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x01);               //ʱ������
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_2,0x00);               //���������Ǻͼ��ٶȼ�
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_CONFIG,0x01);                   //176HZ 1KHZ
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_SMPLRT_DIV,0x07);               //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_GYRO_CONFIG,0x18);              //��2000 dps
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG,0x10);             //��8g
    simiic_write_reg(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG_2,0x23);           //Average 8 samples   44.8HZ
}





//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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
//  @brief      ��ȡICM20602����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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
���ٶȼƵ�ͨ�˲�
���룺����Ƶ�ʣ�ԭ���ٶ�
�������ͨ�˲���ļ��ٶ�
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
�����Ǹ�ͨ�˲�
���룺����Ƶ�ʣ�ԭ���ٶ�
�������ͨ�˲���Ľ��ٶ�
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
��Ԫ����ŷ����
���룺������ٶ�
      ����Ǽ��ٶ�
���������ŷ����
*****************************/
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  // �Ȱ���Щ�õõ���ֵ���
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
		
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc���ݹ�һ��
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              �����������������/��Ǩ
  vx = 2*(q1q3 - q0q2);							//��Ԫ����xyz�ı�ʾ
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           			 //�������������õ���־������
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;							  //�������л���
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   		//�����PI�󲹳��������ǣ����������Ư��
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   		//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�

  // integrate quaternion rate and normalise						   //��Ԫ�ص�΢�ַ���
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
�������˲���ƫ��
���룺�Ƕȣ����ٶ�
������˲��Ƕ�
*********************/
float Kalman_Filter(float Accel,float Gyro)
{ 
    
  Angle+=(Gyro - Q_bias) * dt;                        //�������
  Pdot[0]=Q_angle - PP[0][1] - PP[1][0];              // Pk-����������Э�����΢��
  Pdot[1]=- PP[1][1];
  Pdot[2]=- PP[1][1];
  Pdot[3]=Q_gyro;
  
  PP[0][0] += Pdot[0] * dt;                           // Pk-����������Э����΢�ֵĻ���
  PP[0][1] += Pdot[1] * dt;                           // =����������Э����
  PP[1][0] += Pdot[2] * dt;
  PP[1][1] += Pdot[3] * dt;
  
  Angle_err = Accel - Angle;                          //zk-�������    
  PCt_0 = C_0 * PP[0][0];
  PCt_1 = C_0 * PP[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * PP[0][1];
  
  PP[0][0] -= K_0 * t_0;                              //����������Э���� 
  PP[0][1] -= K_0 * t_1;
  PP[1][0] -= K_1 * t_0;
  PP[1][1] -= K_1 * t_1;

  Angle+= K_0 * Angle_err;                            //�������
  Q_bias+= K_1 * Angle_err;                          //�������
  Gyro_y   = Gyro - Q_bias;                          //���ֵ(�������)��΢��=���ٶ�   
  return  Angle; 

}



/*******************************
�����˲�
���룺���ٶȼƷ����нǶ�
      �����ǽ��ٶ�
������˲��Ƕ�
********************************/
float Complement_Filter(float angle_m,float gyro_m)
{ 
    float Angle_Filter_Temp;                                                         //�����ں��˲��м���
    float Angle_Difference_Value;                                                   //�ںϺ�ĽǶ�����ٶȾ�̬�ǶȲ�ֵ����Ϊ������������ֻ�·���þ���Ϊ0
    
    Angle_Filter_Temp=GYRO_Integration;
    Angle_Difference_Value=(angle_m-Angle_Filter_Temp)*turth_satio;                  //�ںϺ�ĽǶ�����ٶȾ�̬�ǶȲ�ֵ
    GYRO_Integration=GYRO_Integration+(gyro_m+Angle_Difference_Value )*dt;
    return  GYRO_Integration;
} 
 



/***********************
�������˲��Ǿ�����ʽ
************************/

float  N_Kalman_Filter(float Accel,float Gyro)
{
   static float fore_date;        //Ԥ��ֵ
   static float fore_noise_date;  //Ԥ��ֵ����ƫ��
   static float noise_square;     //����������
   static float real_angle;       //�˲���Ƕ�
   static float best_error;       //�������
   
   
   fore_date = real_angle+Gyro*dt;                                       //Ԥ��ֵ=��һ������ֵ�������ǻ���
   fore_noise_date = sqrt(Q_Gyro*Q_Gyro+best_error*best_error);          //Ԥ��ֵ����ƫ��
   noise_square = sqrt((fore_noise_date*fore_noise_date)/(fore_noise_date*fore_noise_date+Q_Acc*Q_Acc));         //����������
   real_angle = fore_date+noise_square*(Accel-fore_date);                //���ŽǶ�=Ԥ��Ƕ�+����������*������ֵ-Ԥ��ֵ��
   best_error = sqrt((1-noise_square)*fore_noise_date*fore_noise_date);  //�����������
   
   nf_Angle = real_angle;
     
   return real_angle;
   
}



