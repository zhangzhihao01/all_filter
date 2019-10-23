/****************************************************************************************************
��ƽ    ̨������K66FX���ܳ�VCĸ��
����    д��CHIUSIR
��E-mail  ��chiusir@aliyun.com
������汾��V1.0
�������¡�2016��08��24��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR7.3
��Target  ��K66FX1M0VLQ18
��Crystal �� 50.000Mhz
��busclock�� 40.000MHz
��pllclock��200.000MHz
=============================================================
����GPIO���ⲿ�ж�
-------------------------
�ⲿIO���жϵ�ʵ�ֹ������£�
1.��ʼ��GPIO��Ϊ���루GPIO.C�У���
2.�����жϿڵ�����λ��GPIO.C�У���
3.�������жϣ�
4.�������������жϺ������ƣ�vectors.h�У���
5.����жϺ���ԭ�ͣ�main.C��,λ�ò��ޣ���

����
���س��򵽵�Ƭ����Ȼ�����Ҫ�жϵ�IO������һ�ξʹ���һ���ⲿ�жϣ���ʱ�����ⲿ�жϺ�����
�������У�ÿ��һ�ο���ƽ̨�İ������ͻᴥ��һ�ζ�ӦLED�ĵ�����
-------------------------
LED����GPIO
LED1--PTA17
LED2--PTC0
LED3--PTD15 
LED4--PTE26

KEY����GPIO
KEY0--PTB20
KEY1--PTB21
KEY2--PTB22
=============================================================
�޸���ʷ��
******************************************************************************************************/

#include "include.h" 
#include "SEEKFREE_IIC.h"
#include "math.h"

extern int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
extern int16 icm_acc_x,icm_acc_y,icm_acc_z;
extern float Q_ANGLE_Yaw,Q_ANGLE_Y,Q_ANGLE_X;                         //��Ԫ��������ŷ����
extern float Angle;                                                  //���󿨶����˲��ø�����
extern float GYRO_Integration;                                       //�����˲��Ƕ�
extern float nf_Angle;                                               //�Ǿ��󿨶����˲��ø�����
short  T_tim=1;                                    //�ж�����
short  Fcut_low=20000;                             //��ͨ�˲�����Ƶ��
short  Fcut_hight=10;                              //��ͨ�˲�����Ƶ��
float  R_gyio_x,R_gyio_y,R_gyio_z;                 //�˲�����ٶ�
float  R_acc_x,R_acc_y,R_acc_z;                    //�˲�����ٶ�
float  Acc_Angle;                                  //���ٶȼ�����ƫ��           
int    send_data[4];

int    standpwm;
float stand_angle = -39.0; 



/*******************
��ʼ������ͨ��PWM
********************/
void Motor_Init(void)
{
    
    FTM_PWM_Init(FTM3,FTM_CH0,5000,0);//Mot0-PTC1
    FTM_PWM_Init(FTM0,FTM_CH2,5000,0);//Mot1-PTC2 
    FTM_PWM_Init(FTM0,FTM_CH3,5000,0);//Mot0-PTC3
    FTM_PWM_Init(FTM0,FTM_CH4,5000,0);//Mot1-PTC4 
}


void Set_Pwm(int MotorLeft,int MotorRight)                           
{
    
    int Limit=3800;                             //pwm�޷�//����Ҫ,�ɸġ�
    int dead=0;                                         //<30?
    MotorLeft*=1;
    MotorRight*=1;
    if(MotorLeft>Limit) MotorLeft=Limit;
    if(MotorLeft<-Limit)  MotorLeft=-Limit;	
    if(MotorRight>Limit)   MotorRight=Limit;
    if(MotorRight<-Limit)  MotorRight=-Limit;	
    
    
    if(MotorLeft<=0)
    {
        FTM_PWM_Duty(FTM3, FTM_CH0, 0);      //PTA4
        FTM_PWM_Duty(FTM0, FTM_CH2,-(MotorLeft+dead));             //PTA5
    }
    else
    {

        FTM_PWM_Duty(FTM3, FTM_CH0,MotorLeft+dead);             //PTA4      
        FTM_PWM_Duty(FTM0, FTM_CH2, 0 );     //PTA5
    }
    
    if(MotorRight<=0)
    {
        FTM_PWM_Duty(FTM0, FTM_CH3, -(MotorRight+dead));           //PTA6
        FTM_PWM_Duty(FTM0, FTM_CH4, 0);  //PTA7
    }
    else
    {
        
     
        FTM_PWM_Duty(FTM0, FTM_CH3,0);
        FTM_PWM_Duty(FTM0, FTM_CH4, MotorRight+dead );
    }
}


int stand_pwm(float angle)
{
  static float error,error_ago;
  static float integration;
  float error_c;
  float abs_error;
  float k_p,k_i,k_d;
  int   pwm;
  
  error_ago = error;
  error = angle - stand_angle ;
  error_c = error-error_ago;
  abs_error = fabs(error);
  integration += error;
  if(integration>600)
    integration=600;
  else if(integration<-600)
    integration=-600;
  E_fuzzificition(error);
  Ec_fuzzificition(error_c);
  k_p = get_pid(ty_p);
  k_i = get_pid(ty_i);
  k_d = get_pid(ty_d);  
  
  pwm = k_p*abs_error + fabs(k_i)*integration + fabs(k_d)*error_c;
  
  return pwm;
}

void PIT0_Interrupt()
{

  
  PIT_Flag_Clear(PIT0);       //���жϱ�־λ
  get_icm20602_accdata();     //��ȡ���ٶ�����
  get_icm20602_gyro();        //��ȡ����������
  
  R_acc_x = low_RC_cut(icm_acc_x,Fcut_low)/16384.0;
  R_acc_y = low_RC_cut(icm_acc_y,Fcut_low)/16384.0;
  R_acc_z = low_RC_cut(icm_acc_z,Fcut_low)/16384.0;
  R_gyio_x = hight_RC_cut(icm_gyro_x,Fcut_hight)/16.4;
  R_gyio_y = hight_RC_cut(icm_gyro_y,Fcut_hight)/16.4;
  R_gyio_z = hight_RC_cut(icm_gyro_z,Fcut_hight)/16.4;
 
  Acc_Angle = (atan2(R_acc_x , R_acc_z))*180/pi;
   
// IMUupdate(R_gyio_x, R_gyio_y, R_gyio_z, R_acc_x , R_acc_y , R_acc_z);        //��Ԫ����ƫ��
// Complement_Filter(Acc_Angle,R_gyio_y);                                       //�����˲���ƫ��
  Kalman_Filter(Acc_Angle,R_gyio_y);                                           //�������˲���ƫ��
//  N_Kalman_Filter(Acc_Angle,R_gyio_y);                                         //�Ǿ��󿨶����˲���ƫ��
  
  standpwm = stand_pwm(Angle);
  
  Set_Pwm(-standpwm,-standpwm);
  
  send_data[0]=GYRO_Integration;
  send_data[1]=Angle;
  send_data[2]=GYRO_Integration;
  send_data[3]=Acc_Angle;
} 



void main(void)
{
	
  
   DisableInterrupts;        //�ر����ж�
   PLL_Init(PLL200);         //��ʼ��PLLΪ200M������Ϊ40MHZ
   PIT_Init(PIT0,T_tim);         //��ʱ���ж�1ms
   Motor_Init();
   IIC_init();         //ģ��IIC ��ʼ��
   icm20602_init();    //���������ǳ�ʼ��
   UART_Init(UART_4,115200);      //�������ڳ�ʼ��
   UART_Irq_En(UART_4);           //���������ж�
   
   EnableInterrupts;  
     
     

   while(1)
   { 
		
     Data_Send(UART_4,send_data);
     
   }
}

