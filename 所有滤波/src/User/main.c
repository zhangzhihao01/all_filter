/****************************************************************************************************
【平    台】龙邱K66FX智能车VC母板
【编    写】CHIUSIR
【E-mail  】chiusir@aliyun.com
【软件版本】V1.0
【最后更新】2016年08月24日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR7.3
【Target  】K66FX1M0VLQ18
【Crystal 】 50.000Mhz
【busclock】 40.000MHz
【pllclock】200.000MHz
=============================================================
测试GPIO口外部中断
-------------------------
外部IO口中断的实现过程如下：
1.初始化GPIO口为输入（GPIO.C中）；
2.开启中断口的允许位（GPIO.C中）；
3.开启总中断；
4.向量表中声明中断函数名称（vectors.h中）；
5.完成中断函数原型（main.C中,位置不限）；

现象：
下载程序到单片机，然后把需要中断的IO口拉低一次就触发一次外部中断，此时调用外部中断函数。
本例程中，每按一次开发平台的按键，就会触发一次对应LED的点亮。
-------------------------
LED测试GPIO
LED1--PTA17
LED2--PTC0
LED3--PTD15 
LED4--PTE26

KEY测试GPIO
KEY0--PTB20
KEY1--PTB21
KEY2--PTB22
=============================================================
修改历史：
******************************************************************************************************/

#include "include.h" 
#include "SEEKFREE_IIC.h"
#include "math.h"

extern int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
extern int16 icm_acc_x,icm_acc_y,icm_acc_z;
extern float Q_ANGLE_Yaw,Q_ANGLE_Y,Q_ANGLE_X;                         //四元解算三周欧拉角
extern float Angle;                                                  //矩阵卡尔曼滤波得俯仰角
extern float GYRO_Integration;                                       //互补滤波角度
extern float nf_Angle;                                               //非矩阵卡尔曼滤波得俯仰角
short  T_tim=1;                                    //中断周期
short  Fcut_low=20000;                             //低通滤波截至频率
short  Fcut_hight=10;                              //高通滤波截至频率
float  R_gyio_x,R_gyio_y,R_gyio_z;                 //滤波后角速度
float  R_acc_x,R_acc_y,R_acc_z;                    //滤波后加速度
float  Acc_Angle;                                  //加速度计所得偏角           
int    send_data[4];

int    standpwm;
float stand_angle = -39.0; 



/*******************
初始化各个通道PWM
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
    
    int Limit=3800;                             //pwm限幅//※重要,可改※
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

  
  PIT_Flag_Clear(PIT0);       //清中断标志位
  get_icm20602_accdata();     //获取加速度数据
  get_icm20602_gyro();        //获取陀螺仪数据
  
  R_acc_x = low_RC_cut(icm_acc_x,Fcut_low)/16384.0;
  R_acc_y = low_RC_cut(icm_acc_y,Fcut_low)/16384.0;
  R_acc_z = low_RC_cut(icm_acc_z,Fcut_low)/16384.0;
  R_gyio_x = hight_RC_cut(icm_gyro_x,Fcut_hight)/16.4;
  R_gyio_y = hight_RC_cut(icm_gyro_y,Fcut_hight)/16.4;
  R_gyio_z = hight_RC_cut(icm_gyro_z,Fcut_hight)/16.4;
 
  Acc_Angle = (atan2(R_acc_x , R_acc_z))*180/pi;
   
// IMUupdate(R_gyio_x, R_gyio_y, R_gyio_z, R_acc_x , R_acc_y , R_acc_z);        //四元法求偏角
// Complement_Filter(Acc_Angle,R_gyio_y);                                       //互补滤波求偏角
  Kalman_Filter(Acc_Angle,R_gyio_y);                                           //卡尔曼滤波求偏角
//  N_Kalman_Filter(Acc_Angle,R_gyio_y);                                         //非矩阵卡尔曼滤波求偏角
  
  standpwm = stand_pwm(Angle);
  
  Set_Pwm(-standpwm,-standpwm);
  
  send_data[0]=GYRO_Integration;
  send_data[1]=Angle;
  send_data[2]=GYRO_Integration;
  send_data[3]=Acc_Angle;
} 



void main(void)
{
	
  
   DisableInterrupts;        //关闭总中断
   PLL_Init(PLL200);         //初始化PLL为200M，总线为40MHZ
   PIT_Init(PIT0,T_tim);         //定时器中断1ms
   Motor_Init();
   IIC_init();         //模拟IIC 初始化
   icm20602_init();    //六轴陀螺仪初始化
   UART_Init(UART_4,115200);      //蓝牙串口初始化
   UART_Irq_En(UART_4);           //蓝牙串口中断
   
   EnableInterrupts;  
     
     

   while(1)
   { 
		
     Data_Send(UART_4,send_data);
     
   }
}

