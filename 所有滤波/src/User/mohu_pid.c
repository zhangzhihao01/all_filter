#include "include.h" 

//横为e,纵为ec
const float rule_Kp[7][7]=
{
  /*NB    NM    NS    ZO    PS    PM    PB*/ //E 
                                                //Ec
  {Kp_PB,Kp_PB,Kp_PM,Kp_PM,Kp_PS,Kp_ZO,Kp_ZO,}, //NB
  {Kp_PB,Kp_PB,Kp_PM,Kp_PS,Kp_PS,Kp_ZO,Kp_NS,}, //NM
  {Kp_PM,Kp_PM,Kp_PM,Kp_PS,Kp_ZO,Kp_NS,Kp_NS,}, //NS
  {Kp_PM,Kp_PM,Kp_PS,Kp_ZO,Kp_NS,Kp_NM,Kp_NM,}, //ZO
  {Kp_PS,Kp_PS,Kp_ZO,Kp_NS,Kp_NS,Kp_NM,Kp_NM,}, //PS
  {Kp_PS,Kp_ZO,Kp_NS,Kp_NM,Kp_NM,Kp_NM,Kp_NB,}, //PM
  {Kp_ZO,Kp_ZO,Kp_NM,Kp_NM,Kp_NM,Kp_NB,Kp_NB,}  //PB
  
};
const float rule_Ki[7][7]=
{
  /*NB    NM    NS    ZO    PS    PM    PB*/ //E 
                                                //Ec
  {Ki_NB,Ki_NB,Ki_NM,Ki_NM,Ki_NS,Ki_ZO,Ki_ZO,}, //NB
  {Ki_NB,Ki_NB,Ki_NM,Ki_NS,Ki_NS,Ki_ZO,Ki_ZO,}, //NM
  {Ki_NB,Ki_NM,Ki_NS,Ki_NS,Ki_ZO,Ki_PS,Ki_PS,}, //NS
  {Ki_NM,Ki_NM,Ki_NS,Ki_ZO,Ki_PS,Ki_PM,Ki_PM,}, //ZO
  {Ki_NM,Ki_NS,Ki_ZO,Ki_PS,Ki_PS,Ki_PM,Ki_PB,}, //PS
  {Ki_ZO,Ki_ZO,Ki_PS,Ki_PS,Ki_PM,Ki_PB,Ki_PB,}, //PM
  {Ki_ZO,Ki_ZO,Ki_PS,Ki_PM,Ki_PM,Ki_PB,Ki_PB,}  //PB
};
const float rule_Kd[7][7]=
{
  /*NB    NM    NS    ZO    PS    PM    PB*/ //E 
                                                //Ec
  {Kd_PS,Kd_NS,Kd_NB,Kd_NB,Kd_NB,Kd_NM,Kd_PS,}, //NB
  {Kd_PS,Kd_NS,Kd_NB,Kd_NM,Kd_NM,Kd_NS,Kd_ZO,}, //NM
  {Kd_ZO,Kd_NS,Kd_NM,Kd_NM,Kd_NS,Kd_NS,Kd_ZO,}, //NS
  {Kd_ZO,Kd_NS,Kd_NS,Kd_NS,Kd_NS,Kd_NS,Kd_ZO,}, //ZO
  {Kd_ZO,Kd_ZO,Kd_ZO,Kd_ZO,Kd_ZO,Kd_ZO,Kd_ZO,}, //PS
  {Kd_PB,Kd_NS,Kd_PS,Kd_PS,Kd_PS,Kd_PS,Kd_PB,}, //PM
  {Kd_PB,Kd_PM,Kd_PM,Kd_PM,Kd_PS,Kd_PS,Kd_PB,}  //PB
};

int E_postion[2];
int Ec_postion[2];
float E_lishudu[2];
float E_level[2];
float Ec_lishudu[2];
float Ec_level[2];


/*输入值e模糊化处理*/
void E_fuzzificition(float e)
{
   
   if(e>e_max)
     e=e_max*0.999;
   else if(e<e_min)
     e=e_min*0.999;
   
   
   if(e<E_NM)
   {
       E_level[0] = E_NB;
       E_level[1] = E_NM;
       E_postion[0] = 0;
       E_postion[1] = 1;
   }
   else if(e<E_NS)
   {
       E_level[0] = E_NM;
       E_level[1] = E_NS;
       E_postion[0] = 1;
       E_postion[1] = 2;
   }
   else if(e<E_ZO)
   {
       E_level[0] = E_NS;
       E_level[1] = E_ZO;
       E_postion[0] = 2;
       E_postion[1] = 3;
   }
   else if(e<E_PS)
   {
       E_level[0] = E_ZO;
       E_level[1] = E_PS;
       E_postion[0] = 3;
       E_postion[1] = 4;
   }
   else if(e<E_PM)
   {
       E_level[0] = E_PS;
       E_level[1] = E_PM;
       E_postion[0] = 4;
       E_postion[1] = 5;
   }
   else if(e<E_PB)
   {
       E_level[0] = E_PM;
       E_level[1] = E_PB;
       E_postion[0] = 5;
       E_postion[1] = 6;
   }
   E_lishudu[0] = (e-E_level[0])/(E_level[1]-E_level[0]);
   E_lishudu[1] = 1-E_lishudu[0]; 
}


/*输入值ec模糊化处理*/
void Ec_fuzzificition(float ec)
{
   
   if(ec>ec_max)
     ec=ec_max*0.999;
   else if(ec<ec_min)
     ec=ec_min*0.999;
   
   
   if(ec<Ec_NM)
   {
       Ec_level[0] = Ec_NB;
       Ec_level[1] = Ec_NM;
       Ec_postion[0] = 0;
       Ec_postion[1] = 1;
   }
   else if(ec<Ec_NS)
   {
       Ec_level[0] = Ec_NM;
       Ec_level[1] = Ec_NS;
       Ec_postion[0] = 1;
       Ec_postion[1] = 2;
   }
   else if(ec<Ec_ZO)
   {
       Ec_level[0] = Ec_NS;
       Ec_level[1] = Ec_ZO;
       Ec_postion[0] = 2;
       Ec_postion[1] = 3;
   }
   else if(ec<Ec_PS)
   {
       Ec_level[0] = Ec_ZO;
       Ec_level[1] = Ec_PS;
       Ec_postion[0] = 3;
       Ec_postion[1] = 4;
   }
   else if(ec<Ec_PM)
   {
       Ec_level[0] = Ec_PS;
       Ec_level[1] = Ec_PM;
       Ec_postion[0] = 4;
       Ec_postion[1] = 5;
   }
   else if(ec<Ec_PB)
   {
       Ec_level[0] = Ec_PM;
       Ec_level[1] = Ec_PB;
       Ec_postion[0] = 5;
       Ec_postion[1] = 6;
   }
   Ec_lishudu[0] = (ec-Ec_level[0])/(Ec_level[1]-Ec_level[0]);
   Ec_lishudu[1] = 1-Ec_lishudu[0]; 
}



/* 反模糊，输出pid */
float  get_pid(enum ty_pid pid)
{
  float K;
  
  
  if(pid == ty_p)
  {
    K = rule_Kp[Ec_postion[0]][E_postion[0]]*E_lishudu[0]*Ec_lishudu[0]+
        rule_Kp[Ec_postion[0]][E_postion[1]]*E_lishudu[1]*Ec_lishudu[0]+
        rule_Kp[Ec_postion[1]][E_postion[0]]*E_lishudu[0]*Ec_lishudu[1]+
        rule_Kp[Ec_postion[1]][E_postion[1]]*E_lishudu[1]*Ec_lishudu[1];
  }
  else if(pid == ty_d)
  {
    K = rule_Kd[Ec_postion[0]][E_postion[0]]*E_lishudu[0]*Ec_lishudu[0]+
        rule_Kd[Ec_postion[0]][E_postion[1]]*E_lishudu[1]*Ec_lishudu[0]+
        rule_Kd[Ec_postion[1]][E_postion[0]]*E_lishudu[0]*Ec_lishudu[1]+
        rule_Kd[Ec_postion[1]][E_postion[1]]*E_lishudu[1]*Ec_lishudu[1];
  }
  else if(pid == ty_i)
  {
    K = rule_Ki[Ec_postion[0]][E_postion[0]]*E_lishudu[0]*Ec_lishudu[0]+
        rule_Ki[Ec_postion[0]][E_postion[1]]*E_lishudu[1]*Ec_lishudu[0]+
        rule_Ki[Ec_postion[1]][E_postion[0]]*E_lishudu[0]*Ec_lishudu[1]+
        rule_Ki[Ec_postion[1]][E_postion[1]]*E_lishudu[1]*Ec_lishudu[1];
  }
  return K;
  
}















