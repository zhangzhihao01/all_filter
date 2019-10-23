

#ifndef  e_max
#define  e_max   10
#define  e_min   (-10)
#define  E_NB    (e_min)
#define  E_NM    (e_min*2/3.0)
#define  E_NS    (e_min/3.0)
#define  E_ZO    0
#define  E_PS    (e_max/3.0)
#define  E_PM    (e_max*2/3.0)
#define  E_PB    (e_max)
#endif

#ifndef  ec_max
#define  ec_max   0.5
#define  ec_min   (-0.5)
#define  Ec_NB    (ec_min)
#define  Ec_NM    (ec_min*2/3.0)
#define  Ec_NS    (ec_min/3.0)
#define  Ec_ZO    0
#define  Ec_PS    (ec_max/3.0)
#define  Ec_PM    (ec_max*2/3.0)
#define  Ec_PB    (ec_max)
#endif

#ifndef  Kp_max
#define  Kp_max   100
#define  Kp_min   -100
#define  Kp_ZO           ((Kp_max+Kp_min)*0.5)
#define  Kp_longth       ((Kp_max-Kp_min)*0.5)
#define  Kp_NB           (Kp_min)
#define  Kp_NM           (Kp_ZO-Kp_longth*2/3.0)
#define  Kp_NS           (Kp_ZO-Kp_longth/3.0)
#define  Kp_PS           (Kp_ZO+Kp_longth/3.0)
#define  Kp_PM           (Kp_ZO+Kp_longth*2/3.0)   
#define  Kp_PB           (Kp_max)    
#endif

#ifndef  Ki_max
#define  Ki_max   1
#define  Ki_min   -1
#define  Ki_ZO           ((Ki_max+Ki_min)*0.5)
#define  Ki_longth       ((Ki_max-Ki_min)*0.5)
#define  Ki_NB           (Ki_min)
#define  Ki_NM           (Ki_ZO-Ki_longth*2/3.0)
#define  Ki_NS           (Ki_ZO-Ki_longth/3.0)
#define  Ki_PS           (Ki_ZO+Ki_longth/3.0)
#define  Ki_PM           (Ki_ZO+Ki_longth*2/3.0)   
#define  Ki_PB           (Ki_max)
#endif

#ifndef  Kd_max
#define  Kd_max   10
#define  Kd_min   -10
#define  Kd_ZO           ((Kd_max+Kd_min)*0.5)
#define  Kd_longth       ((Kd_max-Kd_min)*0.5)
#define  Kd_NB           (Kd_min)
#define  Kd_NM           (Kd_ZO-Kd_longth*2/3.0)
#define  Kd_NS           (Kd_ZO-Kd_longth/3.0)
#define  Kd_PS           (Kd_ZO+Kd_longth/3.0)
#define  Kd_PM           (Kd_ZO+Kd_longth*2/3.0)   
#define  Kd_PB           (Kd_max)
#endif




enum ty_pid
{
  ty_p,
  ty_i,
  ty_d
};


void E_fuzzificition(float e);
void Ec_fuzzificition(float ec);
float  get_pid(enum ty_pid pid);


