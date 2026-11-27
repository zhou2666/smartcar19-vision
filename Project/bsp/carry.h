/*******************************************************************************************************************/
#ifndef __CARRY_H
#define __CARRY_H
/*******************************************************************************************************************/
/***Includes***/
//{
//#include None
//}
/*******************************************************************************************************************/
/***Export Functions***/
//{
//void steering_bullt_A(float degree);
//void steering_bullt_B(int mode,float degree);
//void Electromagnet(uint8 mode);
//void carry_function(int type);
//void carry_back();
//void unload(int type);
//void carrry_picture(uint8 type);
//void remove_picture(uint8 calssify);
void servo_back ();
void servo_work();
void servo_init();
void PWM2_ctrl(uint16 _servo2_angle , float _step_count);
void PWM1_ctrl(uint16 _servo1_angle , float _step_count);
void PWM3_ctrl(uint16 _servo2_angle , float _step_count);
void PWM4_ctrl(uint16 _servo4_angle , float _step_count);
void PWM5_ctrl(uint16 _servo4_angle , float _step_count);

void test_sq();
void sq();
void sq_l();
void sq_r();
void ts();
void fz();
void yb();
void dcs_init(void);
void dcs_work(uint8 n);
void Send_my_element_picture_message();
void steering_degree_in(int small_degree,int big_degree);
void species_scattered(int small_species);
void species_concentrated(int species_p);
void place_picture(int alphabet_count);

void steering_dir(int last, int now);
//}
/*******************************************************************************************************************/
#endif