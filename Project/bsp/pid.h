/*******************************************************************************************************************/
#ifndef __PID_H
#define __PID_H
/*******************************************************************************************************************/
/***Includes***/
//{
//#include None
//}
/*******************************************************************************************************************/
/***Export Functions***/
//{
void Speed_Planning(float x,float y);
void move(float x,float y,float degree);
void Game_Begin(void);
void my_move(float degree);
void compare(float x,float y);
void my_location_PID_function(void);
void location_PID_function(float x,float y);
void location_PID_function_hard(float y);
void angle_PID_function(float degree);
void gory_PID_function(float gory);
void speed_PID_function(float sx,float sy);
 void test_move(float x,float y,float degree);
 void turn_pid_function();
//}
/*******************************************************************************************************************/
#endif