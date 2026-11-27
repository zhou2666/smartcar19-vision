
/*******************************************************************************************************************/
/***Include***/
//{
#include "math.h"
#include "ant.h"
#include "stdio.h"
#include "zf_common_headfile.h"
#include "define.h"  
#include "bsp.h"
//}
/*******************************************************************************************************************/
double ABS_ant(double n){
  if(n>0||n==0)
    return n;
  else
    return -n;
}
void sort(double *p,int lenth){
    for(int i = 0;i<lenth;i++)
        for(int j=0;j<lenth-1;j++)
        {
            double t = p[j];
            if(p[j]>p[j+1])
            {
                p[j]=p[j+1];
                p[j+1]=t;
            }
        }

}

int get_index(double *f,int lenth,double v){
    int t=-1;
    for(int i=0;i<lenth;i++)
    {
        if(ABS_ant(f[i]-v)<0.000001)
            t=i;
    }
    return t;
}
int con_bet(double *p,int lenth){
    double temp[60];
    for(int i=0;i<lenth;i++)
      temp[i]=0.0;
    int ind = -1;
    for (int i=0;i<lenth;i++){
      if(p[i]== 1.0){
        return i;
      }
      temp[i] = p[i];
    }
    sort(temp,lenth);
    ///srand((unsigned)time(NULL));
    double a = rand()%100/100.0;
    for(int i=1;i<lenth;i++)
        temp[i] += temp[i-1];
    for(int i=0;i<lenth-1;i++){
        if(i==lenth-2){
            if (a>temp[i] && a<1.0000){
                double output = temp[i+1]-temp[i];
                ind = get_index(p,lenth,output);
                return ind;


            }
        }
        else{
            if(a>temp[i] && a<temp[i+1])
            {
                double output = temp[i+1]-temp[i];
                ind = get_index(p,lenth,output);
                return ind;

            }
        }
    }
    return 0;
}
int count(int *f,int a,int lenth){
      int coun=0;
    for(int i=0;i<lenth;i++)
        if (f[i]==a){
            coun++;
        }
    return coun;

}
double sum(double *f,int lenth){
    double len = 0;
    for(int i=0;i<lenth;i++)
        len += f[i];
    return len;
}
 



typedef struct {
  int path[30];
int lenth;
double len_path;
}Ant;
Ant ant[50];
double delta[30][30];
double dis_point[30][30];  
int ant_count=15;
int ant_flag[30]={0};
double min_distance = 999.0;
int best[30];
void ant_start(uint8 *point_data,int len_point_data)
{


   len_point_data=len_point_data/2;
      //int point_data[][2] = {{0,0},{23, 22}, {12, 21}, {28, 21}, {2, 18}, {21, 17}, {16, 16}, {6, 15}, {11, 13}, {7, 10}, {29, 10}, {21, 7}, {4, 6}, {9, 6}, {13, 6}, {17, 5}, {31, 5}, {23, 3}, {6, 2},  {30, 1},{12, 2}};
    //定义参数
    int aa=1,b=2;
 
    float r=0.5;
    //初始化信息素矩阵
    
    
    for(int i=0;i<len_point_data;i++)
        for(int j=0;j<len_point_data;j++)
        {
            if(i==j)
                delta[i][j]=0;
            else
                delta[i][j]=0.3;
        }
    //初始化距离矩阵
    
    for(int i=0;i<len_point_data;i++) 
    {
        for (int j = 0; j < len_point_data; j++) 
        {
            dis_point[i][j] = pow(
                    (pow((point_data[2*i] - point_data[2*j]), 2) + pow((point_data[2*i+1] - point_data[2*j+1]), 2)), 0.5);
            //dis_point[i][j] = pow(
                    //(pow((point_data[i][0] - point_data[j][0]), 2) + pow((point_data[i][1] - point_data[j][1]), 2)), 0.5);
        }
    }

 
    //开始迭代
    int iteration = 30;
    for(int ss = 0;ss<iteration;ss++) {
        //初始化蚂蚁的出发点

 //srand((unsigned) ss);
        for(int k=0;k<ant_count;k++){
          //ant[k].path[0] = 0;
            ant[k].lenth = 1;
            ant[k].len_path = 0;
            ant_flag[k]=0;
        }
        for(int k=0;k<ant_count;k++){
          for(int j=1;j<len_point_data;j++)
            
                ant[k].path[j] = -1;
        }
//        printf("%lf\n",ant[2].len_path);

        //while循环直至所有蚂蚁走完所有点
        
        double prob[50];
        while (count(ant_flag, 1, ant_count) != ant_count) {
            //初始化完成标志
            for (int k = 0; k < ant_count; k++) {
                //结束标志
                if (ant[k].lenth == len_point_data) {
                    ant_flag[k] = 1;
                }
                //初始化去各个点的概率
                 
                for (int i = 0; i < len_point_data; i++) {
                    if (count(ant[k].path, i, len_point_data+1))
                        prob[i] = 0;
                    else
                     prob[i] = pow((delta[ant[k].path[ant[k].lenth - 1]][i]), aa) * pow(1/dis_point[ant[k].path[ant[k].lenth - 1]][i], b);
                        
                }
                //归一化
                double su = sum(prob, len_point_data);
                for (int i = 0; i < len_point_data; i++)
                    prob[i] /= su;
                if(ant[k].lenth == len_point_data)
                    ant[k].path[ant[k].lenth] = 0;
                else{
                     int t = con_bet(prob, len_point_data);//轮盘赌法去下一个点
                     while(count(ant[k].path,t,ant[k].lenth))
                       t = con_bet(prob, len_point_data);
                     ant[k].path[ant[k].lenth] = t;
                }
                   
                //叠加距离
                ant[k].len_path += dis_point[ant[k].path[ant[k].lenth-1]][ant[k].path[ant[k].lenth]];
                ant[k].lenth = ant[k].lenth + 1;

            }
         }
        //更新信息素矩阵
        for (int i = 0; i < len_point_data; i++)
            for (int j = 0; j < len_point_data; j++) {
                delta[i][j] = (1 - r) * delta[i][j];
            }

        for (int i = 0; i < ant_count; i++)
            for (int j = 0; j < len_point_data - 1; j++) {
                delta[ant[i].path[j]][ant[i].path[j + 1]] += 1 / ant[i].len_path;
            }
        //选择上次最优路线
        double min_dis = 999.0;
        int min_ind = -1;
        for (int j = 0; j < ant_count; j++) {
            if (min_dis > ant[j].len_path) {
                min_dis = ant[j].len_path;
                min_ind = j;
                
            }
        }
        if(min_distance>min_dis){
        for(int m=0;m<len_point_data+1;m++)
          best[m] = ant[min_ind].path[m];
        min_distance=min_dis;
        }
    

    }
    for(int i=0;i<len_point_data;i++)
   // PRINTF("[%f,%f],",openart1.picture_array[2*i]*0.2, openart1.picture_array[2*i+1]*0.2);
    PRINTF("\n");
    for(int i=0;i<len_point_data+1;i++)
     // PRINTF("%d,",best[i]);
    PRINTF("\n");
    for(int k=0;k<len_point_data+1;k++)
    {
      if(k<len_point_data)
      {
         temp_openart1[2*k]= point_data[2*best[k]];
         temp_openart1[2*k+1]= point_data[2*best[k]+1];
      }
  else
  {
     temp_openart1[2*k] = 3;
         temp_openart1[2*k+1]= 3;
  }
         //temp_openart1[2*k+1]-=0.5;
//      openart1.picture_array[2*k]= point_data[2*best[k]];
//      openart1.picture_array[2*k+1]= point_data[2*best[k]+1];
        //PRINTF("[%d,%d],",point_data[best[k]][0],point_data[best[k]][1]);
         
         
        PRINTF("[%f,%f],",temp_openart1[2*k] ,temp_openart1[2*k+1] );
      PRINTF("\n");
//      PRINTF("%d,",best[k]);
       // PRINTF("\n[%d,%d],",openart1.picture_array[2*k],openart1.picture_array[2*k+1]);

    }
     //PRINTF("%lf\n",min_distance);
}