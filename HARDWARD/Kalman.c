#include "Kalman.h"
/* second-order kalman filter on stm32  二维卡尔曼滤波*/
//#include "arm_math.h"

//#define mat         arm_matrix_instance_f32 
//#define mat_init    arm_mat_init_f32                       //矩阵初始化
//#define mat_add     arm_mat_add_f32                        //矩阵加法
//#define mat_sub     arm_mat_sub_f32                        //矩阵减法
//#define mat_mult    arm_mat_mult_f32                       //矩阵相乘
//#define mat_trans   arm_mat_trans_f32                      //矩阵转置
//#define mat_inv     arm_mat_inverse_f32                    //矩阵求逆

//typedef struct
//{
//  float raw_value;
//  float filtered_value[2];
//  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
//} kalman_filter_t;

//typedef struct
//{
//  float raw_value;
//  float filtered_value[2];
//  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
//  float P_data[4];
//  float AT_data[4], HT_data[4];
//  float A_data[4];
//  float H_data[4];
//  float Q_data[4];
//  float R_data[4];
//} kalman_filter_init_t;

//void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
//{
//  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
//  mat_init(&F->HT,2,2,(float *)I->HT_data);
//  mat_trans(&F->H, &F->HT);
//}

//float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
//{
//  float TEMP_data[4] = {0, 0, 0, 0};                  
//  float TEMP_data21[2] = {0, 0};
//  mat TEMP,TEMP21;

//  mat_init(&TEMP,2,2,(float *)TEMP_data);                       //把数组内数据变为矩阵
//  mat_init(&TEMP21,2,1,(float *)TEMP_data21);

//  F->z.pData[0] = signal1;
//  F->z.pData[1] = signal2;

//  //1. xhat'(k)= A xhat(k-1)
//  mat_mult(&F->A, &F->xhat, &F->xhatminus);       

//  //2. P'(k) = A P(k-1) AT + Q
//  mat_mult(&F->A, &F->P, &F->Pminus);
//  mat_mult(&F->Pminus, &F->AT, &TEMP);
//  mat_add(&TEMP, &F->Q, &F->Pminus);

//  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
//  mat_mult(&F->H, &F->Pminus, &F->K);
//  mat_mult(&F->K, &F->HT, &TEMP);
//  mat_add(&TEMP, &F->R, &F->K);

//  mat_inv(&F->K, &F->P);
//  mat_mult(&F->Pminus, &F->HT, &TEMP);
//  mat_mult(&TEMP, &F->P, &F->K);

//  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
//  mat_mult(&F->H, &F->xhatminus, &TEMP21);
//  mat_sub(&F->z, &TEMP21, &F->xhat);
//  mat_mult(&F->K, &F->xhat, &TEMP21);
//  mat_add(&F->xhatminus, &TEMP21, &F->xhat);

//  //5. P(k) = (1-K(k)H)P'(k)
//  mat_mult(&F->K, &F->H, &F->P);
//  mat_sub(&F->Q, &F->P, &TEMP);
//  mat_mult(&TEMP, &F->Pminus, &F->P);

//  F->filtered_value[0] = F->xhat.pData[0];
//  F->filtered_value[1] = F->xhat.pData[1];

//  return F->filtered_value;
//}

/** *****************************************************************************************
  * @file    kalman.c                                                                                                                                      
  * @author  Liu heng                                                                                                                                   
  * @version V1.0.0                                                                                                                                    
  * @date    27-August-2013                                                                                                                        
  * @brief   一维卡尔曼滤波器的具体实现。实现过程完全与硬件无关，   *
  *   可直接调用，任意移植。
  *   使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器?*
  *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波。               *
  *****************************************************************************************
  *                          使用示例                                                     *
  *          kalman p;                                                                   *
  *          float SersorData;                                                            *
  *          kalmanCreate(&p,20,200);                                                  *
  *          while(1)                                                                     *
  *          {                                                                            *
  *             SersorData = sersor();                                                    *
  *             SersorData = KalmanFilter(&p,SersorData);                                  *
  *             printf("%2.2f",SersorData);                                               *
  *          }                                                                            *
  *****************************************************************************************
  *          MPU6050的卡尔曼滤波器参考参数 Q：10 R：400                                   *
  *****************************************************************************************/

#include "kalman.h"

/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *         
  * @retval none
  */
void kalmanCreate(kalman *p,float T_Q,float T_R)
{
    //kalman* p = ( kalman*)malloc(sizeof( kalman));
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->H = 1;
    p->X_mid = p->X_last;
    //return p;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  */

float KalmanFilter(kalman* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)                     获得下一状态的预测值
    p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q                       更新方差
    p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)                计算增益
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))          获得这一时刻的最优值
    p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)                      获得这一时刻的最优偏差值                 
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;
}

