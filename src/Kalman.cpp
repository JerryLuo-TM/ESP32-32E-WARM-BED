#include "system_init.h"


double X_last=0; //上一时刻的最优结果
double X_mid=0;  //当前时刻的预测结果
double X_now=0;  //当前时刻的最优结果
double P_mid=0;  //当前时刻预测结果的协方差
double P_now=0;  //当前时刻最优结果的协方差
double P_last=0; //上一时刻最优结果的协方差
double kg=0;     //kalman增益
double Kalman_Q=1.0;
double R=0;

double KalmanFilter(double dat)
{
	static double Last_dat;
	double i;
	i = fabsf(Last_dat - dat);
	if(i > 3.0f) {
		i = 3.0f;
		R = 10.0f;
	} else {
		R = 10.0f + fabsf(i - 3.0f) * 1000.0f;
	}
    X_mid = X_last;                //x(k|k-1) = AX(k-1|k-1)+BU(k)
    P_mid = P_last+Kalman_Q;       //p(k|k-1) = Ap(k-1|k-1)A'+Q
    kg = P_mid/(P_mid+R);          //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    X_now=X_mid+kg*(dat-X_mid);    //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    P_now=(1.0f-kg)*P_mid;         //p(k|k) = (I-kg(k)H)P(k|k-1)
    P_last=P_now;                  //状态更新
    X_last=X_now;
	Last_dat=X_now;
    return X_now;
}

