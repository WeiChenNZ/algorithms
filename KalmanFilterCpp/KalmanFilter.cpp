#include "KalmanFilter.h"
#include <Eigen/Dense>

void KalmanFilter::iteration(Eigen::MatrixXd Un, Eigen::MatrixXd Yn1)
{
    Eigen::MatrixXd Xn1_p; 
    Eigen::MatrixXd En1_p; 
    Eigen::MatrixXd Zn1;
    Eigen::MatrixXd Sn1;
    Eigen::MatrixXd Ln1;

    //step1: Predict
    if(iterationCount == 0)
    {
        Xn1_p = A*X0 + B*Un;
        En1_p = A*E0*A.transpose() + W;
    }
    else
    {
        Xn1_p = A*Xn + B*Un;
        En1_p = A*En*A.transpose() + W; 
    }
    iterationCount++;

    //innovation and its cov
    Zn1 = Yn1 - C*Xn1_p;
    Sn1 = C*En1_p*C.transpose() + V;

    //Kalman Gain
    Ln1 = En1_p*C.transpose()*Sn1.inverse();

    //step2: Update
    //in math this should be Xn1 and En1
    //but in order to do iteration, I store the result into Xn, En
    Xn = Xn1_p + Ln1 * Zn1;
    int stateDim = Xn.rows();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateDim, stateDim);
    En = (I - Ln1*C)*En1_p*(I - Ln1*C).transpose() + Ln1*V*Ln1.transpose();
}