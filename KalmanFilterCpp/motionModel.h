#pragma once

#include "KalmanFilter.h"
#include <Eigen/Core>

class MotionModel: public ModelInterface{
    public:
        MotionModel(int t_) : dt(t_){
            formulateModelMatrix();
        };

        void formulateModelMatrix()
        {
            Eigen::MatrixXd A_(6,6);
            A_ << 1,0,0,dt,0,0,
                 0,1,0,0,dt,0,
                 0,0,1,0,0,dt,
                 0,0,0,1,0,0,
                 0,0,0,0,1,0,
                 0,0,0,0,0,1;
            
            Eigen::MatrixXd B_(6,3);
            B_ << 0.5*dt*dt, 0, 0,
                  0, 0.5*dt*dt, 0,
                  0, 0, 0.5*dt*dt,
                  dt, 0, 0,
                  0, dt, 0,
                  0, 0, dt;

            Eigen::MatrixXd C_(3,6);
            C_ << 1, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0,
                  0, 0, 1, 0, 0, 0;
            
            Eigen::MatrixXd W_ = 0.01*Eigen::MatrixXd::Identity(6,6);

            Eigen::MatrixXd V_ = 5*Eigen::MatrixXd::Identity(3,3);

            A = A_;
            B = B_;
            C = C_;
            W = W_;
            V = V_;
        }


        //implement the interface
        Eigen::MatrixXd getModelMatrix_A(){return A;}
        Eigen::MatrixXd getModelMatrix_B(){return B;}
        Eigen::MatrixXd getModelMatrix_C(){return C;}
        Eigen::MatrixXd getModelMatrix_W(){return W;}
        Eigen::MatrixXd getModelMatrix_V(){return V;}

    
    private:
        int dt; //time step
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd C;
        Eigen::MatrixXd W;
        Eigen::MatrixXd V;

};

