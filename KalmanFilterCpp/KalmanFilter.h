#pragma once

#include<Eigen/Core>
#include <memory>

class ModelInterface{
    public:
        virtual Eigen::MatrixXd getModelMatrix_A() = 0;  //state matrix
        virtual Eigen::MatrixXd getModelMatrix_B() = 0;  //control matrix
        virtual Eigen::MatrixXd getModelMatrix_C() = 0;  //observation matrix
        virtual Eigen::MatrixXd getModelMatrix_W() = 0;  //control noise cov
        virtual Eigen::MatrixXd getModelMatrix_V() = 0;  //obervation noise cov

};


class KalmanFilter{

    public:
        KalmanFilter(std::unique_ptr<ModelInterface> model_): model(std::move(model_)){
            A = model->getModelMatrix_A();
            B = model->getModelMatrix_B();
            C = model->getModelMatrix_C();
            W = model->getModelMatrix_W();
            V = model->getModelMatrix_V();
            iterationCount = 0;
        }; 

        void setInitialState(Eigen::MatrixXd x0_, Eigen::MatrixXd E0_)
        {
            X0 = x0_;
            E0 = E0_;
            iterationCount = 0;
        }

        Eigen::MatrixXd getLastState()
        {
            return Xn;
        }
        
        //kalman filter's main process, include predict, update
        void iteration(Eigen::MatrixXd, Eigen::MatrixXd);


    private:
        //model
        std::unique_ptr<ModelInterface> model;
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd C;
        Eigen::MatrixXd W;
        Eigen::MatrixXd V;

        //initial value
        Eigen::MatrixXd X0;
        Eigen::MatrixXd E0;

        //result
        Eigen::MatrixXd Xn;
        Eigen::MatrixXd En;

        int iterationCount;
};