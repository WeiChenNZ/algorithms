#include <matplotlibcpp/matplotlibcpp.h>
#include <iostream>
#include <vector>
#include "KalmanFilter.h"
#include "motionModel.h"
#include <memory>
#include <math.h>
#include <random>

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

int main()
{
    double dt = 0.1;
    double t = 20;
    double timeSteps = t/dt;

    //random numbers generator
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<> dist(0.0, 1.0);

    //define a motion model
    unique_ptr<ModelInterface> model = make_unique<MotionModel>(dt);

    //Kalman Filter
    KalmanFilter kf(move(model));

    //initial data
    MatrixXd X0(6,1);
    X0 << 0,0,0,0,0,0;

    MatrixXd E0 = 100 * MatrixXd::Identity(6,6);

    //start processing
    kf.setInitialState(X0, E0);

    MatrixXd result = MatrixXd::Zero(6,timeSteps);
    MatrixXd groundTruth = MatrixXd::Zero(3,timeSteps);
    MatrixXd trueState = MatrixXd::Zero(6,1);
    MatrixXd Y = MatrixXd::Zero(3,timeSteps);
    MatrixXd time = MatrixXd::Zero(1, timeSteps);

    for(int i = 0; i < int(timeSteps); i++)
    {
        //generate control u and observation y
        MatrixXd u(3,1);
        u << sin(i*dt), cos(i*dt), -sin(i*dt);

        //generate ground truth 
        trueState.block(3,0,3,1) += u*dt;        //[3:6, 0]
        trueState.block(0,0,3,1) += trueState.block(3,0,3,1)*dt + 0.5*u*dt*dt;
        groundTruth.col(i) = trueState.block(0,0,3,1);

        //observation
        Y.col(i) << dist(gen)*sqrt(5), dist(gen)*sqrt(5), dist(gen)*sqrt(5);
        Y.col(i) += groundTruth.col(i);
        
        //run Kalman Filter
        kf.iteration(u, Y.col(i));

        result.col(i) = kf.getLastState();
        time(i) = i;
    }

    //visualization
    vector<double> x0,x1,x2, z0,z1,z2, time_x;
    for(int i = 0; i < int(timeSteps); i++)
    {
        x0.push_back(result(0,i));
        x1.push_back(result(1,i));
        x2.push_back(result(2,i));
        z0.push_back(groundTruth(0,i));
        z1.push_back(groundTruth(1,i));
        z2.push_back(groundTruth(2,i));
        time_x.push_back(i);
    }

    plt::suptitle("Kalman Filter");
    const long nrows = 3, ncols = 1;
    long row = 0, col = 0;
    plt::subplot2grid(nrows,ncols,row,col);
    plt::plot(time_x, z0, "b-");
    plt::plot(time_x, x0, "r-");
    row = 1, col = 0;
    plt::subplot2grid(nrows,ncols,row,col);
    plt::plot(time_x, z1, "b-");
    plt::plot(time_x, x1, "r-");
    row = 2, col = 0;
    plt::subplot2grid(nrows,ncols,row,col);
    plt::plot(time_x, z2, "b-");
    plt::plot(time_x, x2, "r-");
    plt::show();

    return 0;
}