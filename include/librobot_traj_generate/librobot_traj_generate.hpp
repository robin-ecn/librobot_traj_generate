#ifndef librobot_trajectory_generation_H
#define librobot_trajectory_generation_H
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

class LibTrajGenerate

{
    private:
    Eigen::VectorXd q0_;
    Eigen::VectorXd qf_;

    Eigen::VectorXd distance_;
    
    double r_;
    double dr_;
    double ddr_;

    double travelling_time_;

    public:
    
    LibTrajGenerate(const Eigen::VectorXd q0, const Eigen::VectorXd qf, double tf);

    void doFifthDegreeTrajectory(double time_step);

    void getTrajectory(Eigen::VectorXd &q, Eigen::VectorXd &dq, Eigen::VectorXd &ddq);
    
    ~LibTrajGenerate();


};






#endif
