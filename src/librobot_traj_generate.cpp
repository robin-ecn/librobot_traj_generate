#include "librobot_traj_generate/librobot_traj_generate.hpp"


LibTrajGenerate::LibTrajGenerate(const Eigen::VectorXd q0, const Eigen::VectorXd qf, double tf):q0_(q0),qf_(qf), travelling_time_(tf)
{

        distance_ = qf_- q0_;

}

LibTrajGenerate::~LibTrajGenerate()
{
}



void LibTrajGenerate::doFifthDegreeTrajectory(double time_step)
{

        double factor = time_step/travelling_time_;

        if (factor<= 1)
        {
               r_ = 10 * pow(factor, 3) -15 * pow(factor, 4) + 6 * pow(factor, 5);

                dr_ = 30 * pow(factor, 4) - 60 * pow(factor, 3) + 30 * pow(factor, 2);

                ddr_ = 120 * pow(factor, 3) - 180 * pow(factor, 2) + 60 * factor;
        }
        else
        {
                std::cout<<"The time step is reaching the travelling time"<<std::endl;
        }
        
        

        


}



void LibTrajGenerate::getTrajectory(Eigen::VectorXd &q, Eigen::VectorXd &dq, Eigen::VectorXd &ddq)
{

    q = q0_ + r_ * distance_;

    dq = dr_* distance_;

    ddq = ddr_ * distance_;

}