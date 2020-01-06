#include <gtest/gtest.h>
#include "librobot_traj_generate/librobot_traj_generate.hpp"
#include <Eigen/Dense>



TEST(LibTraGenerateTest, testTrajectoryPoint1)
{
    Eigen::VectorXd q0(6);
    q0<<0,0,0,0,0,0;
    Eigen::VectorXd qf(6);
    qf<<1,0,0,0,0,0;    
    double tf = 1;

    LibTrajGenerate trajectory_generator(q0, qf, tf);

    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;

    double time_step = 0;
    trajectory_generator.doFifthDegreeTrajectory(time_step);
    trajectory_generator.getTrajectory(q, dq, ddq);

    // check initial condition
    ASSERT_EQ(q, q0);
    ASSERT_EQ(dq, Eigen::MatrixXd::Zero(6,1));
    ASSERT_EQ(ddq,Eigen::MatrixXd::Zero(6,1));
    std::cout<<"initial condition tested"<<std::endl;

    time_step = tf;
    trajectory_generator.doFifthDegreeTrajectory(time_step);
    trajectory_generator.getTrajectory(q, dq, ddq);


    // check final condition
    ASSERT_EQ(q, qf);
    ASSERT_EQ(dq, Eigen::MatrixXd::Zero(6,1));
    ASSERT_EQ(ddq,Eigen::MatrixXd::Zero(6,1));
    std::cout<<"final condition tested"<<std::endl;


    // check one point
    time_step = 0.6;
    trajectory_generator.doFifthDegreeTrajectory(time_step);
   
    trajectory_generator.getTrajectory(q, dq, ddq);
   
    Eigen::VectorXd q_des(6);
    q_des<<0.68255999999,0,0,0,0,0;
  
    Eigen::VectorXd dq_des(6);
    dq_des<<1.72800000000,0,0,0,0,0;
    Eigen::VectorXd ddq_des(6);
    ddq_des<<-2.8800000,0,0,0,0,0;
 
    ASSERT_NEAR(q(0), q_des(0), 1*pow(10,-4));
    ASSERT_NEAR(dq(0), dq_des(0), 1*pow(10,-4));
    ASSERT_NEAR(ddq(0), ddq_des(0), 1*pow(10,-4));        
    std::cout<<"one point tested"<<std::endl;

    EXPECT_TRUE(true);
}

TEST(LibTraGenerateTest, testTrajectoryPoint2)
{
    Eigen::VectorXd q0(6);
    q0<<0.1,-10, 0.002, 0.55, 1, 2;
    Eigen::VectorXd qf(6);
    qf<<20,10, -2, 2, 0.6,-10;    
    double tf = 10;

    LibTrajGenerate trajectory_generator(q0, qf, tf);

    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;

    double time_step = 0;
    trajectory_generator.doFifthDegreeTrajectory(time_step);
    trajectory_generator.getTrajectory(q, dq, ddq);

    // check initial condition
    ASSERT_EQ(q(0), q0(0));
    ASSERT_EQ(q(1), q0(1));
    ASSERT_EQ(q(2), q0(2));
    ASSERT_EQ(q(3), q0(3));
    ASSERT_EQ(q(4), q0(4));
    ASSERT_EQ(q(5), q0(5));
    ASSERT_EQ(dq(0), 0);
    ASSERT_EQ(dq(1), 0);
    ASSERT_EQ(dq(2), 0);
    ASSERT_EQ(dq(3), 0);
    ASSERT_EQ(dq(4), 0);
    ASSERT_EQ(dq(5), 0);
    ASSERT_EQ(ddq(0), 0);
    ASSERT_EQ(ddq(1), 0);
    ASSERT_EQ(ddq(2), 0);
    ASSERT_EQ(ddq(3), 0);
    ASSERT_EQ(ddq(4), 0);
    ASSERT_EQ(ddq(5), 0);

    // check final condition
    time_step = tf;
    trajectory_generator.doFifthDegreeTrajectory(time_step);
    trajectory_generator.getTrajectory(q, dq, ddq);
    ASSERT_DOUBLE_EQ(q(0), qf(0));
    ASSERT_DOUBLE_EQ(q(1), qf(1));
    ASSERT_DOUBLE_EQ(q(2), qf(2));
    ASSERT_DOUBLE_EQ(q(3), qf(3));
    ASSERT_DOUBLE_EQ(q(4), qf(4));
    ASSERT_DOUBLE_EQ(q(5), qf(5));
    ASSERT_DOUBLE_EQ(dq(0), 0);
    ASSERT_DOUBLE_EQ(dq(1), 0);
    ASSERT_DOUBLE_EQ(dq(2), 0);
    ASSERT_DOUBLE_EQ(dq(3), 0);
    ASSERT_DOUBLE_EQ(dq(4), 0);
    ASSERT_DOUBLE_EQ(dq(5), 0);
    ASSERT_DOUBLE_EQ(ddq(0), 0);
    ASSERT_DOUBLE_EQ(ddq(1), 0);
    ASSERT_DOUBLE_EQ(ddq(2), 0);
    ASSERT_DOUBLE_EQ(ddq(3), 0);
    ASSERT_DOUBLE_EQ(ddq(4), 0);
    ASSERT_DOUBLE_EQ(ddq(5), 0);

    // check one point
    time_step = 0.224* tf;
    trajectory_generator.doFifthDegreeTrajectory(time_step);
    trajectory_generator.getTrajectory(q, dq, ddq);  
    ASSERT_DOUBLE_EQ(q(0), 1.6524680806957059431);
    ASSERT_DOUBLE_EQ(q(1), -8.4397305721651196819);
    ASSERT_DOUBLE_EQ(q(2), -0.15418296972627149044);
    ASSERT_DOUBLE_EQ(q(3),  0.66311953351802888079);
    ASSERT_DOUBLE_EQ(q(4), 0.96879461144330236699);
    ASSERT_DOUBLE_EQ(q(5), 1.063838343299071898);

    ASSERT_DOUBLE_EQ(dq(0), 18.038225436672000512);
    ASSERT_DOUBLE_EQ(dq(1), 18.128869785600002729);
    ASSERT_DOUBLE_EQ(dq(2), -1.8146998655385599974);
    ASSERT_DOUBLE_EQ(dq(3), 1.3143430594560001623);
    ASSERT_DOUBLE_EQ(dq(4), -0.36257739571200003903);
    ASSERT_DOUBLE_EQ(dq(5), -10.877321871360001282);

    ASSERT_DOUBLE_EQ(ddq(0), 114.5653125119999487);
    ASSERT_DOUBLE_EQ(ddq(1), 115.14101759999995522);
    ASSERT_DOUBLE_EQ(ddq(2), -11.525615861759995084);
    ASSERT_DOUBLE_EQ(ddq(3), 8.3477237759999969313);
    ASSERT_DOUBLE_EQ(ddq(4),  -2.3028203519999990156);
    ASSERT_DOUBLE_EQ(ddq(5), -69.084610559999973134);

    EXPECT_TRUE(true);
}


int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
