#include <iostream>
#include <sstream>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

double LF_VEL = 0.0 , RF_VEL = 0.0 ;

void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  LF_VEL = msg->axes[1]*100;
  RF_VEL = msg->axes[3]*100;
  
}



int main(int argc , char* argv[])
{
    ros::init(argc , argv , "Raisim_Server");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000);
    ros::Subscriber sub = n.subscribe("joy", 1000, joystickCallback);


    std_msgs::String msg;

    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World::setActivationKey(binaryPath.getDirectory()+"\\rsc\\activation.raisim");

    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);

    /// create objects
    auto ground = world.addGround();
    ground->setAppearance("steel");
    auto robot = world.addArticulatedSystem("/home/yunjin/Documents/ros_ws/src/raisim_ros/models/megabot/smb.urdf");

    robot->setName("smb");
    Eigen::VectorXd gc(robot->getGeneralizedCoordinateDim()), gv(robot->getDOF());
    gc.setZero(); gv.setZero();
  //  raisim::Vec<4> quat; quat = {0, 0.0499792, 0, 0.9987503}; quat/= quat.norm();
    gc.segment<7>(0) << 0, 0, 0.197, 1, 0, 0, 0;
    
    

    robot->setGeneralizedCoordinate(gc);
    robot->setGeneralizedVelocity(gv);
    robot->setGeneralizedForce({0, 0, 0, 0, 0, -80, 0, 0, 0, 0});
    robot->setIntegrationScheme(raisim::ArticulatedSystem::IntegrationScheme::TRAPEZOID);
    robot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

    double *LF_WHEEL = &gv(6);
    double *RF_WHEEL = &gv(7);
    double *LH_WHEEL = &gv(8);
    double *RH_WHEEL = &gv(9);


    
    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);

    
    while (ros::ok())
    {

        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        server.integrateWorldThreadSafe();

        *LF_WHEEL = LF_VEL;
        *RF_WHEEL = RF_VEL;

        *LH_WHEEL = LF_VEL;
        *RH_WHEEL = RF_VEL;

        std::stringstream ss;
        robot->setGeneralizedVelocity(gv);
        // //std::cout << gv << "\n--------\n" ;
        // std::cout << LF_VEL << "\n--------\n" ;
        // std::cout << RF_VEL << "\n--------\n" ;

        msg.data = "It is Working Now!";

        chatter_pub.publish(msg);

        
       
        ros::spinOnce();
    }


        

    std::cout<<"total mass "<<robot->getCompositeMass()[0]<<std::endl;

    
    server.killServer();

    return 0 ;
}