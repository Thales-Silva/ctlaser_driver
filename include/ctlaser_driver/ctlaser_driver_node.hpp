#ifndef CTLASER_DRIVER_NODE_HPP
#define CTLASER_DRIVER_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/SetBool.h>
#include <ctlaser_driver/set_float.h>

#include <ctlaser_driver/ctlaser_driver.hpp>

class CtlaserDriverNode : public CtlaserDriver
{
    private:
        ros::NodeHandle nh;
        std::string node_name;
        ros::Publisher temp_pub;
        ros::ServiceServer lights_srv;
        ros::ServiceServer set_epsilon_srv;
        ros::ServiceServer set_transm_srv;

    public:
        CtlaserDriverNode(ros::NodeHandle &nh);
        ~CtlaserDriverNode();

        void publishTemperature();
        bool lightsServiceCallback(std_srvs::SetBool::Request &req,
                                   std_srvs::SetBool::Response &res);
        bool setEpsilonServiceCallback(ctlaser_driver::set_float::Request &req,
                                       ctlaser_driver::set_float::Response &res);
        bool setTramsissionServiceCallback(ctlaser_driver::set_float::Request &req,
                                           ctlaser_driver::set_float::Response &res);
};

#endif
