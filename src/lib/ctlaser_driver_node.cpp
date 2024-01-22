#include <ctlaser_driver/ctlaser_driver_node.hpp>

CtlaserDriverNode::CtlaserDriverNode(ros::NodeHandle &nh) :
    nh(nh)
{
    node_name = ros::this_node::getName();

    std::string temp_str;
    if (!nh.getParam("/" + node_name + "/laser_ip", temp_str))
        throw std::runtime_error("[ ERROR] Could not find laser_ip in the parameter server.");

    int temp_int;
    if (!nh.getParam("/" + node_name + "/laser_port", temp_int))
        throw std::runtime_error("[ ERROR] Could not find laser_ip in the parameter server.");

    setServerInfo(temp_str, temp_int);
    configureServerAddress();
    stablishConnection();

    bool temp_bool;
    if (!nh.getParam("/" + node_name + "/target_lights", temp_bool))
        std::cerr << "[ WARN] Could not find target_lights state in the parameter server. "
                     "Using default." << std::endl;
    else
        setLaserState(temp_bool);

    float temp_float;
    if (!nh.getParam("/" + node_name + "/emissivity", temp_float))
        std::cerr << "[ WARN] Could not find emissivity in the parameter server. "
                     "Using default." << std::endl;
    else
        setIrVariables(temp_float, CtlaserCommand::SET_EPSILON);

    if (!nh.getParam("/" + node_name + "/transmissivity", temp_float))
        std::cerr << "[ WARN] Could not find transmissivity in the parameter server. "
                     "Using default." << std::endl;
    else
        setIrVariables(temp_float, CtlaserCommand::SET_TRANSMISSION);

    temp_pub = nh.advertise<sensor_msgs::Temperature>(node_name + "/current_temperature", 5);
    lights_srv = nh.advertiseService(node_name + "/lights_switch", &CtlaserDriverNode::lightsServiceCallback, this);
    set_epsilon_srv = nh.advertiseService(node_name + "/set_emissivity", &CtlaserDriverNode::setEpsilonServiceCallback, this);
    set_transm_srv = nh.advertiseService(node_name + "/set_transmissivity", &CtlaserDriverNode::setTramsissionServiceCallback, this);
}

void CtlaserDriverNode::publishTemperature()
{
    readTemperature();

    sensor_msgs::Temperature msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = node_name + "_focal_frame";
    msg.temperature = getCurrentTemperature();
    msg.variance = 0;

    temp_pub.publish(msg);
}

bool CtlaserDriverNode::lightsServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = setLaserState(req.data);

    if (res.success)
        res.message = req.data ? "Lights on!" : "Lights off!";

    return true;
}

bool CtlaserDriverNode::setEpsilonServiceCallback(ctlaser_driver::set_float::Request &req, ctlaser_driver::set_float::Response &res)
{
    res.success = setIrVariables(req.value, CtlaserCommand::SET_EPSILON);
    return true;
}

bool CtlaserDriverNode::setTramsissionServiceCallback(ctlaser_driver::set_float::Request &req, ctlaser_driver::set_float::Response &res)
{
    res.success = setIrVariables(req.value, CtlaserCommand::SET_TRANSMISSION);
    return true;
}

CtlaserDriverNode::~CtlaserDriverNode()
{

}
