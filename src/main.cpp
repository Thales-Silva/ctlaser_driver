#include <ctlaser_driver/ctlaser_driver_node.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ctlaser_driver");

    ros::NodeHandle nh("~");

    CtlaserDriverNode cdn(nh);

    double frequency = 1;
    if (!nh.getParam("/" + ros::this_node::getName() + "/frequency", frequency))
        std::cerr << "[ WARN] Could not find parameter hz in the parameter server. "
                     "Using frequency equals to 1 hz." << std::endl;

    ros::Rate r(frequency);
    while (ros::ok())
    {
        cdn.publishTemperature();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
