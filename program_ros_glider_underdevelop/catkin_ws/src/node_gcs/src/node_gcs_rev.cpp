// miniCT node made from scratch
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <glider_msg/gcsmsg.h>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_reader");
    ros::NodeHandle nh;

    std::string port = "/dev/ttyUSB0"; // replace with your port name
    int baudrate = 115200;             // replace with your baudrate
    serial::Serial ser(port, baudrate);
    ser.setTimeout(serial::Timeout::max(), 100, 0, 100, 0);

    ros::Publisher MessagerGCS = nh.advertise<glider_msg::gcsmsg>("messageGCS", 1000);
    ros::Rate rate(100);

    while (ros::ok())
    {
        while (ser.available())
        {
            std::string data = ser.read(1);
            char c = data[0];
            if (c == '\n')
            {
                break;
            }
        }
        
        std::string line = ser.readline();
        std::cout << line << std::endl;

        glider_msg::gcsmsg msg;
        std::stringstream ss;
        std::string messgGCS;
        ss << messgGCS;
        msg.messageGCS = line;

        ROS_INFO("MessageGCS: %s",msg.messageGCS.c_str());
        //msg.message_GCS = message_GCS;
        MessagerGCS.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}