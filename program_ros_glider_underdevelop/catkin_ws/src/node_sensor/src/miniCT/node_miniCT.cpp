// miniCT node made from scratch
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <glider_msg/miniCTmsg.h>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_reader");
    ros::NodeHandle nh;

    std::string port = "/dev/ttyACM0"; // replace with your port name
    int baudrate = 115200;             // replace with your baudrate
    serial::Serial ser(port, baudrate);
    ser.setTimeout(serial::Timeout::max(), 100, 0, 100, 0);

    ros::Publisher miniCTMessager = nh.advertise<glider_msg::miniCTmsg>("messageMiniCT", 1000);
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

        std::stringstream ss(line);
        std::string value1_str, value2_str;
        ss >> value1_str >> value2_str;
        // float conductivity, temperature;
        // ss >> conductivity >> temperature;

        if (!ss.fail())
        {
            float temperature = std::stof(value1_str);
            float conductivity = std::stof(value2_str);
            ROS_INFO("Temperature: %.2f, Conductivity: %.2f", temperature, conductivity);

            glider_msg::miniCTmsg msg;

            msg.conductivity = conductivity;
            msg.temperature = temperature;

            miniCTMessager.publish(msg);
            
        }
        else
        {
            ROS_WARN("Invalid data: %s", line.c_str());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}