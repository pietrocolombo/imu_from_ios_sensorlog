#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

ros::Publisher imu_pub;

using namespace std;
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "imu_from_ios_sensorlog_node");

	ros::NodeHandle nh;

    sensor_msgs::Imu msg_imu;
    int port, maxline;
    std::string ip, frame_id, topic;

    // param
    nh.param("ip", ip, std::string("192.168.0.60"));
    nh.param("port", port, 12000);
    nh.param("maxline", maxline, 4096);
    nh.param("topic", topic, std::string("imu_iphone"));
    nh.param("frame_id", frame_id, std::string("imu_iphone"));
    
    imu_pub = nh.advertise<sensor_msgs::Imu>(topic, 1);

    msg_imu.header.frame_id = frame_id;


    int sockfd;
    struct sockaddr_in serverAddr;
    char buffer[maxline];
    socklen_t addr_size;
    int n;
    std::vector<std::string> result;

    sockfd = socket(PF_INET, SOCK_STREAM, 0);  // for udp SOCK_DGRAM for tcp SOCK_STREAM
    memset(&serverAddr, '\0', sizeof(serverAddr));

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = inet_addr(ip.c_str());


    if (connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
        {
            printf("\nConnection Failed \n");
            return -1;
        }

    n = read( sockfd , buffer, maxline);

    buffer[n] = '\0';
    ROS_INFO_STREAM(buffer);
    ROS_INFO_STREAM(n);

    result.clear();
    stringstream s_stream(buffer);
    while( s_stream.good() )
    {
        string substr;
        getline( s_stream, substr, ',' );
        result.push_back( substr );
        ROS_INFO_STREAM(substr);
    }
    if(result.size() == 25)
    {
        ROS_INFO_STREAM("invalid data buffer");
    }

    /*
    strcpy(buffer, "Hello Server\n");
    sendto(sockfd, buffer, maxline, 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    printf("[+]Data Send: %s", buffer);


    addr_size = sizeof(serverAddr);
    n = recvfrom(sockfd, buffer, maxline, 0, (struct sockaddr*)&serverAddr, &addr_size);
    buffer[n] = '\0';

    n = recvfrom(sockfd, buffer, maxline, 0, (struct sockaddr*)&serverAddr, &addr_size);
    buffer[n] = '\0';

    std::ofstream myfile;
    myfile.open("/home/usad2/example.csv");
    myfile << buffer;
    myfile.close();

    ROS_INFO_STREAM(buffer);
    ROS_INFO_STREAM(n);
    */

    while (ros::ok())
    {
        //n = recvfrom(sockfd, buffer, maxline, 0, (struct sockaddr*)&serverAddr, &addr_size);
        n = read( sockfd , buffer, maxline);
        buffer[n] = '\0';
        //ROS_INFO_STREAM(buffer);
        if(n>750){
            ROS_INFO_STREAM(n);
            ROS_INFO_STREAM("Transmission error");
        }else{
            result.clear();
            stringstream s_stream(buffer);
            while( s_stream.good() )
            {
                string substr;
                getline( s_stream, substr, ',' );
                result.push_back( substr );
                //ROS_INFO_STREAM(substr);
            }
            if(result.size() == 25)
            {
                //ROS_INFO_STREAM("buffer data ok");

                msg_imu.header.stamp = ros::Time::now();
                msg_imu.header.seq = stoi(result[1]);
                tf2::Quaternion orentation;
                orentation.setRPY(std::stod(result[4]), std::stod(result[5]), std::stod(result[3]));
                msg_imu.orientation.w = orentation.getW();
                msg_imu.orientation.x = orentation.getX();
                msg_imu.orientation.y = orentation.getY();
                msg_imu.orientation.z = orentation.getZ();
                msg_imu.orientation_covariance = {0.00872665, 0.0, 0.0, 0.0, 0.00872665, 0.0, 0.0, 0.0, 0.00872665};

                msg_imu.angular_velocity.x = std::stod(result[6]);
                msg_imu.angular_velocity.y = std::stod(result[7]);
                msg_imu.angular_velocity.z = std::stod(result[8]);
                msg_imu.angular_velocity_covariance = {0.000872665, 0.0, 0.0, 0.0, 0.000872665, 0.0, 0.0, 0.0, 0.000872665};

                msg_imu.linear_acceleration.x = std::stod(result[9])*9.81;
                msg_imu.linear_acceleration.y = std::stod(result[10])*9.81;
                msg_imu.linear_acceleration.z = std::stod(result[11])*9.81;
                msg_imu.linear_acceleration_covariance = {0.003, 0.0, 0.0, 0.0, 0.003, 0.0, 0.0, 0.0, 0.003};

                imu_pub.publish(msg_imu);
            }

        }

        ros::spinOnce();
    }
    //close(sockfd);

}
