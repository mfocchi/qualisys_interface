#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "RTProtocol.h"
#include "RTPacket.h"
#include <Eigen/Geometry>
#include <iostream>
#include <chrono>
#include <unistd.h>

using namespace std::chrono;

class RTProtocolNode : public rclcpp::Node
{
public:
    RTProtocolNode()
        : Node("rt_protocol_node"), j(0), data_available(false), stream_frames(false), udp_port(6734)
    {
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/optitrack/pose", 10);
        start = high_resolution_clock::now();
    }

    void run()
    {
        while (rclcpp::ok())
        {
            if (!rtProtocol.Connected())
            {
                if (!rtProtocol.Connect(serverAddr, basePort, &udp_port, majorVersion, minorVersion, bigEndian))
                {
                    RCLCPP_ERROR(this->get_logger(), "rtProtocol.Connect: %s", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
            }

            if (!data_available)
            {
                if (!rtProtocol.Read6DOFSettings(data_available))
                {
                    RCLCPP_ERROR(this->get_logger(), "rtProtocol.Read6DOFSettings: %s", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
            }

            if (!stream_frames)
            {
                if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udp_port, NULL, CRTProtocol::cComponent6d))
                {
                    RCLCPP_ERROR(this->get_logger(), "rtProtocol.StreamFrames: %s", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
                stream_frames = true;
                RCLCPP_INFO(this->get_logger(), "Starting to stream 6DOF data");
            }

            CRTPacket::EPacketType packet_type;
            if (rtProtocol.ReceiveRTPacket(packet_type, true) > 0)
            {
                if (packet_type == CRTPacket::PacketData)
                {
                    float fX, fY, fZ;
                    float rotationMatrix[9];

                    CRTPacket* rtPacket = rtProtocol.GetRTPacket();

                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {
                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                        {
                            auto message = geometry_msgs::msg::PoseStamped();
                            message.header.stamp = this->now();
                            const char* body_name = rtProtocol.Get6DOFBodyName(i);
                            message.header.frame_id = body_name ? body_name : "world";

                            message.pose.position.x = fX/1000.0;
                            message.pose.position.y = fY/1000.0;
                            message.pose.position.z = fZ/1000.0;

                            Eigen::Matrix3f rotation;
                            rotation << rotationMatrix[0], rotationMatrix[1], rotationMatrix[2],
                                        rotationMatrix[3], rotationMatrix[4], rotationMatrix[5],
                                        rotationMatrix[6], rotationMatrix[7], rotationMatrix[8];
                            Eigen::Quaternionf quaternion(rotation.transpose());

                            message.pose.orientation.x = quaternion.x();
                            message.pose.orientation.y = quaternion.y();
                            message.pose.orientation.z = quaternion.z();
                            message.pose.orientation.w = quaternion.w();

                            pose_publisher_->publish(message);
                        }
                    }

                    auto current = high_resolution_clock::now();
                    duration<double> elapsed = current - start;
                    j++;
                    double averageFrequency = j / elapsed.count();
                }
            }

            rclcpp::spin_some(this->get_node_base_interface());
        }

        rtProtocol.StopCapture();
        rtProtocol.Disconnect();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    CRTProtocol rtProtocol;
    const char serverAddr[16] = "192.168.1.216";
    const unsigned short basePort = 22222;
    const int majorVersion = 1;
    const int minorVersion = 19;
    const bool bigEndian = false;
    bool data_available;
    bool stream_frames;
    unsigned short udp_port;
    int j;
    high_resolution_clock::time_point start;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RTProtocolNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
