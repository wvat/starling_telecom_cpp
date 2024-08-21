#include <memory>
#include <atomic>
#include <thread>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <net/if.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MessageTransmitter : public rclcpp::Node
{
  public:
    MessageTransmitter()
    : Node("message_transmitter"), socketfd(-1), serverAddr()
    {
      const char* interfaceName = "wlan0";
      
      if ((socketfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket() error");
        exit(2);
      }

      if (setsockopt(socketfd, SOL_SOCKET, SO_BINDTODEVICE, interfaceName, strlen(interfaceName)) == -1) {
        perror("setsocketopt() error");
        close(socketfd);
        exit(3);
      }

      serverAddr.sin_family = AF_INET;
      serverAddr.sin_port = htons(901);
      serverAddr.sin_addr.s_addr = inet_addr("10.42.0.50");

      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "incoming_data", 10, std::bind(&MessageTransmitter::transmit_message, this, _1));
    }

  private:
    void transmit_message(const std_msgs::msg::String::SharedPtr msg) const
    {
      auto message = msg->data.c_str();
      size_t messageSize = strlen(message) * sizeof(char);

      ssize_t numBytesSent = sendto(
        socketfd,
        reinterpret_cast<const char*>(message),
        messageSize,
        0,
        (sockaddr*) &serverAddr,
        sizeof(serverAddr)
      );
      if (numBytesSent == -1) {
        perror("sendto() error");
        close(socketfd);
        exit(4);
      }

      RCLCPP_INFO(this->get_logger(), "I sent a packet containing: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    int socketfd;
    sockaddr_in serverAddr;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MessageTransmitter>());
  rclcpp::shutdown();
  return 0;
}
