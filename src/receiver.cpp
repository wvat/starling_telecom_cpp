#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
#include <net/if.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

constexpr int BUFFER_SIZE = 1024; // Size of the RX buffer

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MessageReceiver : public rclcpp::Node
{
  public:
    MessageReceiver()
    : Node("message_receiver")
    {
      if ((socketfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket() error");
        exit(2);
      }

      const char* interfaceName = "wlan0";

      if (setsockopt(socketfd, SOL_SOCKET, SO_BINDTODEVICE, interfaceName, strlen(interfaceName)) == -1) {
        perror("setsockopt() error");
        close(socketfd);
        exit(3);
      }

      serverAddr.sin_family = AF_INET;
      serverAddr.sin_port = htons(901);
      serverAddr.sin_addr.s_addr = inet_addr("10.42.0.50");

      if (bind(socketfd, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) == -1) {
        perror("bind() error");
        close(socketfd);
        exit(4);
      }

      publisher_ = this->create_publisher<std_msgs::msg::String>("outgoing_data", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&MessageReceiver::receive_message, this));
    }

  private:
    void receive_message()
    {
      std::vector<char> buffer(BUFFER_SIZE / sizeof(char));
      sockaddr_in clientAddr {};
      socklen_t clientAddrLen = sizeof(clientAddr);
      ssize_t numBytesReceived;

      numBytesReceived = recvfrom(
        socketfd,
        buffer.data(),
        BUFFER_SIZE,
        0,
        reinterpret_cast<sockaddr*>(&clientAddr),
        &clientAddrLen
      );
      if (numBytesReceived == -1) {
        perror("recvfrom() error");
        close(socketfd);
        exit(5);
      }

      size_t numCharsReceived = numBytesReceived / sizeof(char);
      std::string message(buffer.begin(), buffer.begin() + numCharsReceived);

      std::string ipAddress = inet_ntoa(clientAddr.sin_addr);

      auto msg = std_msgs::msg::String();
      msg.data = message;
      RCLCPP_INFO(this->get_logger(), "I received this message from %s: '%s'", ipAddress.c_str(), msg.data.c_str());
      publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int socketfd;
    sockaddr_in serverAddr;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MessageReceiver>());
  rclcpp::shutdown();
  return 0;
}
