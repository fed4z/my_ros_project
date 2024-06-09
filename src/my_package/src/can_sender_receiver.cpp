#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "canlib.h"
#include <vector>
#include <thread>
#include <chrono>

class CANSenderReceiver : public rclcpp::Node
{
public:
  CANSenderReceiver() : Node("can_sender_receiver")
  {
    // Initialize CAN library
    canInitializeLibrary();

    // Subscriber setup
    subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
        "can_topic", 10, std::bind(&CANSenderReceiver::topic_callback, this, std::placeholders::_1));

    // Publisher setup
    publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("received_can_topic", 10);

    // CAN setup
    channel_ = canOpenChannel(0, canOPEN_ACCEPT_VIRTUAL);
    if (channel_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CAN channel");
      rclcpp::shutdown();
    } else {
      canSetBusParams(channel_, canBITRATE_250K, 0, 0, 0, 0, 0);
      canBusOn(channel_);

      // Start a thread to handle CAN reception
      receiver_thread_ = std::thread(&CANSenderReceiver::receive_can_messages, this);
    }
  }

  ~CANSenderReceiver()
  {
    // Cleanup
    if (channel_ >= 0) {
      canBusOff(channel_);
      canClose(channel_);
    }
    if (receiver_thread_.joinable()) {
      receiver_thread_.join();
    }
  }

private:
  void topic_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg)
  {
    if (msg->data.size() > 8) {
      RCLCPP_WARN(this->get_logger(), "Message size larger than 8 bytes, truncating");
    }

    // Send CAN message
    canWrite(channel_, 123, msg->data.data(), std::min(static_cast<size_t>(8), msg->data.size()), 0);
    canWriteSync(channel_, 100);
  }

  void receive_can_messages()
  {
    canStatus stat;
    long id;
    unsigned int dlc, flags;
    unsigned char msg[8];
    unsigned long timestamp;

    while (rclcpp::ok()) {
      stat = canReadWait(channel_, &id, msg, &dlc, &flags, &timestamp, 100);
      if (stat == canOK) {
        if (flags & canMSG_ERROR_FRAME) {
          RCLCPP_ERROR(this->get_logger(), "***ERROR FRAME RECEIVED***");
        } else {
          // Prepare and publish the received CAN message
          auto message = std_msgs::msg::Int8MultiArray();
          message.data.assign(msg, msg + dlc);
          publisher_->publish(message);

          // Print the received message
          RCLCPP_INFO(this->get_logger(), "Id: %ld, Msg: %u %u %u %u %u %u %u %u length: %u Flags: %lu",
                      id, msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7], dlc, timestamp);
        }
      } else if (stat != canERR_NOMSG) {
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        RCLCPP_ERROR(this->get_logger(), "canRead: failed, stat=%d (%s)", (int)stat, buf);
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr publisher_;
  canHandle channel_;
  std::thread receiver_thread_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CANSenderReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}