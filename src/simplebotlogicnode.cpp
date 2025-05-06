#include "simplebotlogicnode.hpp"
#include "audio_tools/msg/audio_data_stamped.hpp"
#include "audio_tools/msg/voice_activity.hpp"
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <chrono>

using namespace std::chrono_literals;

SimpleBotLogicNode::SimpleBotLogicNode() : Node("custom_node"), _forward_audio(false) {
  RCLCPP_INFO(this->get_logger(), "Custom node has been started.");

  // Declare parameter for wakeword name
  this->declare_parameter<std::string>("bot_name", "Hey Mycroft!");
  this->get_parameter("bot_name", _bot_name);

  // Subscriber to lowwi_ww (wake word)
  _lowwi_sub = this->create_subscription<std_msgs::msg::String>(
      "lowwi_ww", 10,
      std::bind(&SimpleBotLogicNode::lowwi_callback, this, std::placeholders::_1));

  // Subscriber to audio_stamped
  _audio_sub = this->create_subscription<audio_tools::msg::AudioDataStamped>(
      "audio_stamped", 10,
      std::bind(&SimpleBotLogicNode::audio_callback, this, std::placeholders::_1));

  // Subscriber to voice activity
  _voice_activity_sub = this->create_subscription<audio_tools::msg::VoiceActivity>(
      "/voice-activity", 10,
      std::bind(&SimpleBotLogicNode::voice_activity_callback, this, std::placeholders::_1));

  // Publisher to /whisper/audio_in
  _audio_forward_pub = this->create_publisher<audio_tools::msg::AudioDataStamped>(
      "/whisper/audio_in", 10);

  // Timer to stop forwarding after 10 seconds
  _timeout_timer = this->create_wall_timer(10s, [this]() {
    if (_forward_audio) {
      RCLCPP_INFO(this->get_logger(), "Timeout reached. Stopping audio forwarding.");
      _forward_audio = false;
    }
  });
  _timeout_timer->cancel(); // initially disabled
}

void SimpleBotLogicNode::lowwi_callback(const std_msgs::msg::String::SharedPtr msg) {
  if (msg->data == _bot_name) {
    RCLCPP_INFO(this->get_logger(), "Wake word detected: %s", msg->data.c_str());
    _forward_audio = true;
    _timeout_timer->reset();
    _timeout_timer->reset(); // ensures it will count again from 10s
  }
}

void SimpleBotLogicNode::audio_callback(const audio_tools::msg::AudioDataStamped::SharedPtr msg) {
  if (_forward_audio) {
    _audio_forward_pub->publish(*msg);
  }
}

void SimpleBotLogicNode::voice_activity_callback(const audio_tools::msg::VoiceActivity::SharedPtr msg) {
  if (msg->active) {
    _timeout_timer->reset(); // reset 10 second timer on activity
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBotLogicNode>());
  rclcpp::shutdown();
  return 0;
}
