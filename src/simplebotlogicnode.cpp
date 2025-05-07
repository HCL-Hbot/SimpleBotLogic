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
  this->declare_parameter<std::string>("bot_name", "Hey Mycroft");
  this->get_parameter("bot_name", _bot_name);

  // Subscriber to lowwi_ww (wake word)
  _lowwi_sub = this->create_subscription<lowwi::msg::WakeWord>(
      "/lowwi_ww", 10,
      std::bind(&SimpleBotLogicNode::lowwi_callback, this, std::placeholders::_1));

  // Subscriber to audio_stamped
  _audio_sub = this->create_subscription<audio_tools::msg::AudioDataStamped>(
      "/audio_stamped", 10,
      std::bind(&SimpleBotLogicNode::audio_callback, this, std::placeholders::_1));

  // Subscriber to voice activity
  _voice_activity_sub = this->create_subscription<audio_tools::msg::VoiceActivity>(
      "/voice_activity", 10,
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

void SimpleBotLogicNode::lowwi_callback(const lowwi::msg::WakeWord::SharedPtr msg) {
  if (msg->wakeword_name == _bot_name) {
    RCLCPP_INFO(this->get_logger(), "Wake word detected: %s", msg->wakeword_name.c_str());

    _forward_audio = true;
    _timeout_timer->reset();

    // How many previous packets to include
    constexpr size_t PREV_PACKET_COUNT = 10;

    // Find iterator to first packet >= wakeword timestamp
    rclcpp::Time wake_ts(msg->header.stamp);
    auto it = std::find_if(
      _audio_buffer.begin(), _audio_buffer.end(),
      [wake_ts](const auto& pkt) {
        return rclcpp::Time(pkt->header.stamp) >= wake_ts;
      }
    );

    // Determine starting point N packets before
    auto start_it = (it == _audio_buffer.end()) ? it : it;
    for (int i = 0; i < PREV_PACKET_COUNT && start_it != _audio_buffer.begin(); ++i) {
      --start_it;
    }

    // Publish from start_it to end
    for (auto pub_it = start_it; pub_it != _audio_buffer.end(); ++pub_it) {
      _audio_forward_pub->publish(**pub_it);
    }
  }
}



void SimpleBotLogicNode::audio_callback(const audio_tools::msg::AudioDataStamped::SharedPtr msg) {
  if (_audio_buffer.size() >= _max_buffer_size) {
    _audio_buffer.pop_front();
  }
  _audio_buffer.push_back(msg);

  // If forwarding is active, publish the current message
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
