#ifndef SIMPLEBOTLOGIC_NODE_HPP
#define SIMPLEBOTLOGIC_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <audio_tools/msg/audio_data_stamped.hpp>
#include <audio_tools/msg/voice_activity.hpp>

class SimpleBotLogicNode : public rclcpp::Node {
public:
  SimpleBotLogicNode();

private:
  // Timer for timeout control
  rclcpp::TimerBase::SharedPtr _timeout_timer;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _lowwi_sub;
  rclcpp::Subscription<audio_tools::msg::AudioDataStamped>::SharedPtr _audio_sub;
  rclcpp::Subscription<audio_tools::msg::VoiceActivity>::SharedPtr _voice_activity_sub;

  // Publisher
  rclcpp::Publisher<audio_tools::msg::AudioDataStamped>::SharedPtr _audio_forward_pub;

  // Callbacks
  void lowwi_callback(const std_msgs::msg::String::SharedPtr msg);
  void audio_callback(const audio_tools::msg::AudioDataStamped::SharedPtr msg);
  void voice_activity_callback(const audio_tools::msg::VoiceActivity::SharedPtr msg);

  // Internal state
  bool _forward_audio;
  std::string _bot_name;
};

#endif /* SIMPLEBOTLOGIC_NODE_HPP */
