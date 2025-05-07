#ifndef SIMPLEBOTLOGIC_NODE_HPP
#define SIMPLEBOTLOGIC_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <std_msgs/msg/string.hpp>
#include <audio_tools/msg/audio_data_stamped.hpp>
#include <audio_tools/msg/voice_activity.hpp>
#include <lowwi/msg/wake_word.hpp>

class SimpleBotLogicNode : public rclcpp::Node {
public:
  SimpleBotLogicNode();

private:
  // Timer for timeout control
  rclcpp::TimerBase::SharedPtr _timeout_timer;

  // Subscriptions
  rclcpp::Subscription<lowwi::msg::WakeWord>::SharedPtr _lowwi_sub;
  rclcpp::Subscription<audio_tools::msg::AudioDataStamped>::SharedPtr _audio_sub;
  rclcpp::Subscription<audio_tools::msg::VoiceActivity>::SharedPtr _voice_activity_sub;

  // Publisher
  rclcpp::Publisher<audio_tools::msg::AudioDataStamped>::SharedPtr _audio_forward_pub;

  // Callbacks
  void lowwi_callback(const lowwi::msg::WakeWord::SharedPtr msg);
  void audio_callback(const audio_tools::msg::AudioDataStamped::SharedPtr msg);
  void voice_activity_callback(const audio_tools::msg::VoiceActivity::SharedPtr msg);

  // Internal state
  bool _forward_audio;
  std::string _bot_name;
  
  std::deque<audio_tools::msg::AudioDataStamped::SharedPtr> _audio_buffer;
  const size_t _max_buffer_size = 100;  // Store last ~100 packets (adjust as needed)
};

#endif /* SIMPLEBOTLOGIC_NODE_HPP */
