#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cstdint>

#include "monitor.h"
#include "monitor.c"

using std::placeholders::_1;


float temperature;


class CopilotRV : public rclcpp::Node {
  public:
    CopilotRV() : Node("copilotrv") {
      /*
        Added QoS Profile to the Subscription and Publisher
      */
      rclcpp::QoS qos(1);
      qos.transient_local();


      
      temperature_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "copilot/temperature", qos,
        std::bind(&CopilotRV::temperature_callback, this, _1));
      

      
      heaton_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/heaton", qos);
      
      heatoff_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "copilot/heatoff", qos);
      

    }

    
    // Report (publish) monitor violations.
    void heaton() {
      auto output = std_msgs::msg::Empty();
      heaton_publisher_->publish(output);
    }
    
    // Report (publish) monitor violations.
    void heatoff() {
      auto output = std_msgs::msg::Empty();
      heatoff_publisher_->publish(output);
    }
    

    // Needed so we can report messages to the log.
    static CopilotRV& getInstance() {
      static CopilotRV instance;
      return instance;
    }

  private:
    
    void temperature_callback(const std_msgs::msg::Float32::SharedPtr msg) const {
      temperature = msg->data;
      step();
    }
    
  
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr temperature_subscription_;
    

    
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heaton_publisher_;
    
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heatoff_publisher_;
    

};


// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void heaton(float heaton_arg0) {
  CopilotRV::getInstance().heaton();
}

// Pass monitor violations to the actual class, which has ways to
// communicate with other applications.
void heatoff(float heatoff_arg0) {
  CopilotRV::getInstance().heatoff();
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CopilotRV>());
  rclcpp::shutdown();
  return 0;
}