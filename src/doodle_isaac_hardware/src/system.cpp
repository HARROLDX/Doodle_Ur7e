#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <thread>
#include <cstdlib>

namespace doodle_isaac_hardware {
class System : public hardware_interface::SystemInterface {
  std::vector<double> q_, v_, cmd_;
  double scaling_ = 0.;
  bool received_ = false;
  std::chrono::steady_clock::time_point last_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    const char * domain = std::getenv("ROS_DOMAIN_ID");
    if (!domain || std::string(domain) != "231" || info_.joints.size() != 6)
      return hardware_interface::CallbackReturn::ERROR;
    q_.resize(6); v_.resize(6); cmd_.resize(6);
    node_ = std::make_shared<rclcpp::Node>("doodle_isaac_hardware");
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/doodle_sim/joint_commands", 1);
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/doodle_sim/joint_feedback", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::vector<double> q(6), v(6);
        for (size_t i=0; i<6; ++i) {
          auto it=std::find(msg->name.begin(),msg->name.end(),info_.joints[i].name);
          auto j=std::distance(msg->name.begin(),it);
          if(it==msg->name.end() || size_t(j)>=msg->position.size() || size_t(j)>=msg->velocity.size()
              || !std::isfinite(msg->position[j]) || !std::isfinite(msg->velocity[j])) return;
          q[i]=msg->position[j]; v[i]=msg->velocity[j];
        }
        q_=q; v_=v; if(!received_) cmd_=q;
        received_=true; last_=std::chrono::steady_clock::now();
      });
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> out;
    for(size_t i=0;i<6;++i) {
      out.emplace_back(info_.joints[i].name,"position",&q_[i]);
      out.emplace_back(info_.joints[i].name,"velocity",&v_[i]);
    }
    out.emplace_back("speed_scaling","speed_scaling_factor",&scaling_); return out;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> out;
    for(size_t i=0;i<6;++i) out.emplace_back(info_.joints[i].name,"position",&cmd_[i]);
    return out;
  }
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    auto until=std::chrono::steady_clock::now()+std::chrono::seconds(15);
    while(!received_ && std::chrono::steady_clock::now()<until) {
      rclcpp::spin_some(node_); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    cmd_=q_; return received_ ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
  }
  hardware_interface::return_type read(const rclcpp::Time &,const rclcpp::Duration &) override {
    rclcpp::spin_some(node_);
    bool fresh=received_ && std::chrono::steady_clock::now()-last_<std::chrono::seconds(2);
    scaling_=fresh ? 1. : 0.;
    return fresh ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
  }
  hardware_interface::return_type write(const rclcpp::Time & time,const rclcpp::Duration &) override {
    if(!received_ || scaling_==0.) return hardware_interface::return_type::ERROR;
    sensor_msgs::msg::JointState msg; msg.header.stamp=time; msg.position=cmd_;
    for(auto & j:info_.joints) msg.name.push_back(j.name);
    pub_->publish(msg); return hardware_interface::return_type::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(doodle_isaac_hardware::System,hardware_interface::SystemInterface)
