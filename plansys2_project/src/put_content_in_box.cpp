#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class PutContentInBox : public plansys2::ActionExecutorClient
{
public:
  PutContentInBox()
      : plansys2::ActionExecutorClient("put_content_in_box", 200ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    std::vector<std::string> arguments = get_arguments();
    if (progress_ < 1.0)
    {
      progress_ += 0.2;
      send_feedback(progress_, "Robot " + arguments[2] + " is putting content " + arguments[1] + " in box " + arguments[0] + " in zone " + arguments[3] + "\n");
    }
    else
    {
      finish(true, 1.0, "Robot " + arguments[2] + " putted content " + arguments[1] + " in box " + arguments[0] + " in zone " + arguments[3] + "\n");
      progress_ = 0.0;
      std::cout << std::endl;
    }
    std::cout << "Robot " + arguments[2] + " is putting content " + arguments[1] + " in box " + arguments[0] + " in zone " + arguments[3] + "\n" << std::flush;
    // std::cout << "Robot " + arguments[2] + " is putting content " + arguments[1] + " in box " + arguments[0] + " in zone " + arguments[3] + " [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }
  float progress_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PutContentInBox>();
  node->set_parameter(rclcpp::Parameter("action_name", "put_content_in_box"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}