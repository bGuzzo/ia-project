#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveRobot : public plansys2::ActionExecutorClient
{
public:
    MoveRobot()
        : plansys2::ActionExecutorClient("move_robot", 200ms)
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
            send_feedback(progress_, "Robot " + arguments[0] + " is moving from zone " + arguments[1] + " to zone " + arguments[2]) + " [" << std::min(100.0, progress_ * 100.0) << "%]  \n" << std::flush;
        }
        else
        {
            finish(true, 1.0, "Robot " + arguments[0] + " moved from zone " + arguments[1] + " to zone " + arguments[2]) + " [" << std::min(100.0, progress_ * 100.0) << "%]  \n" << std::flush;
            progress_ = 0.0;
            std::cout << std::endl;
        }
    }
    float progress_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobot>();
    node->set_parameter(rclcpp::Parameter("action_name", "move_robot"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}