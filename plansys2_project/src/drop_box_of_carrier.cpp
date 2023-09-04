#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DropBoxOfCarrier : public plansys2::ActionExecutorClient
{
public:
    DropBoxOfCarrier()
        : plansys2::ActionExecutorClient("drop_box_of_carrier", 200ms)
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
            send_feedback(progress_, "Robot " + arguments[2] + " is dropping box " + arguments[0] + " of the carrier " + arguments[1] + " in zone " + arguments[3] + " releasing slot " + arguments[4]) + " [" << std::min(100.0, progress_ * 100.0) << "%]  \n" << std::flush;
        }
        else
        {
            finish(true, 1.0, "Robot " + arguments[2] + " dropped box " + arguments[0] + " of the carrier " + arguments[1] + " in zone " + arguments[3] + " releasing slot " + arguments[4]) + " [" << std::min(100.0, progress_ * 100.0) << "%]  \n" << std::flush;
            progress_ = 0.0;
            std::cout << std::endl;
        }
    }
    float progress_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DropBoxOfCarrier>();
    node->set_parameter(rclcpp::Parameter("action_name", "drop_box_of_carrier"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}