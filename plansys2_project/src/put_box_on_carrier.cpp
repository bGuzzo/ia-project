#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class PutBoxOnCarrier : public plansys2::ActionExecutorClient
{
public:
    PutBoxOnCarrier()
        : plansys2::ActionExecutorClient("put_box_on_carrier", 200ms)
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
            send_feedback(progress_, "Robot " + arguments[2] + " is putting box " + arguments[0] + " on the carrier " + arguments[1] + " in zone " + arguments[3] + " occupying slot " + arguments[4]);
        }
        else
        {
            finish(true, 1.0, "Robot " + arguments[2] + " putted box " + arguments[0] + " on the carrier " + arguments[1] + " in zone " + arguments[3] + " occupying slot " + arguments[4]);
            progress_ = 0.0;
            std::cout << std::endl;
        }
        std::cout << "\r\e[K" << std::flush;
        std::cout << "Robot " + arguments[2] + " is putting box " + arguments[0] + " on the carrier " + arguments[1] + " in zone " + arguments[3] + " occupying slot " + arguments[4] << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
    }
    float progress_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PutBoxOnCarrier>();
    node->set_parameter(rclcpp::Parameter("action_name", "put_box_on_carrier"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}