#include <rclcpp/rclcpp.hpp>

class PGONode : public rclcpp::Node
{
public:
    PGONode() : Node("pgo_node")
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PGONode>());
    rclcpp::shutdown();
    return 0;
}