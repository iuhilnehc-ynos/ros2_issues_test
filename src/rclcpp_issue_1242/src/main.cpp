#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TestNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit TestNode(const rclcpp::NodeOptions options)
      : rclcpp_lifecycle::LifecycleNode("test_node", options)
    {}

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State&) { return LifecycleCallbackReturn::SUCCESS; }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State&)
    {
        auto node = std::make_shared<rclcpp::Node>("service_node");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State&) { return LifecycleCallbackReturn::SUCCESS; }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State&) { return LifecycleCallbackReturn::SUCCESS; }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State&) { return LifecycleCallbackReturn::SUCCESS; }

    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State&) { return LifecycleCallbackReturn::SUCCESS; }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto scheduler = std::make_shared<TestNode>(options);
  exec.add_node(scheduler->get_node_base_interface());

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
