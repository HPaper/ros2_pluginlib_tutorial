#include "rclcpp/rclcpp.hpp"
#include <pluginlib/class_loader.h>
#include "polygon_base/polygon_base.h"

namespace polygon_loader
{
class PolygonLoader : public rclcpp::Node
{
private:
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader_;
  std::shared_ptr<polygon_base::RegularPolygon> triangle_;
  std::shared_ptr<polygon_base::RegularPolygon> square_;

public:
  PolygonLoader()
    : Node("polygon_loader")
    , poly_loader_("polygon_base", "polygon_base::RegularPolygon")
    , triangle_(nullptr)
    , square_(nullptr)
  {
    try
    {
      triangle_ = poly_loader_.createSharedInstance("polygon_plugins/regular_triangle");
      triangle_->initialize(10.0);

      square_ = poly_loader_.createSharedInstance("polygon_plugins/regular_square");
      square_->initialize(10.0);

      RCLCPP_INFO(this->get_logger(),"Triangle area: %.2f", triangle_->area());
      RCLCPP_INFO(this->get_logger(),"Square area: %.2f", square_->area());
    }
    catch(pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(this->get_logger(),"The plugin failed to load for some reason. Error: %s", ex.what());
    }
  }

  ~PolygonLoader(){}
};
}


int main(int argc, char** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto polygon_loader = std::make_shared<polygon_loader::PolygonLoader>();

  executor.add_node(polygon_loader);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
