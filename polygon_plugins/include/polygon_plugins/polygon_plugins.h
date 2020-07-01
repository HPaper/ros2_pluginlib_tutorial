#ifndef PLUGINLIB_TUTORIALS_POLYGON_PLUGINS_H_
#define PLUGINLIB_TUTORIALS_POLYGON_PLUGINS_H_

#include <cmath>
#include <polygon_base/polygon_base.h>
#include <pluginlib/class_list_macros.h>

namespace polygon_plugins
{
  class Triangle : public polygon_base::RegularPolygon
  {
    public:
      Triangle(){}
      void initialize(double side_length);
      double area();
      double getHeight();

    private:
      double side_length_;
  };

  class Square : public polygon_base::RegularPolygon
  {
    public:
      Square(){}
      void initialize(double side_length);
      double area();

    private:
      double side_length_;
  };
}
#endif
