#include "polygon_plugins/polygon_plugins.h"

void polygon_plugins::Triangle::initialize(double side_length)
{
  side_length_ = side_length;
}

double polygon_plugins::Triangle::area()
{
  return 0.5 * side_length_ * getHeight();
}

double polygon_plugins::Triangle::getHeight()
{
  return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
}

void polygon_plugins::Square::initialize(double side_length)
{
  side_length_ = side_length;
}

double polygon_plugins::Square::area()
{
  return side_length_ * side_length_;
}

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
