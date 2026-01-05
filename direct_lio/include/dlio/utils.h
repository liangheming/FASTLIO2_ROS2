/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "rclcpp/rclcpp.hpp"

namespace dlio
{

template <typename T>
struct identity
{
  typedef T type;
};

template <typename T>
void declare_param(rclcpp::Node* node, const std::string param_name, T& param,
                   const typename identity<T>::type& default_value)
{
  node->declare_parameter(param_name, default_value);
  node->get_parameter(param_name, param);
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << a_value;
  return out.str();
}

}  // namespace dlio
