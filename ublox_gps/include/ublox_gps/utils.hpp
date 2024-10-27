#ifndef UBLOX_GPS_UTILS_HPP
#define UBLOX_GPS_UTILS_HPP

#include <ctime>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/mkgmtime.h>

namespace ublox_node {

/**
 * @brief Convert date/time to UTC time in seconds.
 */
template<typename NavPVT>
time_t toUtcSeconds(const NavPVT& msg) {
  // Create TM struct for mkgmtime
  struct tm time{};
  time.tm_year = msg.year - 1900;
  time.tm_mon = msg.month - 1;
  time.tm_mday = msg.day;
  time.tm_hour = msg.hour;
  time.tm_min = msg.min;
  time.tm_sec = msg.sec;
  // C++ STL doesn't have a mkgmtime (though other libraries do)
  // STL mktime converts date/time to seconds in local time
  // A modified version of code external library is used for mkgmtime
  return mkgmtime(&time);
}

/**
 * @brief Check that the parameter is above the minimum.
 * @param val the value to check
 * @param min the minimum for this value
 * @param name the name of the parameter
 * @throws std::runtime_error if it is below the minimum
 */
template <typename V, typename T>
void checkMin(V val, T min, const std::string & name) {
  if (val < min) {
    std::stringstream oss;
    oss << "Invalid settings: " << name << " must be > " << min;
    throw std::runtime_error(oss.str());
  }
}

/**
 * @brief Check that the parameter is in the range.
 * @param val the value to check
 * @param min the minimum for this value
 * @param max the maximum for this value
 * @param name the name of the parameter
 * @throws std::runtime_error if it is out of bounds
 */
template <typename V, typename T>
void checkRange(V val, T min, T max, const std::string & name) {
  if (val < min || val > max) {
    std::stringstream oss;
    oss << "Invalid settings: " << name << " must be in range [" << min <<
        ", " << max << "].";
    throw std::runtime_error(oss.str());
  }
}

/**
 * @brief Check that the elements of the vector are in the range.
 * @param val the vector to check
 * @param min the minimum for this value
 * @param max the maximum for this value
 * @param name the name of the parameter
 * @throws std::runtime_error value it is out of bounds
 */
template <typename V, typename T>
void checkRange(std::vector<V> val, T min, T max, const std::string & name) {
  for (size_t i = 0; i < val.size(); i++)  {
    std::stringstream oss;
    oss << name << "[" << i << "]";
    checkRange(val[i], min, max, oss.str());
  }
}

/**
 * @brief Get a unsigned integer value from the parameter server.
 * @param key the key to be used in the parameter server's dictionary
 * @param u storage for the retrieved value.
 * @throws std::runtime_error if the parameter is out of bounds
 * @return true if found, false if not found.
 */
template <typename U>
bool getRosUint(rclcpp::Node* node, const std::string& key, U &u) {
  rclcpp::Parameter parameter;
  if (!node->get_parameter(key, parameter)) {
    return false;
  }
  U param = parameter.get_value<U>();
  // Check the bounds
  U min = std::numeric_limits<U>::lowest();
  U max = std::numeric_limits<U>::max();
  checkRange(param, min, max, key);
  // set the output
  u = static_cast<U>(param);
  return true;
}

/**
 * @brief Get a unsigned integer value from the parameter server.
 * @param key the key to be used in the parameter server's dictionary
 * @param u storage for the retrieved value.
 * @param val value to use if the server doesn't contain this parameter.
 * @throws std::runtime_error if the parameter is out of bounds
 * @return true if found, false if not found.
 */
template <typename U, typename V>
void getRosUint(rclcpp::Node* node, const std::string& key, U &u, V default_val) {
  if (!getRosUint(node, key, u)) {
    u = default_val;
  }
}

/**
 * @brief Get a unsigned integer vector from the parameter server.
 * @throws std::runtime_error if the parameter is out of bounds.
 * @return true if found, false if not found.
 */
template <typename U>
bool getRosUint(rclcpp::Node* node, const std::string& key, std::vector<U> &u) {
  std::vector<U> param;
  if (!node->get_parameter(key, param)) {
    return false;
  }

  // Check the bounds
  U min = std::numeric_limits<U>::lowest();
  U max = std::numeric_limits<U>::max();
  checkRange(param, min, max, key);

  // set the output
  u.insert(u.begin(), param.begin(), param.end());
  return true;
}
/**
 * @brief Alternate version to Get a unsigned integer vector from the parameter server. Avoids coercing to uint_8 vector in rclcpp internals
 * @throws std::runtime_error if the parameter is out of bounds.
 * @return true if found, false if not found.
*/
template <typename U>
bool getRosUintAlt(rclcpp::Node* node, const std::string& key, std::vector<U> &u) {
  rclcpp::Parameter parameter_variant;

  // Retrieve as generic Parameter to access type directly
  if (!node->get_parameter(key, parameter_variant)) {
    return false;
  }

  // Check if it's an integer array, and perform manual conversion if necessary
  if (parameter_variant.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    const std::vector<int64_t> &int_values = parameter_variant.as_integer_array();
    u.clear();
    for (const int64_t &value : int_values) {
      if (value < std::numeric_limits<U>::min() || value > std::numeric_limits<U>::max()) {
        throw std::runtime_error("Parameter " + key + " value out of bounds: " + std::to_string(value));
      }
      u.push_back(static_cast<U>(value));
    }
    return true;
  }

  // Handle other cases if required (like logging or returning an error)
  throw std::runtime_error("Parameter " + key + " is not of type integer array.");
}

static inline bool getRosBoolean(rclcpp::Node* node, const std::string &name)
{
  bool ret;
  if (!node->get_parameter(name, ret)) {
    // Note that if this is used after declare_parameter, this should never happen.
    throw std::runtime_error("Required parameter '" + name + "' has not been declared");
  }

  return ret;
}

template <typename T>
T declareRosIntParameter(rclcpp::Node* node, const std::string& name, int64_t default_value)
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.name = name;
  param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  param_desc.description = name;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = std::numeric_limits<T>::lowest();
  range.to_value = std::numeric_limits<T>::max();
  param_desc.integer_range.push_back(range);
  return node->declare_parameter(name, default_value, param_desc);
}

static inline bool isRosParameterSet(rclcpp::Node* node, const std::string& name)
{
  rclcpp::Parameter param;
  node->get_parameter(name, param);
  return param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
}

}  // namespace ublox_node

#endif  // UBLOX_GPS_UTILS_HPP
