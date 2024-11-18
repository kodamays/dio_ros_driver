/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @package dio_ros_dirver
 * @file dio_ros_dirver.cpp
 * @brief DIO ROS Driver class
 * @author Takayuki AKAMINE
 */


#include <cstring>
#include <iostream>
#include <cstdlib>
#include <chrono>

#include "dio_ros_driver/dio_ros_driver.hpp"

namespace dio_ros_driver {
/**
 * @brief Constructor of DIO_ROSDriver
 * Prepare ROS specific variable and initialize variable
 * @param nh node handler
 * @param pnh private node handler
 */
DIO_ROSDriver::DIO_ROSDriver(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options),
      din_port_publisher_array_(),
      dout_port_subscriber_array_(),
      din_accessor_(nullptr),
      dout_accessor_(nullptr),
      dio_diag_updater_(nullptr),
      access_frequency_(declare_parameter<int64_t>("access_frequency", 1)),
      chip_name_(declare_parameter<std::string>("chip_name", "gpiochip0")),
      din_port_offset_(),
      din_value_inverse_(declare_parameter<bool>("din_value_inverse", false)),
      dout_port_offset_(),
      dout_value_inverse_(declare_parameter<bool>("dout_value_inverse", false)),
      dout_default_value_(declare_parameter<bool>("dout_default_value", false)),
      dout_user_update_(),
      dio_chip_(nullptr) {

  // read parameter
  this->declare_parameter("din_port_offset", std::vector<int64_t>{});
  this->declare_parameter("dout_port_offset", std::vector<int64_t>{});

  auto tmp_din_offset_list = this->get_parameter("din_port_offset").as_integer_array();
  for (auto tmp_port_offset : tmp_din_offset_list) {
    din_port_offset_.push_back(static_cast<uint32_t>(tmp_port_offset));
  }
  
  auto tmp_dout_offset_list = this->get_parameter("dout_port_offset").as_integer_array();
  for (auto tmp_port_offset : tmp_dout_offset_list) {
    dout_port_offset_.push_back(static_cast<uint32_t>(tmp_port_offset));
  }


  // prepare publishers
  for (uint32_t i = 0; i < MAX_PORT_NUM; i++) {
    std::string topic_name = "/dio/din" + std::to_string(i);
    din_port_publisher_array_.at(i) = this->create_publisher<dio_ros_driver::msg::DIOPort>(topic_name, rclcpp::QoS(1));
  }

  din_port_array_publisher_ = this->create_publisher<dio_ros_driver::msg::DIOArray>("/dio/din_array", rclcpp::QoS(1));

  // prepare subscribers
  for (uint32_t i = 0; i < MAX_PORT_NUM; i++) {
    std::string topic_name = "/dio/dout" + std::to_string(i);
    std::function<void(std::shared_ptr<dio_ros_driver::msg::DIOPort>)> callback = std::bind(&DIO_ROSDriver::receiveWriteRequest, this, std::placeholders::_1, i);
    dout_port_subscriber_array_.at(i) = this->create_subscription<dio_ros_driver::msg::DIOPort>(topic_name,
                                                                                                rclcpp::QoS(1),
                                                                                                callback);
  }

  dout_port_array_subscriber_ = this->create_subscription<dio_ros_driver::msg::DIOArray>("/dio/dout_array",
                                                                                        rclcpp::QoS(1),
                                                                                        std::bind(&DIO_ROSDriver::receiveArrayWriteRequest, this, std::placeholders::_1));
  

  // create walltimer callback
  std::chrono::milliseconds update_cycle = std::chrono::milliseconds(1000U/static_cast<uint32_t>(access_frequency_));
  dio_update_timer_ = this->create_wall_timer(update_cycle,
                                              std::bind(&DIO_ROSDriver::update, this));

  // initialize accessors and diagnostic updater.
  din_accessor_ = std::make_shared<DINAccessor>();
  dout_accessor_ = std::make_shared<DOUTAccessor>();
}

/**
 * @brief DIO Accessor Initialization.
 * initialization to access dio chip and add ports to access.
 * @retval 0 always 0, but update status topic if any error occur
 */
int DIO_ROSDriver::init(void) {
  dio_chip_ = gpiod_chip_open_by_name(chip_name_.c_str());
  din_accessor_->initialize(dio_chip_, din_value_inverse_);
  dout_accessor_->initialize(dio_chip_, dout_value_inverse_, dout_default_value_);

  addAccessorPorts(din_port_offset_, din_accessor_);
  addAccessorPorts(dout_port_offset_, dout_accessor_);
  dio_diag_updater_ = std::make_shared<DIO_DiagnosticUpdater>(shared_from_this(), din_accessor_, dout_accessor_);

  return 0;
}

/**
 * @brief main routine of this node
 * update DO port and send topic periodically based on given access_frequency
 */
void DIO_ROSDriver::update(void) {

  // read data from DIN ports.
  readDINPorts();
  // write data to DOUT ports.
  writeDOUTPorts();
  // publish DIN ports value.
  publishDINPortValue();
  // activate diag updater.
  dio_diag_updater_->force_update();
}

/**
 * @brief terminate processing
 * After receiving SIGTERM signal, this method will be called.
 * @param signal_id signal id which is expected by SIGTERM.
 */
void DIO_ROSDriver::terminate(int signal_id) {
  int32_t exit_status = 0;

  // check if DIO chip opened
  if (dio_chip_ == nullptr) {
    // do not need to close chip
    exit_status = 0;
    std::exit(exit_status);
  }

  // check if any occurs at ports
  if (dout_accessor_->getNumOfPorts() == 0) {
    // only close chip.
    exit_status = 0;
    goto CLOSE_DIO_CHIP;
  }

  if (dout_accessor_->resetAllPorts() != 0) {
    // reset all dout ports.
    exit_status = -1;
    dio_diag_updater_->force_update();
    goto CLOSE_DIO_CHIP;
  }
  // release ports
  dout_accessor_->releaseAllPorts();
  din_accessor_->releaseAllPorts();
  exit_status = 0;

CLOSE_DIO_CHIP:
  gpiod_chip_close(dio_chip_);

  std::exit(exit_status);
}

/// Private methods.
/**
 * @brief add access port based on given parameter (after load .yaml)
 * @param[in] param_name ROS parameter name to list port id and offset
 * @param[out] dio_accessor dio accessor to update
 */
void DIO_ROSDriver::addAccessorPorts(const std::vector<uint32_t> offset_array, std::shared_ptr<DIO_AccessorBase> dio_accessor) {
  // get port number array.
   for (const auto &offset : offset_array) {
     dio_accessor->addPort(offset);
  }
  return;
}

/**
 * @brief callback function to hold and notify write request
 * @param[in] dout_topic topic to update port from application
 * @param[in] port_id targeted port number.
 */
void DIO_ROSDriver::receiveWriteRequest(const dio_ros_driver::msg::DIOPort::SharedPtr &dout_topic, const uint32_t &port_id) {
  dout_update &targeted_dout_update = dout_user_update_.at(port_id);
  targeted_dout_update.update_ = true;
  targeted_dout_update.value_ = dout_topic->value;
}

/**
 * @brief callback function to hold and notify write request
 * @param[in] dout_array_topic topic to update ports from application
 * @param[in] array_id  targeted array_id number.
 */
void DIO_ROSDriver::receiveArrayWriteRequest(const dio_ros_driver::msg::DIOArray::ConstSharedPtr &dout_array_topic) {
  for (auto dout_port_data : dout_array_topic->values) {
    if (dout_port_data.port < MAX_PORT_NUM) {
      dout_update &targeted_dout_update = dout_user_update_.at(dout_port_data.port);
      targeted_dout_update.update_ = true;
      targeted_dout_update.value_ = dout_port_data.value;
    }
  }
}

/**
 * @brief convert read value to topic
 * read values from all DI ports.
 */
void DIO_ROSDriver::readDINPorts(void) {
  for (uint32_t i = 0; i < din_accessor_->getNumOfPorts(); i++) {
    read_din_values_[i] = din_accessor_->readPort(i);
  }
}

/**
 * @brief write value to DO port
 * update value of DO port according to user's request.
 */
void DIO_ROSDriver::writeDOUTPorts(void) {
  for (uint32_t i = 0; i < dout_accessor_->getNumOfPorts(); i++) {
    if (dout_user_update_.at(i).update_ == true) {
      dout_accessor_->writePort(i, dout_user_update_.at(i).value_);
      dout_user_update_.at(i).update_ = false;
    }
  }
}

/**
 * @brief publish DI port value
 * send read values from all DIO ports as respective topic and array topic to application node
 */
void DIO_ROSDriver::publishDINPortValue(void) {
  dio_ros_driver::msg::DIOPort din_port;
  dio_ros_driver::msg::DIOArray port_array_values;
  dio_ros_driver::msg::DIOPortValue port_value;
  for (uint32_t i = 0; i < din_accessor_->getNumOfPorts(); i++) {
    if (read_din_values_[i] >= 0) {
      din_port.value = static_cast<bool>(read_din_values_[i]);
      din_port_publisher_array_.at(i)->publish(din_port);
      port_value.port = static_cast<u_int8_t>(i);
      port_value.value = static_cast<bool>(read_din_values_[i]);
      port_array_values.values.push_back(port_value);
    }
  }
  port_array_values.stamp = get_clock()->now();
  din_port_array_publisher_->publish(port_array_values);
}

}  // namespace dio_ros_driver
