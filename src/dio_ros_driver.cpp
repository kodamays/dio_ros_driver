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

  initPortsArray();

  // prepare publishers
  for (uint32_t i = 0; i < MAX_PORT_NUM; i++) {
    std::string topic_name = "/dio/din" + std::to_string(i);
    din_port_publisher_array_.at(i) = this->create_publisher<dio_ros_driver::msg::DIOPort>(topic_name, rclcpp::QoS(1));
  }

  for (uint32_t i = 0; i < MAX_PORT_NUM; i++) {
    if(din_array_list_[i] == true) {
      std::string topic_name = "/dio/din_array" + std::to_string(i);
      rclcpp::Publisher<dio_ros_driver::msg::DIOArray>::SharedPtr din_array;
      din_array = this->create_publisher<dio_ros_driver::msg::DIOArray>(topic_name, rclcpp::QoS(1));
      dio_din_arrays_.push_back(din_array);
    }
  }

  // prepare subscribers
  for (uint32_t i = 0; i < MAX_PORT_NUM; i++) {
    std::string topic_name = "/dio/dout" + std::to_string(i);
    std::function<void(std::shared_ptr<dio_ros_driver::msg::DIOPort>)> callback = std::bind(&DIO_ROSDriver::receiveWriteRequest, this, std::placeholders::_1, i);
    dout_port_subscriber_array_.at(i) = this->create_subscription<dio_ros_driver::msg::DIOPort>(topic_name,
                                                                                                rclcpp::QoS(1),
                                                                                                callback);
  }

  for (uint32_t i = 0; i < MAX_PORT_NUM; i++) {
    if(dout_array_list_[i] == true) {
      std::string topic_name = "/dio/dout_array" + std::to_string(i);
      rclcpp::Subscription<dio_ros_driver::msg::DIOArray>::SharedPtr dout_array;
      std::function<void(std::shared_ptr<dio_ros_driver::msg::DIOArray>)> callback = std::bind(&DIO_ROSDriver::receiveArrayWriteReqests, this, std::placeholders::_1, i);
      dout_array = this->create_subscription<dio_ros_driver::msg::DIOArray>(topic_name,
                                                                              rclcpp::QoS(1),
                                                                              callback);
      dio_dout_arrays_.push_back(dout_array);                                                                      
    }
  }

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
void DIO_ROSDriver::receiveArrayWriteReqests(const dio_ros_driver::msg::DIOArray::ConstSharedPtr &dout_array_topic, const uint32_t &array_id) {
  dout_arrays_user_update_[array_id].update_ = true;
  dout_arrays_user_update_[array_id].value_ = dout_array_topic;
}

/**
 * @brief convert read value to topic
 * read values from all DI ports and send them as respective topic to application node
 */
void DIO_ROSDriver::readDINPorts(void) {
  dio_ros_driver::msg::DIOPort din_port;
  for (uint32_t i = 0; i < din_accessor_->getNumOfPorts(); i++) {
    if (use_din_port_[i] == false) {
      int32_t read_value = din_accessor_->readPort(i);
      if (read_value >= 0) {
        din_port.value = static_cast<bool>(read_value);
        din_port_publisher_array_.at(i)->publish(din_port);
      }
    }
  }
  for (uint32_t i = 0; i < MAX_PORT_NUM; i++) {
    if (din_ports_arrays_[i].size() > 0) {
      dio_ros_driver::msg::DIOArray read_values;
      dio_ros_driver::msg::DIOPortValue value;
      bool end_din_ports_array = true;
      for (auto din_port_value : din_ports_arrays_[i]) {
        int32_t read_value = din_accessor_->readPort(din_port_value);
        if (read_value >= 0) {
          value.port = din_port_value;
          value.value = static_cast<bool>(read_value);
          read_values.values.push_back(value);
        } else {
          end_din_ports_array = false;
          break;
        }
      }
      if(end_din_ports_array == true && read_values.values.size() > 0) {
        dio_din_arrays_[i]->publish(read_values);
      }
    }
  }
}

/**
 * @brief write value to DO port
 * update value of DO port according to user's request.
 */
void DIO_ROSDriver::writeDOUTPorts(void) {
  for (uint32_t i = 0; i < dout_accessor_->getNumOfPorts(); i++) {
    if (use_dout_port_[i] == false) {
      if (dout_user_update_.at(i).update_ == true) {
        dout_accessor_->writePort(i, dout_user_update_.at(i).value_);
        dout_user_update_.at(i).update_ = false;
      }
    }
  }


  for (uint32_t i = 0; i < enable_dout_ports_array_.size(); i++) {
    if (enable_dout_ports_array_[i] == true && dout_arrays_user_update_[i].update_ == true) {
      for (auto dout_port_value : dout_arrays_user_update_[i].value_->values) {
        dout_accessor_->writePort(dout_port_value.port, dout_port_value.value);
      }
      dout_arrays_user_update_[i].update_ = false;
    }
  }
}

/**
 * @brief  DI, DO ports array initialization
 * initialize DI,DO ports array with parameters.
 */
void DIO_ROSDriver::initPortsArray(void) {
  for (uint32_t i = 0; i < MAX_PORT_NUM; i++) {
    std::string dout_ports_array_name = "dout_ports_array" + std::to_string(i);
    std::string din_ports_array_name  = "din_ports_array" + std::to_string(i);
    this->declare_parameter(dout_ports_array_name, std::vector<int64_t>{});
    this->declare_parameter(din_ports_array_name, std::vector<int64_t>{});
    auto dout_ports_array_list = this->get_parameter(dout_ports_array_name).as_integer_array();
    auto din_ports_array_list = this->get_parameter(din_ports_array_name).as_integer_array();
    if (dout_ports_array_list.size() > 0) {
      RCLCPP_INFO(this->get_logger(), "dout_ports_array%d: size:%d", i, static_cast<int>(dout_ports_array_list.size()));
      bool dout_array_set = true;
      std::vector<uint32_t> dout_ports;
      for (auto port : dout_ports_array_list) {
        uint32_t port_no = static_cast<uint32_t>(port);
        if( (use_dout_port_[port_no] == false) && (port_no < MAX_PORT_NUM) ) {
          dout_ports.push_back(port_no);
          use_dout_port_[port_no] = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "[dio_ros_driver] Failed to set dout_ports_array%d port %d.", i, static_cast<int>(port));
          dout_array_set = false;
          break;
        }
      }
      if (dout_array_set == true) {
        dout_ports_arrays_[i] = dout_ports;
        enable_dout_ports_array_[i] = true;
        dout_array_list_[i] = true;
      }
    }
    if (din_ports_array_list.size() > 0) {
      RCLCPP_INFO(this->get_logger(), "din_ports_array%d: size:%d", i, static_cast<int>(din_ports_array_list.size()));
      bool din_array_set = true;
      std::vector<uint32_t> din_ports;
      for (auto port : din_ports_array_list) {
        uint32_t port_no = static_cast<uint32_t>(port);
        if ( (use_din_port_[port_no] == false) && (port_no < MAX_PORT_NUM) ) {
          din_ports.push_back(port_no);
          use_din_port_[port_no] = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "[dio_ros_driver] Failed to set din_ports_array%d port %d.", i, static_cast<int>(port));
          din_array_set = false;
          break;
        }
      }
      if (din_array_set == true) {
        din_ports_arrays_[i] = din_ports;
        enable_din_ports_array_[i] = true;
        din_array_list_[i] = true;
      }
    }
  }
}
}  // namespace dio_ros_driver
