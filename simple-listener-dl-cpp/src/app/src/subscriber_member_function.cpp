// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <filesystem>
#include <csignal>
#include <thread>
#include <functional>
#include <memory>

#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"

#include "ctrlx_datalayer_helper.h"

using std::placeholders::_1;


using comm::datalayer::IProviderNode;

// Basic class Provider node interface for providing data to the system
class MyProviderNode: public IProviderNode
{
private:
  comm::datalayer::Variant m_data;

public:
  MyProviderNode(comm::datalayer::Variant data)
    : m_data(data)
  {};
  void setString(const std::string& input)
  { 
    m_data.setValue(input);

  }
  virtual ~MyProviderNode() override {};

  // Create function of an object. Function will be called whenever a object should be created.
  virtual void onCreate(const std::string& address, const comm::datalayer::Variant* data, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }

  // Read function of a node. Function will be called whenever a node should be read.
  virtual void onRead(const std::string& address, const comm::datalayer::Variant* data, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    comm::datalayer::Variant dataRead;
    dataRead = m_data;
    callback(comm::datalayer::DlResult::DL_OK, &dataRead);
  }

  // Write function of a node. Function will be called whenever a node should be written.
  virtual void onWrite(const std::string& address, const comm::datalayer::Variant* data, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    std::cout << "INFO onWrite " <<  address << std::endl;
    
    if (data->getType() != m_data.getType())
    {
      callback(comm::datalayer::DlResult::DL_TYPE_MISMATCH, nullptr);
    }

    m_data = *data;
    callback(comm::datalayer::DlResult::DL_OK, data);
  }

  // Remove function for an object. Function will be called whenever a object should be removed.
  virtual void onRemove(const std::string& address, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }

  // Browse function of a node. Function will be called to determine children of a node.
  virtual void onBrowse(const std::string& address, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }

  // Read function of metadata of an object. Function will be called whenever a node should be written.
  virtual void onMetadata(const std::string& address, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    // Keep this comment! Can be used as sample creating metadata programmatically.
    // callback(comm::datalayer::DlResult::DL_OK, &_metaData);

    // Take metadata from metadata.mddb
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }
};


class MinimalSubscriber : public rclcpp::Node
{
  comm::datalayer::Variant m_myString;
  comm::datalayer::DatalayerSystem m_datalayerSystem;
  std::unique_ptr<comm::datalayer::IProvider> m_provider;
  std::unique_ptr<MyProviderNode> m_pippo;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;

public:

  MinimalSubscriber()
  : Node("minimal_subscriber")
  , m_pippo(std::make_unique<MyProviderNode>(MyProviderNode(m_myString)))
  { 
    m_subscription = this->create_subscription<std_msgs::msg::String>(
      "ros2_simple_talker_cpp", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      
    // Starts the ctrlX Data Layer system without a new broker because one broker is already running on ctrlX CORE  
    m_datalayerSystem.start(false);
    m_provider.reset(getProvider(m_datalayerSystem)); // ctrlX CORE (virtual)
    m_pippo->setString("No Message yet");
    std::cout << "INFO Register node 'ros/listenercpp/mymessage'  " << std::endl;
    comm::datalayer::DlResult result = m_provider->registerNode("ros/listenercpp/mymessage",m_pippo.get());
    if (STATUS_FAILED(result))
    {
      std::cout << "WARN Register node 'sdk-cpp-registernode/myString' failed with: " << result.toString() << std::endl;
    }
  }

  ~MinimalSubscriber() 
  {
    std::cout << __func__ << std::endl;    
    m_provider->stop();
    m_provider = nullptr;
    m_datalayerSystem.stop();
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    m_pippo->setString( msg.data.c_str() );
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  {
    std::shared_ptr<MinimalSubscriber> node = std::make_shared<MinimalSubscriber>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
