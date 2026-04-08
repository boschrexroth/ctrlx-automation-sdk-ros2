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
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <filesystem>
#include <csignal>
#include <thread>
#include <iostream>
#include <source_location>

#include <stdio.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <comm/datalayer/datalayer.h>
#include <comm/datalayer/datalayer_system.h>
#include <common/scheduler/scheduler_remote_system.h>

#include "minimal_publisher.h"
#include "callable_factory.h"

using namespace std::chrono_literals;

/*
  Wait until the framework is ready for operation
*/
static bool wait_on_framework_started(comm::datalayer::DatalayerSystem& dl, const std::string &ipcPath)
{
  std::cout << std::source_location::current().function_name() << std::endl;

  std::shared_ptr<comm::datalayer::IClient> client;
  client.reset(dl.factory()->createClient(ipcPath.c_str()));

  for (int i = 0; i < 10; i++)
  {
    if (!client->isConnected())
    {
      std::cout << std::source_location::current().function_name() << " Client: is not connected" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }

    comm::datalayer::Variant value;
    auto result = client->readSync("framework/events/framework-started", &value);
    if (result != DL_OK)
    {
      std::cout << std::source_location::current().function_name() << " Client: 'framework/events/framework-started' error: " << result.toString() << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }

    if ((uint32_t)value == 1)
    {
      return true;
    }

    std::cout << std::source_location::current().function_name() << " Client: framework is not started " << (uint32_t)value << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  return false;
}

/*
  Register callable factory
*/
static bool register_factory(const std::shared_ptr<common::scheduler::IScheduler3>& scheduler, const std::shared_ptr<common::scheduler::ICallableFactory2>& factory) {
  std::cout << std::source_location::current().function_name() << std::endl;

  for (int attempt = 1; attempt <= 5; ++attempt)
  {
    auto result = scheduler->registerCallableFactory(factory, "remote-ros-factory");

    if (result == common::scheduler::SchedStatus::SCHED_S_OK)
    {
      return true;
    }
    std::cout << std::source_location::current().function_name() << " Registration failed, error:" << common::scheduler::getSchedStatusAsString(result) << " waiting for AutomationCore to be available..." << attempt << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  return false;
}

/*
  Main function
*/
int main(int argc, char * argv[])
{
   // Initialize ROS 2
  rclcpp::init(argc, argv);

  {
    std::shared_ptr<MinimalPublisher> pippo = std::make_shared<MinimalPublisher>();

    std::cout << "INFO Starting ctrlX Data Layer system (without broker)" << std::endl;
    comm::datalayer::DatalayerSystem datalayerSystem;
    datalayerSystem.start(false);

    if (!wait_on_framework_started(datalayerSystem, DL_IPC))
    {
      std::cout << "INFO Starting error: " << common::scheduler::getSchedStatusAsString(common::scheduler::SchedStatus::SCHED_E_NOT_INITIALIZED) << std::endl;
      return 1;
    }

    {
      std::shared_ptr<common::scheduler::IScheduler3> scheduler =  std::make_shared<common::scheduler::SchedulerRemoteSystem>(datalayerSystem.factory5());
      std::shared_ptr<common::scheduler::ICallableFactory2> factory = std::make_shared<CallableFactory>(pippo);
      if (!register_factory(scheduler, factory)) {
        std::cout << "INFO Register problem: 'sudo snap connect ros2-simple-talker-rs-cpp:process-control'" << std::endl;
        return 2;
      }

      rclcpp::spin(pippo);

      std::cout << "INFO >App shutdown and unregister callback factory" << std::endl;
      scheduler->unregisterCallableFactory(factory, true);

    }
    // Workaround: Give some time to the scheduler to process the unregistration before stopping the datalayer system
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::cout << "INFO Stop ctrlX Data Layer system" << std::endl;
    datalayerSystem.stop();
  }

  rclcpp::shutdown();
  return 1; // We exit because an error happend
}
