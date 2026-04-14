/*
 * SPDX-FileCopyrightText: Bosch Rexroth AG
 *
 * SPDX-License-Identifier: MIT
 */

#include <vector>
#include <iostream>
#include <source_location>

#include <common/scheduler/i_scheduler4.h>
#include <common/scheduler/callable_configurations_generated.h>

#include "callable_factory.h"

/*
  * @brief Constructor for the CallableFactory class.
*/
CallableFactory::CallableFactory(const std::shared_ptr<MinimalPublisher>& mp)
: m_mp(mp)
{
  std::cout << std::source_location::current().function_name() << std::endl;
}

/*
  * @brief Destructor for the CallableFactory class.
*/
CallableFactory::~CallableFactory()
{
  std::cout << std::source_location::current().function_name() << std::endl;
  m_mp = nullptr;
}

/*
  * @brief This method creates a new callable instance.
*/
std::shared_ptr<common::scheduler::ICallable> CallableFactory::createCallable(const comm::datalayer::Variant &param)
{
  std::cout << std::source_location::current().function_name() << std::endl;

  auto callable = std::make_shared<Callable>(m_mp);
  m_callables.push_back(callable);
  return callable;
}

/*
  * @brief This method destroys the specified callable and removes it from the list of callables.
*/
common::scheduler::SchedStatus CallableFactory::destroyCallable(const std::shared_ptr<common::scheduler::ICallable> &callable)
{
  std::cout << std::source_location::current().function_name() << std::endl;

  auto iterCallable = m_callables.begin();
  while (iterCallable != m_callables.end())
  {
    if (*iterCallable == callable)
    {
      m_callables.erase(iterCallable);
      break;
    }
    ++iterCallable;
  }
  return common::scheduler::SchedStatus::SCHED_S_OK;
}

/*
  * @brief This method retrieves the arguments for the callable.
*/
common::scheduler::SchedStatus CallableFactory::getCallableArguments(std::vector<std::string> &arguments)
{
  std::cout << std::source_location::current().function_name() << std::endl;

  return common::scheduler::SchedStatus::SCHED_S_OK;
}

// ICallableFactory2
void CallableFactory::getFactoryDescription(std::vector<std::string> &description) const
{
  std::cout << std::source_location::current().function_name() << std::endl;

  description.push_back("@author Bosch Rexroth AG");
  description.push_back("@origin Remote Scheduler App");
  description.push_back("@description This callable factory creates a relationship between the ROS system and the remote scheduler interface.");
}

/*
  * @brief This method defines the callable configuration for the remote scheduler.
  * In this example, a single callable configuration is defined,
  * which is based on a cyclic task with a cycle time of 1000 ms.
*/
void CallableFactory::getCallableConfigurations(comm::datalayer::Variant &configurations) const
{
  std::cout << std::source_location::current().function_name() << std::endl;

  /**/
  flatbuffers::FlatBufferBuilder builder;

  // define task specs
  auto fbsTaskSpec = common::scheduler::fbs::CreateTaskSpecsDirect(builder,
                                                                   "RosTask", // Max. 16 characters, '-' NOT allowed
                                                                   common::scheduler::TASK_PRIORITY_RANGE_MID,
                                                                   "cyclic/ms/1000");

  auto config = common::scheduler::fbs::CreateCallableConfigurationDirect(builder, "RosCallable", 0, nullptr, common::scheduler::fbs::CallableWdgConfig_WDG_NONE, fbsTaskSpec);
  auto configs = std::vector<flatbuffers::Offset<common::scheduler::fbs::CallableConfiguration>>({config});
  builder.Finish(common::scheduler::fbs::CreateCallableConfigurationsDirect(builder, &configs));
  configurations.copyFlatbuffers(builder);
}
