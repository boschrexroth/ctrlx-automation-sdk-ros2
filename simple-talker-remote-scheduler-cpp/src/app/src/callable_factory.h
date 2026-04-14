/*
 * SPDX-FileCopyrightText: Bosch Rexroth AG
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <iostream>
#include <vector>

#include <common/scheduler/i_scheduler4.h>
#include <common/scheduler/callable_configurations_generated.h>

#include "minimal_publisher.h"
#include "callable.h"

/*
  * @brief The CallableFactory class is responsible for creating and managing callables for the remote scheduler.
  * It implements the ICallableFactory2 interface, which defines methods for creating and destroying callables,
  * as well as retrieving callable arguments and configurations.
*/
class CallableFactory
    : public common::scheduler::ICallableFactory2
{
public:
  CallableFactory(const std::shared_ptr<MinimalPublisher>& mp);
  virtual ~CallableFactory();

  // ICallableFactory
  virtual std::shared_ptr<common::scheduler::ICallable> createCallable(const comm::datalayer::Variant &param);
  virtual common::scheduler::SchedStatus destroyCallable(const std::shared_ptr<common::scheduler::ICallable> &callable);
  virtual common::scheduler::SchedStatus getCallableArguments(std::vector<std::string> &arguments);

  // ICallableFactory2
  virtual void getFactoryDescription(std::vector<std::string> &description) const;
  virtual void getCallableConfigurations(comm::datalayer::Variant &configurations) const;

private:
  std::vector<std::shared_ptr<Callable>>  m_callables;
  std::shared_ptr<MinimalPublisher>       m_mp;
};
