/*
 * SPDX-FileCopyrightText: Bosch Rexroth AG
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once
#include <common/scheduler/i_scheduler4.h>
#include "minimal_publisher.h"

/*
  * @brief The Callable class is responsible for executing tasks for the remote scheduler.
  * It implements the ICallable interface, which defines the execute method for handling scheduler events.
*/
class Callable
    : public common::scheduler::ICallable
{

public:
  Callable(const std::shared_ptr<MinimalPublisher> &mp);
  virtual ~Callable();

  virtual common::scheduler::SchedEventResponse execute(const common::scheduler::SchedEventType &eventType,
                                                        const common::scheduler::SchedEventPhase &eventPhase,
                                                        comm::datalayer::Variant &param);

private:
  common::scheduler::SchedEventResponse onEventTick(const common::scheduler::SchedEventPhase &eventPhase, comm::datalayer::Variant &param);
  common::scheduler::SchedEventResponse onSwitchToExit(const common::scheduler::SchedEventPhase &eventPhase, comm::datalayer::Variant &param);

  std::shared_ptr<MinimalPublisher> m_mp;
};
