/*
 * SPDX-FileCopyrightText: Bosch Rexroth AG
 *
 * SPDX-License-Identifier: MIT
 */
#include <iostream>
#include <source_location>

#include "callable.h"

/// @brief Constructor for the Callable class.
Callable::Callable(const std::shared_ptr<MinimalPublisher> &mp)
  : m_mp(mp)
{
  std::cout << std::source_location::current().function_name() << std::endl;
}

/// @brief Destructor for the Callable class.
Callable::~Callable()
{
  std::cout << std::source_location::current().function_name() << std::endl;
}

/*
  * @brief This method executes the specified scheduler event and
  * performs the corresponding actions based on the event type and phase.
*/
common::scheduler::SchedEventResponse Callable::execute(
    const common::scheduler::SchedEventType &eventType,
    const common::scheduler::SchedEventPhase &eventPhase,
    comm::datalayer::Variant &param)
{
  std::cout << std::source_location::current().function_name() << " event type: " << common::scheduler::getSchedEventTypeAsString(eventType) << " eventPhase " <<  common::scheduler::getSchedEventPhaseAsString(eventPhase) << std::endl;
  switch (eventType)
  {
  case common::scheduler::SchedEventType::SCHED_EVENT_TICK:
    return onEventTick(eventPhase, param);
  case common::scheduler::SchedEventType::SCHED_EVENT_SWITCH_TO_EXIT:
    return onSwitchToExit(eventPhase, param);

  case common::scheduler::SchedEventType::SCHED_EVENT_SWITCH_TO_CONFIG:
  case common::scheduler::SchedEventType::SCHED_EVENT_SWITCH_TO_RUN:
  case common::scheduler::SchedEventType::SCHED_EVENT_TASK_PROPERTIES_CHANGE:
  case common::scheduler::SchedEventType::SCHED_EVENT_GET_CURRENT_STATE:
  case common::scheduler::SchedEventType::SCHED_EVENT_SWITCH_TO_OPERATING:
  case common::scheduler::SchedEventType::SCHED_EVENT_SWITCH_TO_SETUP:
  case common::scheduler::SchedEventType::SCHED_EVENT_SWITCH_TO_SERVICE:
    break;
  }
  return common::scheduler::SchedEventResponse::SCHED_EVENT_RESP_OKAY;
}

//! The onEventTick method, called when a tick event occurs
common::scheduler::SchedEventResponse Callable::onEventTick(const common::scheduler::SchedEventPhase &eventPhase, comm::datalayer::Variant &param)
{
  std::cout << std::source_location::current().function_name() << " eventPhase " <<  common::scheduler::getSchedEventPhaseAsString(eventPhase) << std::endl;
  if (eventPhase == common::scheduler::SchedEventPhase::SCHED_EVENT_PHASE_NONE)
  {
    static u_int counter = 1701;

    m_mp->myprint(std::to_string(counter++));
  }

  return common::scheduler::SchedEventResponse::SCHED_EVENT_RESP_OKAY;
}

//! The onSwitchToExit method, called when a switch to exit event occurs
common::scheduler::SchedEventResponse Callable::onSwitchToExit(const common::scheduler::SchedEventPhase &eventPhase, comm::datalayer::Variant &param)
{
  std::cout << std::source_location::current().function_name() << " eventPhase " <<  common::scheduler::getSchedEventPhaseAsString(eventPhase) << std::endl;
  if (eventPhase == common::scheduler::SchedEventPhase::SCHED_EVENT_PHASE_END)
  {
    m_mp->myprint("EXIT");
  }

  return common::scheduler::SchedEventResponse::SCHED_EVENT_RESP_OKAY;
}
