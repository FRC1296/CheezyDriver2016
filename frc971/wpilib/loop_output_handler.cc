#include "frc971/wpilib/loop_output_handler.h"

#include <sys/timerfd.h>

#include <thread>
#include <functional>

#include "aos/linux_code/init.h"
#include "aos/common/messages/robot_state.q.h"

namespace frc971 {
namespace wpilib {

LoopOutputHandler::LoopOutputHandler(const ::aos::time::Time &timeout)
    : watchdog_(this, timeout) {}

void LoopOutputHandler::operator()() {
  ::std::thread watchdog_thread(::std::ref(watchdog_));
  ::aos::SetCurrentThreadName("OutputHandler");

  ::aos::SetCurrentThreadRealtimePriority(30);
  while (run_) {
    no_joystick_state_.Print();
    fake_joystick_state_.Print();
    Read();
    ::aos::joystick_state.FetchLatest();
    if (!::aos::joystick_state.get()) {
      LOG_INTERVAL(no_joystick_state_);
      continue;
    }
    if (::aos::joystick_state->fake) {
      LOG_INTERVAL(fake_joystick_state_);
      continue;
    }

    watchdog_.Reset();
    Write();
  }

  Stop();

  watchdog_.Quit();
  watchdog_thread.join();
}

LoopOutputHandler::Watchdog::Watchdog(LoopOutputHandler *handler,
                                      const ::aos::time::Time &timeout)
    : handler_(handler),
      timeout_(timeout),
      timerfd_(timerfd_create(::aos::time::Time::kDefaultClock, 0)) {
  if (timerfd_.get() == -1) {
    PLOG(FATAL, "timerfd_create(Time::kDefaultClock, 0)");
  }
}

void LoopOutputHandler::Watchdog::operator()() {
  ::aos::SetCurrentThreadRealtimePriority(35);
  ::aos::SetCurrentThreadName("OutputWatchdog");
  uint8_t buf[8];
  while (run_) {
    PCHECK(read(timerfd_.get(), buf, sizeof(buf)));
    handler_->Stop();
  }
}

void LoopOutputHandler::Watchdog::Reset() {
  itimerspec value = itimerspec();
  value.it_value = timeout_.ToTimespec();
  PCHECK(timerfd_settime(timerfd_.get(), 0, &value, nullptr));
}

}  // namespace wpilib
}  // namespace frc971
