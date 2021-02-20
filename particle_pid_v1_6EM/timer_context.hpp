/*
 * Copyright 2019-present Facebook, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <cassert>
#include <type_traits>

#undef abs
#include <chrono>

#include <unifex/config.hpp>
#include <unifex/get_stop_token.hpp>
#include <unifex/manual_lifetime.hpp>
#include <unifex/receiver_concepts.hpp>
#include <unifex/scheduler_concepts.hpp>
#include <unifex/stop_token_concepts.hpp>

#include <unifex/detail/prologue.hpp>

namespace arduino {
  constexpr long clockHz = 16'000'000;
  constexpr int timer_prescaler = 8;
  constexpr int timer_prescaler_mask = (0 << CS12) | (1 << CS11) | (0 << CS10);
  constexpr long clicks_per_second = clockHz / timer_prescaler;

  using clicks = std::chrono::duration<float, std::ratio<1, clicks_per_second>>;

  constexpr clicks max_clicks{65'535};                                   // 16bit clock

  struct steady_clock
  {
    typedef clicks 	                                         duration;
    typedef duration::rep                                    rep;
    typedef duration::period                                 period;
    typedef std::chrono::time_point<steady_clock, duration>  time_point;

    static constexpr bool is_steady = true;

    static time_point
    now() noexcept {
      return time_point{std::chrono::duration_cast<clicks>(std::chrono::microseconds(micros()))};
    }
  };
}

namespace unifex {
class timer_context;

namespace _timer_context {
  using clock_t = arduino::steady_clock;
  using time_point = typename clock_t::time_point;

  struct task_base {
    using execute_fn = void(task_base*) noexcept;

    explicit task_base(timer_context& context, execute_fn* execute) noexcept
      : context_(&context), execute_(execute) {}

    timer_context* const context_;
    task_base* next_ = nullptr;
    task_base** prevNextPtr_ = nullptr;
    execute_fn* execute_;
    time_point dueTime_;

    void execute() noexcept {
      execute_(this);
    }
  };

  class cancel_callback {
    task_base* const task_;
   public:
    explicit cancel_callback(task_base* task) noexcept
      : task_(task) {}

    void operator()() noexcept;
  };

  class scheduler;

  template <typename Duration>
  struct _schedule_after_sender {
    class type;
  };
  template <typename Duration>
  using schedule_after_sender = typename _schedule_after_sender<Duration>::type;

  template <typename Duration, typename Receiver>
  struct _after_op {
    class type;
  };
  template <typename Duration, typename Receiver>
  using after_operation =
      typename _after_op<Duration, remove_cvref_t<Receiver>>::type;

  template <typename Duration, typename Receiver>
  class _after_op<Duration, Receiver>::type final : task_base {
    friend schedule_after_sender<Duration>;

    template <typename Receiver2>
    explicit type(
        timer_context& context,
        Duration duration,
        Receiver2&& receiver)
        : task_base(context, &type::execute_impl),
          duration_(duration),
          receiver_((Receiver2 &&) receiver) {
      assert(context_ != nullptr);
    }

    static void execute_impl(task_base* t) noexcept {
      auto& self = *static_cast<type*>(t);
      self.cancelCallback_.destruct();
      if constexpr (is_stop_never_possible_v<
                        stop_token_type_t<Receiver&>>) {
        unifex::set_value(static_cast<Receiver&&>(self.receiver_));
      } else {
        if (get_stop_token(self.receiver_).stop_requested()) {
          unifex::set_done(static_cast<Receiver&&>(self.receiver_));
        } else {
          unifex::set_value(static_cast<Receiver&&>(self.receiver_));
        }
      }
    }

    Duration duration_;
    UNIFEX_NO_UNIQUE_ADDRESS Receiver receiver_;
    UNIFEX_NO_UNIQUE_ADDRESS manual_lifetime<typename stop_token_type_t<
        Receiver&>::template callback_type<cancel_callback>>
        cancelCallback_;

   public:
    void start() noexcept;
  };

  template <typename Duration>
  class _schedule_after_sender<Duration>::type {
    friend scheduler;

    explicit type(
        timer_context& context,
        Duration duration) noexcept
      : context_(&context), duration_(duration) {}

    timer_context* context_;
    Duration duration_;

   public:
    template <
        template <typename...> class Variant,
        template <typename...> class Tuple>
    using value_types = Variant<Tuple<>>;

    template <template <typename...> class Variant>
    using error_types = Variant<>;

    static constexpr bool sends_done = true;

    template <typename Receiver>
    after_operation<Duration, Receiver> connect(Receiver&& receiver) const {
      return after_operation<Duration, Receiver>{
          *context_, duration_, (Receiver &&) receiver};
    }
  };

  template <typename Receiver>
  struct _at_op {
    class type;
  };
  template <typename Receiver>
  using at_operation = typename _at_op<remove_cvref_t<Receiver>>::type;

  template <typename Receiver>
  class _at_op<Receiver>::type final : task_base {
    static void execute_impl(task_base* p) noexcept {
      auto& self = *static_cast<type*>(p);
      self.cancelCallback_.destruct();
      if constexpr (is_stop_never_possible_v<
                        stop_token_type_t<Receiver&>>) {
        unifex::set_value(static_cast<Receiver&&>(self.receiver_));
      } else {
        if (get_stop_token(self.receiver_).stop_requested()) {
          unifex::set_done(static_cast<Receiver&&>(self.receiver_));
        } else {
          unifex::set_value(static_cast<Receiver&&>(self.receiver_));
        }
      }
    }

    UNIFEX_NO_UNIQUE_ADDRESS Receiver receiver_;
    UNIFEX_NO_UNIQUE_ADDRESS manual_lifetime<typename stop_token_type_t<
        Receiver&>::template callback_type<cancel_callback>>
        cancelCallback_;

   public:
    template <typename Receiver2>
    explicit type(
        timer_context& scheduler,
        clock_t::time_point dueTime,
        Receiver2&& receiver)
        : task_base(scheduler, &type::execute_impl)
        , receiver_((Receiver2 &&) receiver) {
      this->dueTime_ = dueTime;
    }

    void start() noexcept;
  };

  class schedule_at_sender {
    friend scheduler;

    explicit schedule_at_sender(
        timer_context& context,
        time_point dueTime)
      : context_(&context), dueTime_(dueTime) {}

    timer_context* context_;
    time_point dueTime_;
   public:
    template <
        template <typename...> class Variant,
        template <typename...> class Tuple>
    using value_types = Variant<Tuple<>>;

    template <template <typename...> class Variant>
    using error_types = Variant<>;

    static constexpr bool sends_done = true;

    template <typename Receiver>
    at_operation<remove_cvref_t<Receiver>> connect(Receiver&& receiver) {
      return at_operation<remove_cvref_t<Receiver>>{
          *context_, dueTime_, (Receiver &&) receiver};
    }
  };

  class scheduler {
    friend timer_context;

    explicit scheduler(timer_context& context) noexcept
      : context_(&context) {}

    friend bool operator==(scheduler a, scheduler b) noexcept {
      return a.context_ == b.context_;
    }
    friend bool operator!=(scheduler a, scheduler b) noexcept {
      return a.context_ != b.context_;
    }

    timer_context* context_;
   public:
    template <typename Rep, typename Ratio>
    auto schedule_after(std::chrono::duration<Rep, Ratio> delay) const noexcept
        -> schedule_after_sender<std::chrono::duration<Rep, Ratio>> {
      return schedule_after_sender<std::chrono::duration<Rep, Ratio>>{
          *context_, delay};
    }

    auto schedule_at(clock_t::time_point dueTime) const noexcept {
      return schedule_at_sender{*context_, dueTime};
    }

    auto schedule() const noexcept {
      return schedule_after(std::chrono::milliseconds{0});
    }
  };
} // namespace _timer_context

class timer_context {
  using scheduler = _timer_context::scheduler;
  using task_base = _timer_context::task_base;
  using cancel_callback = _timer_context::cancel_callback;
  friend cancel_callback;
  friend scheduler;
  template <typename Duration, typename Receiver>
  friend struct _timer_context::_after_op;
  template <typename Receiver>
  friend struct _timer_context::_at_op;

  void enqueue(task_base* task) noexcept;

  // Head of a linked-list in ascending order of due-time.
  task_base* head_ = nullptr;
  bool stop_ = false;

 public:
  using clock_t = _timer_context::clock_t;
  using time_point = _timer_context::time_point;

  timer_context();
  ~timer_context();

  void run();

  scheduler get_scheduler() noexcept {
    return scheduler{*this};
  }

  friend scheduler tag_invoke(tag_t<unifex::get_scheduler>, const timer_context& self) noexcept {
    return const_cast<timer_context&>(self).get_scheduler();
  }
};

namespace _timer_context {
  template <typename Duration, typename Receiver>
  inline void _after_op<Duration, Receiver>::type::start() noexcept {
    this->dueTime_ = clock_t::now() + std::chrono::duration_cast<typename clock_t::duration>(duration_);
    cancelCallback_.construct(
        get_stop_token(receiver_), cancel_callback{this});
    context_->enqueue(this);
  }

  template <typename Receiver>
  inline void _at_op<Receiver>::type::start() noexcept {
    cancelCallback_.construct(
        get_stop_token(receiver_), cancel_callback{this});
    this->context_->enqueue(this);
  }
} // namespace _timer_context
} // namespace unifex

#include <unifex/detail/epilogue.hpp>
