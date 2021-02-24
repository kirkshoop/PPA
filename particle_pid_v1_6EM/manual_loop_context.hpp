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

#include <unifex/blocking.hpp>
#include <unifex/get_stop_token.hpp>
#include <unifex/receiver_concepts.hpp>
#include <unifex/scheduler_concepts.hpp>
#include <unifex/stop_token_concepts.hpp>

#include "sequence.hpp"

#include <unifex/detail/prologue.hpp>

namespace unifex {
namespace _manual_loop_context {
class context;

struct task_base {
  using execute_fn = void(task_base*) noexcept;

  explicit task_base(execute_fn* execute) noexcept
  : execute_(execute)
  {}

  void execute() noexcept {
    this->execute_(this);
  }

  task_base* next_ = nullptr;
  execute_fn* execute_;
};

template <typename Receiver>
struct _op {
  class type;
  class unwinder;
};
template <typename Receiver>
using operation = typename _op<remove_cvref_t<Receiver>>::type;

template<typename Receiver>
struct _op<Receiver>::unwinder {
  using op_t = typename _op<Receiver>::type;
  op_t* op_;
  Receiver& get_receiver() {
    return op_->receiver_;
  } 
  template <typename Cpo, typename... Vn>
  friend void tag_invoke(unifex::tag_t<unifex::unwind>, unwinder& self, Cpo cpo, Vn&&... vn) noexcept {
    unifex::unwound(self.get_receiver(), cpo, (Vn&&) vn...);
  }
};

template <typename Receiver>
class _op<Receiver>::type final : task_base {
  using unwinder_t = typename _op<Receiver>::unwinder;
  friend class _op<Receiver>::unwinder;
  using stop_token_type = stop_token_type_t<Receiver&>;

 public:
  template <typename Receiver2>
  explicit type(Receiver2&& receiver, context* loop)
    : task_base(&type::execute_impl)
    , receiver_((Receiver2 &&) receiver)
    , loop_(loop) {}

  friend unwinder_t tag_invoke(unifex::tag_t<unifex::get_unwinder>, const type& self) noexcept {
    return unwinder_t{const_cast<type*>(&self)};
  }

  friend void tag_invoke(unifex::tag_t<unifex::step>, type&) noexcept {
  }

  void start() noexcept;

 private:
  static void execute_impl(task_base* t) noexcept {
    auto& self = *static_cast<type*>(t);
    if constexpr (is_stop_never_possible_v<stop_token_type>) {
      unifex::set_value(std::move(self.receiver_));
    } else {
      if (get_stop_token(self.receiver_).stop_requested()) {
        unifex::set_done(std::move(self.receiver_));
      } else {
        unifex::set_value(std::move(self.receiver_));
      }
    }
  }

  UNIFEX_NO_UNIQUE_ADDRESS Receiver receiver_;
  context* const loop_;
};

class context {
  template <class Receiver>
  friend struct _op;
 public:
  class scheduler {
    class schedule_task {
      friend constexpr blocking_kind tag_invoke(
          tag_t<blocking>,
          const schedule_task&) noexcept {
        return blocking_kind::never;
      }

     public:
      template <
          template <typename...> class Variant,
          template <typename...> class Tuple>
      using value_types = Variant<Tuple<>>;

      template <template <typename...> class Variant>
      using error_types = Variant<>;

      static constexpr bool sends_done = true;

      template <typename Receiver>
      operation<Receiver> connect(Receiver&& receiver) const& {
        return operation<Receiver>{(Receiver &&) receiver, loop_};
      }

    private:
      friend scheduler;

      explicit schedule_task(context* loop) noexcept
        : loop_(loop)
      {}

      context* const loop_;
    };

    friend context;

    explicit scheduler(context* loop) noexcept : loop_(loop) {}

   public:
    schedule_task schedule() const noexcept {
      return schedule_task{loop_};
    }

    friend bool operator==(scheduler a, scheduler b) noexcept {
      return a.loop_ == b.loop_;
    }
    friend bool operator!=(scheduler a, scheduler b) noexcept {
      return a.loop_ != b.loop_;
    }

   private:
    context* loop_;
  };

  scheduler get_scheduler() const noexcept {
    return scheduler{this};
  }

  friend scheduler tag_invoke(tag_t<unifex::get_scheduler>, const context& self) noexcept {
    return const_cast<context&>(self).get_scheduler();
  }

  void run_once();

  void stop();

 private:
  void enqueue(task_base* task);

  task_base* head_ = nullptr;
  task_base* tail_ = nullptr;
  bool stop_ = false;
};

template <typename Receiver>
inline void _op<Receiver>::type::start() noexcept {
  loop_->enqueue(this);
}

} // namespace _manual_loop_context

using manual_loop_context = _manual_loop_context::context;
} // namespace unifex

#include <unifex/detail/epilogue.hpp>
