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

#include <unifex/config.hpp>
#include <unifex/get_stop_token.hpp>
#include <unifex/manual_lifetime.hpp>
#include <unifex/receiver_concepts.hpp>
#include <unifex/scheduler_concepts.hpp>
#include <unifex/stop_token_concepts.hpp>

#include "sequence.hpp"

#include <unifex/detail/prologue.hpp>

namespace unifex {
class interrupt_context;

namespace _interrupt_context {

  struct task_base {
    using execute_fn = void(task_base*) noexcept;

    explicit task_base(interrupt_context& context, execute_fn* execute) noexcept
      : context_(&context), execute_(execute) {}

    interrupt_context* const context_;
    task_base* next_ = nullptr;
    task_base** prevNextPtr_ = nullptr;
    execute_fn* execute_;
    std::ptrdiff_t index_ = 0;

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

  template <typename Receiver>
  struct _interrupts_op {
    class type;
    class unwinder;
  };
  template <typename Receiver>
  using interrupts_operation = typename _interrupts_op<remove_cvref_t<Receiver>>::type;

  template<typename Receiver>
  struct _interrupts_op<Receiver>::unwinder {
    using op_t = typename _interrupts_op<Receiver>::type;
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
  class _interrupts_op<Receiver>::type final : task_base {
    using unwinder_t = typename _interrupts_op<Receiver>::unwinder;
    friend class _interrupts_op<Receiver>::unwinder;
    static void execute_impl(task_base* p) noexcept {
      auto& self = *static_cast<type*>(p);
      self.cancelCallback_.destruct();
      if constexpr (is_stop_never_possible_v<
                        stop_token_type_t<Receiver&>>) {
        unifex::set_index(static_cast<Receiver&&>(self.receiver_), p->index_++);
      } else {
        if (get_stop_token(self.receiver_).stop_requested()) {
          unifex::set_done(static_cast<Receiver&&>(self.receiver_));
        } else {
          unifex::set_index(static_cast<Receiver&&>(self.receiver_), p->index_++);
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
        interrupt_context& scheduler,
        Receiver2&& receiver)
        : task_base(scheduler, &type::execute_impl)
        , receiver_((Receiver2 &&) receiver) {
    }

    friend unwinder_t tag_invoke(unifex::tag_t<unifex::get_unwinder>, const type& self) noexcept {
      return unwinder_t{const_cast<type*>(&self)};
    }

    friend void tag_invoke(unifex::tag_t<unifex::step>, type&) noexcept {
    }

    void start() noexcept;
  };

  class interrupts_sender {
    friend class unifex::interrupt_context;

    explicit interrupts_sender(
        const interrupt_context& context)
      : context_(&context) {}

    mutable interrupt_context* context_;
   public:
    template <
        template <typename...> class Variant,
        template <typename...> class Tuple>
    using value_types = Variant<Tuple<std::ptrdiff_t>>;

    template <template <typename...> class Variant>
    using error_types = Variant<>;

    static constexpr bool sends_done = true;

    template <typename Receiver>
    interrupts_operation<remove_cvref_t<Receiver>> connect(Receiver&& receiver) {
      return interrupts_operation<remove_cvref_t<Receiver>>{
          *context_, (Receiver &&) receiver};
    }
  };
} // namespace _interrupt_context

class interrupt_context {
  using task_base = _interrupt_context::task_base;
  using cancel_callback = _interrupt_context::cancel_callback;
  friend cancel_callback;
  template <typename Receiver>
  friend struct _interrupt_context::_interrupts_op;

  void enqueue(task_base* task) noexcept;

  // Head of a linked-list in ascending order of due-time.
  task_base* head_ = nullptr;
  bool stop_ = false;

  static void run();
  static interrupt_context* self_;
 public:

  interrupt_context();
  ~interrupt_context();

  void attach(std::uint8_t pin, std::uint8_t mode);

  auto all_interrupts() const noexcept {
    return _interrupt_context::interrupts_sender{*this};
  }
};

namespace _interrupt_context {
  template <typename Receiver>
  inline void _interrupts_op<Receiver>::type::start() noexcept {
    cancelCallback_.construct(
        get_stop_token(receiver_), cancel_callback{this});
    this->context_->enqueue(this);
  }
} // namespace _interrupt_context
} // namespace unifex

#include <unifex/detail/epilogue.hpp>
