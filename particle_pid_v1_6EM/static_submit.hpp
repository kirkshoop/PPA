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
#include <unifex/manual_lifetime.hpp>
#include <unifex/receiver_concepts.hpp>
#include <unifex/scheduler_concepts.hpp>

#include "sequence.hpp"

#include <unifex/detail/prologue.hpp>

namespace unifex {

struct static_submit_sender_complete {
  using type = static_submit_sender_complete;
  using destruct_fn = void() noexcept;

  destruct_fn* destruct_;

  template <typename Index, typename... Values>
  friend void tag_invoke(unifex::tag_t<unifex::set_index>, const type&, Index&&, Values&&...) noexcept {
  }
  friend void tag_invoke(unifex::tag_t<unifex::set_end>, type&& self) noexcept {
    self.destruct_();
  }

  template<typename... Vn>
  friend void tag_invoke(unifex::tag_t<unifex::set_value>, type&& self, Vn&&...) {
    self.destruct_();
  }

  template<typename Error>
  friend void tag_invoke(unifex::tag_t<unifex::set_error>, type&& self, Error&& error) noexcept {
    self.destruct_();
  }
  friend void tag_invoke(unifex::tag_t<unifex::set_done>, type&& self) noexcept {
    self.destruct_();
  }
  template <typename... UVn>
  friend void tag_invoke(unifex::tag_t<unifex::unwound>, const type&, unifex::tag_t<unifex::set_index>, UVn&&...) noexcept {
    // non-terminating
  }
  template <typename Cpo, typename... UVn>
  friend void tag_invoke(unifex::tag_t<unifex::unwound>, const type& self, Cpo, UVn&&...) noexcept {
    // terminating
  }

};

template<typename Sender>
void static_submit(Sender sender) {
  using sender_complete_t = static_submit_sender_complete;
  using sender_operation_t = unifex::connect_result_t<Sender, sender_complete_t>;
  using sender_op_storage_t = unifex::manual_lifetime<sender_operation_t>;

  static bool started_ = false;
  static sender_op_storage_t sender_op_;
  struct destruct_fn {
    ~destruct_fn() {
    //   if (started_) {
    //     Serial.println("static_submit RUNNING");
    //   }
      // make sure that main does not exit before the sender is stopped.
      assert(!started_);
    }
    static void destruct() noexcept {
      if (started_) {
        sender_op_.destruct();
        started_ = false;
      }
    }
  };
  static destruct_fn assert_state_;

//   if (started_) {
//     Serial.println("static_submit ALREADY RUNNING");
//   }
//   assert(!started_);

  sender_complete_t senderComplete{&destruct_fn::destruct};

  auto& senderOp = sender_op_.construct_from([&]() noexcept {
    return unifex::connect(std::move(sender), std::move(senderComplete));
  });

  started_ = true;
  unifex::start(senderOp);
}

} // namespace unifex

#include <unifex/detail/epilogue.hpp>
