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
#include "stdlibpatch.h"
#include "manual_loop_context.hpp"

namespace unifex {
namespace _manual_loop_context {

void context::run_once() {
  if (head_ == nullptr || stop_) return;
  auto* task = head_;
  head_ = task->next_;
  if (head_ == nullptr) {
    tail_ = nullptr;
  }
  task->execute();
}

void context::stop() {
  stop_ = true;
}

void context::enqueue(task_base* task) {
  if (head_ == nullptr) {
    head_ = task;
  } else {
    tail_->next_ = task;
  }
  tail_ = task;
  task->next_ = nullptr;
}

} // _manual_loop_context
} // unifex
