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

#include <Arduino.h>
#include "stdlibpatch.h"
#include "interrupt_context.hpp"

namespace unifex {

//static 
interrupt_context* interrupt_context::self_ = nullptr;

interrupt_context::interrupt_context()
{
  assert(self_ == nullptr);
  self_ = this;
}

interrupt_context::~interrupt_context() {
  stop_ = true;
  assert(head_ == nullptr);
  self_ = nullptr;
}

void interrupt_context::enqueue(task_base* task) noexcept {
//   Serial.println('e');
  noInterrupts();                          // disable all interrupts while interrupt parameters are changing
  // Insert at the head of the queue.
  task->next_ = head_;
  task->prevNextPtr_ = &head_;
  if (head_ != nullptr) {
    head_->prevNextPtr_ = &task->next_;
  }
  head_ = task;
  interrupts();                            // enable all interrupts
//   Serial.println("~e");
}

// static
void interrupt_context::run() {

//   Serial.println('i');

//   noInterrupts();                          // disable all interrupts while interrupt parameters are changing
  auto cursor_ = self_->head_;
  while (!self_->stop_ && cursor_ != nullptr) {
      auto task_ = cursor_;
      cursor_ = cursor_->next_;
    //   interrupts();                            // enable all interrupts
      task_->execute();
    //   noInterrupts();                          // disable all interrupts while interrupt parameters are changing
  }

//   interrupts();                            // enable all interrupts

//   Serial.println("~i");
}

void _interrupt_context::cancel_callback::operator()() noexcept {
  if (task_->prevNextPtr_ != nullptr) {
    // Task is still in the queue, dequeue and requeue it.
    
    noInterrupts();                          // disable all interrupts while interrupt parameters are changing

    // Remove from the queue.
    *task_->prevNextPtr_ = task_->next_;
    if (task_->next_ != nullptr) {
    task_->next_->prevNextPtr_ = task_->prevNextPtr_;
    }
    task_->prevNextPtr_ = nullptr;

    interrupts();                            // enable all interrupts
  }
}

void interrupt_context::attach(std::uint8_t pin, std::uint8_t mode) {
  attachInterrupt(digitalPinToInterrupt(pin), run, mode);
}

} // namespace unifex
