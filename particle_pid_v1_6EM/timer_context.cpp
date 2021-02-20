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
#include "timer_context.hpp"

namespace unifex {

namespace _timer_context {

timer_context* unique_timer_context = nullptr;

} // namespace _timer_context 

timer_context::timer_context()
{
  assert(_timer_context::unique_timer_context == nullptr);
  _timer_context::unique_timer_context = this;
}

timer_context::~timer_context() {
  stop_ = true;
  assert(head_ == nullptr);
  _timer_context::unique_timer_context = nullptr;
}

void timer_context::enqueue(task_base* task) noexcept {
  if (head_ == nullptr || task->dueTime_ < head_->dueTime_) {
    // Insert at the head of the queue.
    task->next_ = head_;
    task->prevNextPtr_ = &head_;
    if (head_ != nullptr) {
      head_->prevNextPtr_ = &task->next_;
    }
    head_ = task;

    auto now = clock_t::now();
    std::uint16_t click_count = std::min(head_->dueTime_ - now, arduino::max_clicks).count();

    // New minimum due-time has changed, set the hardware timer.
    noInterrupts();                          // disable all interrupts while interrupt parameters are changing
    TCCR1A = 0;                              // reset all bits
    TCCR1B = 0;                              // reset all bits
    OCR1A = click_count;                     // input is already in clicks, in theory should subtract 1
    TCCR1B |= (1 << WGM12);                  // CTC mode
    TCCR1B |= arduino::timer_prescaler_mask; // On a 16MHz Arduino, a prescaler of 64 gives a 4usecond click
    TIFR1 |= (1 << OCF1A);                   // clear

    TIMSK1 |= (1 << OCIE1A);                 // enable timer compare interrupt
    TCNT1 = 0;                               // set counter to 0
    interrupts();                            // enable all interrupts
  } else {
    auto* queuedTask = head_;
    while (queuedTask->next_ != nullptr &&
           queuedTask->next_->dueTime_ <= task->dueTime_) {
      queuedTask = queuedTask->next_;
    }

    // Insert after queuedTask
    task->prevNextPtr_ = &queuedTask->next_;
    task->next_ = queuedTask->next_;
    if (task->next_ != nullptr) {
      task->next_->prevNextPtr_ = &task->next_;
    }
    queuedTask->next_ = task;
  }
}

void timer_context::run() {
  noInterrupts();                       // disable all interrupts
  TIMSK1 &= ~(1 << OCIE1A);             // disable future interrupts ie. CTC interrupt to give a 'one-shot' effect
  interrupts();                         // enable all interrupts

  
  while (!stop_ && head_ != nullptr && head_->dueTime_ <= clock_t::now()) {
    auto now = clock_t::now();
    auto nextDueTime = head_->dueTime_;
    if (nextDueTime <= now) {
      // Ready to run

      // Dequeue item
      auto* task = head_;
      head_ = task->next_;
      if (head_ != nullptr) {
        head_->prevNextPtr_ = &head_;
      }

      // Flag the task as dequeued.
      task->prevNextPtr_ = nullptr;

      task->execute();
    } else {
      // Not yet ready to run.
      std::uint16_t click_count = std::min(nextDueTime - now, arduino::max_clicks).count();

      noInterrupts();                          // disable all interrupts while interrupt parameters are changing
      TCCR1A = 0;                              // reset all bits
      TCCR1B = 0;                              // reset all bits
      OCR1A = click_count;                     // input is already in clicks, in theory should subtract 1
      TCCR1B |= (1 << WGM12);                  // CTC mode
      TCCR1B |= arduino::timer_prescaler_mask; // On a 16MHz Arduino, a prescaler of 64 gives a 4usecond click
      TIFR1 |= (1 << OCF1A);                   // clear

      TIMSK1 |= (1 << OCIE1A);                 // enable timer compare interrupt
      TCNT1 = 0;                               // set counter to 0
      interrupts();                            // enable all interrupts
    }
  }
}

void _timer_context::cancel_callback::operator()() noexcept {
  auto now = clock_t::now();
  if (now < task_->dueTime_) {
    task_->dueTime_ = now;

    if (task_->prevNextPtr_ != nullptr) {
      // Task is still in the queue, dequeue and requeue it.

      // Remove from the queue.
      *task_->prevNextPtr_ = task_->next_;
      if (task_->next_ != nullptr) {
        task_->next_->prevNextPtr_ = task_->prevNextPtr_;
      }
      task_->prevNextPtr_ = nullptr;

      // And requeue with an updated time.
      task_->dueTime_ = now;
      task_->context_->enqueue(task_);
    }
  }
}

} // namespace unifex

// **********   Hardware timing interrupt event  ***************

ISR(TIMER1_COMPA_vect) // timer compare interrupt service routine
{
  if (unifex::_timer_context::unique_timer_context != nullptr) {
   unifex::_timer_context::unique_timer_context->run();
  }
}
