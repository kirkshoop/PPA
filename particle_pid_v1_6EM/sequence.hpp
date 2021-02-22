#pragma once;

#include <unifex/receiver_concepts.hpp>
#include <unifex/sender_concepts.hpp>

// new cpos and concepts for unwind and sequences
//

#include <unifex/detail/prologue.hpp>
namespace unifex {
// async cleanup
//
// Given:
// os = connect(s, r);
//
// one of the following sequences of signals will occur
//
// set_value(r, vn...) eventually responds with 
// unwind(get_unwinder(os), set_value) which eventually responds with
// unwound(r, set_value)
//
// set_error(r, e) eventually responds with 
// unwind(get_unwinder(os), set_error) which eventually responds with
// unwound(r, set_error)
//
// set_done(r) eventually responds with 
// unwind(get_unwinder(os), set_done) which eventually responds with
// unwound(r, set_done)

inline const struct _get_unwinder_fn {
private:
  template <typename OperationState>
  using _result_t = meta_tag_invoke_result<_get_unwinder_fn>::template apply<OperationState>;
public:
  template(typename OperationState)
    (requires tag_invocable<_get_unwinder_fn, const OperationState&>)
  auto operator()(const OperationState& os) const
    noexcept(
      is_nothrow_tag_invocable_v<_get_unwinder_fn, const OperationState&>)
    -> _result_t<const OperationState&> {
    return unifex::tag_invoke(_get_unwinder_fn{}, os);
  }
} get_unwinder{};

inline const struct _unwind_fn {
private:
  template <typename Unwinder, typename Cpo, typename... Values>
  using _result_t = meta_tag_invoke_result<_unwind_fn>::template apply<Unwinder, Cpo, Values...>;
public:
  template(typename Unwinder, typename Cpo, typename... Values)
    (requires tag_invocable<_unwind_fn, Unwinder&, Cpo, Values...>)
  auto operator()(Unwinder& cln, Cpo&& cpo, Values&&... values) const
    noexcept(
      is_nothrow_tag_invocable_v<_unwind_fn, Unwinder&, Cpo, Values...>)
    -> _result_t<Unwinder&, Cpo, Values...> {
    return unifex::tag_invoke(
      _unwind_fn{}, cln, (Cpo &&) cpo, (Values &&) values...);
  }
} unwind{};

inline const struct _unwound_fn {
private:
  template <typename Receiver, typename Cpo, typename... Values>
  using _result_t = meta_tag_invoke_result<_unwound_fn>::template apply<Receiver, Cpo, Values...>;
public:
  template(typename Receiver, typename Cpo, typename... Values)
    (requires tag_invocable<_unwound_fn, Receiver&, Cpo, Values...>)
  auto operator()(Receiver& r, Cpo&& cpo, Values&&... values) const
    noexcept(
      is_nothrow_tag_invocable_v<_unwound_fn, Receiver&, Cpo, Values...>)
    -> _result_t<Receiver&, Cpo, Values...> {
    return unifex::tag_invoke(
      _unwound_fn{}, r, (Cpo &&) cpo, (Values &&) values...);
  }
} unwound{};

#if UNIFEX_CXX_CONCEPTS
// Defined the receiver concepts without the macros for improved diagnostics
template <typename C, typename Cpo, typename... Vn>
concept //
  unwinder_of = //
    requires(C&& c, Cpo cpo, Vn&&... vn)
    {
      { unwind((C&&) c, cpo, (Vn&&) vn...) } noexcept;
    };
#else
template <typename T, typename Cpo, typename... Vn>
UNIFEX_CONCEPT_FRAGMENT( 
  _unwinder_of, 
    requires(T&& t, Cpo&& cpo, Vn&&... vn) 
    (
      set_index((T&&) t, (Cpo&&) cpo, (Vn&&) vn...)
    ));

template <typename T, typename Cpo, typename... Vn>
UNIFEX_CONCEPT //
  unwinder_of = //
    UNIFEX_FRAGMENT(unifex::_unwinder_of, T, Cpo, Vn...);
#endif

template <typename T, typename Cpo, typename... Vn>
inline constexpr bool is_nothrow_unwinder_of_v =
  unwinder_of<T, Cpo, Vn...> &&
  is_nothrow_callable_v<decltype(unwind), T, Cpo, Vn...>;

// sequences
//
// the signals for a sequence of values followed by async cleanup
//
// Given:
// os = connect(s, r);
//
// one or more set_index(r, i, vn...) mixed with
// one or more step(os) which eventually responds with one or more
// set_index(r, i, vn...)
// each set_index(r, i, vn...) eventually responds with 
//   step(os)
// each set_index(r, i, vn...) eventually responds with 
//   unwind(get_unwinder(os), set_index, i) which eventually responds with 
//   unwound(r, set_index, i)
// set_end(r) eventually responds with 
//   unwind(get_unwinder(os), set_end) which eventually responds with 
//   unwound(r, set_end)
// set_error(r, e) eventually responds with 
//   unwind(get_unwinder(os), set_error) which eventually responds with 
//   unwound(get_unwinder(os), set_error)
// set_done(r) eventually responds with 
//   unwind(get_unwinder(os), set_done) which eventually responds with 
//   unwound(get_unwinder(os), set_done)
// calls to set_index after set_end, set_error and 
// set_done are UB.
//

inline const struct _step_fn {
private:
  template <typename OperationState>
  using _result_t = meta_tag_invoke_result<_step_fn>::template apply<OperationState>;
public:
  template(typename OperationState)
    (requires tag_invocable<_step_fn, OperationState&>)
  auto operator()(OperationState& os) const
    noexcept(
      is_nothrow_tag_invocable_v<_step_fn, OperationState&>)
    -> _result_t<OperationState&> {
    return unifex::tag_invoke(_step_fn{}, os);
  }
} step{};

inline const struct _set_index_fn {
private:
  template <typename Receiver, typename Index, typename... Values>
  using _result_t = meta_tag_invoke_result<_set_index_fn>::template apply<Receiver, Index, Values...>;
public:
  template(typename Receiver, typename Index, typename... Values)
    (requires tag_invocable<_set_index_fn, const Receiver&, Index, Values...>)
  auto operator()(const Receiver& r, Index&& i, Values&&... values) const
    noexcept(
      is_nothrow_tag_invocable_v<_set_index_fn, const Receiver&, Index, Values...>)
    -> _result_t<const Receiver&, Index, Values...> {
    return unifex::tag_invoke(
      _set_index_fn{}, r, (Index &&) i, (Values &&) values...);
  }
  // drop values to interop with receivers that do not support set_index and set_end
  template(typename Receiver, typename Index, typename... Values)
    (requires (!tag_invocable<_set_index_fn, const Receiver&, Index, Values...>) AND
               tag_invocable<tag_t<unifex::set_value>, Receiver&&>)
  void operator()(const Receiver& , Index&& , Values&&... ) const noexcept {
  }
} set_index{};

inline const struct _set_end_fn {
private:
  template <typename Receiver>
  using _result_t = meta_tag_invoke_result<_set_end_fn>::template apply<Receiver>;
public:
  template(typename Receiver)
    (requires tag_invocable<_set_end_fn, Receiver>)
  auto operator()(Receiver&& r) const noexcept -> _result_t<Receiver> {
    return unifex::tag_invoke(_set_end_fn{}, (Receiver &&) r);
  }
  // translate to set_value() to interop with receivers that 
  // do not have set_index and set_end
  template(typename Receiver)
    (requires (!tag_invocable<_set_end_fn, Receiver>) AND
               tag_invocable<tag_t<unifex::set_value>, Receiver>)
  auto operator()(Receiver&& r) const noexcept -> 
    tag_invoke_result_t<tag_t<unifex::set_value>, Receiver> {
    if constexpr (is_nothrow_tag_invocable_v<tag_t<unifex::set_value>, Receiver>) {
      return unifex::set_value((Receiver &&) r);
    } else {
      UNIFEX_TRY {
        return unifex::set_value((Receiver &&) r);
      } UNIFEX_CATCH (...) {
        unifex::set_error((Receiver &&) r, std::current_exception());
      }
    }
  }
} set_end{};

#if UNIFEX_CXX_CONCEPTS
// Defined the receiver concepts without the macros for improved diagnostics
template <typename R, typename E = std::exception_ptr>
concept //
  sequence_receiver = //
    receiver<R, E> &&
    requires(R&& r)
    {
      { set_end(std::move(r)) } noexcept;
      { unwound((R&&)r, set_end) } noexcept;
      // move to receiver
      { unwound((R&&)r, set_error) } noexcept;
      { unwound((R&&)r, set_done) } noexcept;
    };

template <typename R, typename Index, typename... An>
concept //
  sequence_receiver_of = //
    sequence_receiver<R> &&
    requires(R&& r, Index&& i, An&&... an) //
    {
      set_index((R&&) r, (Index&&) i, (An&&) an...);
      { unwound((R&&) r, set_index, i) } noexcept;
    };
#else
template <typename R>
UNIFEX_CONCEPT_FRAGMENT(
  _sequence_receiver,
    requires(R&& r) (
      noexcept(set_end(std::move(r))),
      noexcept(unwound((R&&)r, set_end)),
      noexcept(unwound((R&&)r, set_error)),
      noexcept(unwound((R&&)r, set_done))
    ));

template <typename R, typename E = std::exception_ptr>
UNIFEX_CONCEPT //
  sequence_receiver = //
    receiver<R, E> &&
    UNIFEX_FRAGMENT(unifex::_sequence_receiver, R);

template <typename T, typename Index, typename... An>
UNIFEX_CONCEPT_FRAGMENT( 
  _sequence_receiver_of, 
    requires(T&& t, Index&& i, An&&... an) (
      set_index((T&&) t, (Index&&) i, (An&&) an...),
      noexcept(unwound((T&&)t, set_index, i))
    ));

template <typename R, typename Index, typename... An>
UNIFEX_CONCEPT //
  sequence_receiver_of = //
    sequence_receiver<R> &&
    UNIFEX_FRAGMENT(unifex::_sequence_receiver_of, R, Index, An...);
#endif

template <typename R, typename Index, typename... An>
inline constexpr bool is_nothrow_sequence_receiver_of_v =
  sequence_receiver_of<R, Index, An...> &&
  is_nothrow_callable_v<decltype(set_index), R, Index, An...>;

//////////////////
// Metafunctions for checking callability of specific receiver methods

template <typename R, typename Index, typename... An>
inline constexpr bool is_index_sequence_receiver_v =
  is_callable_v<decltype(set_index), R&, Index, An...>;

template <typename R, typename Index, typename... An>
inline constexpr bool is_nothrow_index_sequence_receiver_v =
  is_nothrow_callable_v<decltype(set_index), R&, Index, An...>;

} // namespace unifex
#include <unifex/detail/epilogue.hpp>

// transform() algo with support for unwind and sequences
//

#include <unifex/config.hpp>
#include <unifex/receiver_concepts.hpp>
#include <unifex/sender_concepts.hpp>
#include <unifex/stream_concepts.hpp>
#include <unifex/type_traits.hpp>
#include <unifex/blocking.hpp>
#include <unifex/get_stop_token.hpp>
#include <unifex/async_trace.hpp>
#include <unifex/type_list.hpp>
#include <unifex/std_concepts.hpp>
#include <unifex/bind_back.hpp>

#include <exception>
#include <functional>
#include <type_traits>
#include <utility>

#include <unifex/detail/prologue.hpp>

namespace unifex {
namespace _tfx {
namespace detail {
  template <typename Result, typename = void>
  struct result_overload {
    using type = type_list<Result>;
  };
  template <typename Result>
  struct result_overload<Result, std::enable_if_t<std::is_void_v<Result>>> {
    using type = type_list<>;
  };
}

template <typename Predecessor, typename Receiver, typename Func>
struct _receiver {
  struct type;
};
template <typename Predecessor, typename Receiver, typename Func>
using receiver_t = typename _receiver<Predecessor, Receiver, Func>::type;

template <typename Predecessor, typename Receiver, typename Func>
struct _op {
  struct type;
};
template <typename Predecessor, typename Receiver, typename Func>
using op_t = typename _op<Predecessor, Receiver, Func>::type;

template <typename Predecessor, typename Receiver, typename Func>
struct _op<Predecessor, Receiver, Func>::type {
  using receiver_t = receiver_t<Predecessor, Receiver, Func>;
  using pred_state_t = connect_result_t<Predecessor, receiver_t>;
  UNIFEX_NO_UNIQUE_ADDRESS Func func_;
  UNIFEX_NO_UNIQUE_ADDRESS Receiver receiver_;
  UNIFEX_NO_UNIQUE_ADDRESS pred_state_t pred_state_;

  template <typename Predecessor2, typename Receiver2, typename Func2>
  type(Predecessor2&& pred, Receiver2&& receiver, Func2&& func) : 
    func_((Func2&&) func),
    receiver_((Receiver2&&) receiver),
    pred_state_(unifex::connect(
      (Predecessor2&&) pred,
      receiver_t{this})) {
  }

  friend auto tag_invoke(unifex::tag_t<unifex::get_unwinder>, type& self) noexcept {
    return unifex::get_unwinder(self.pred_state_);
  }

  friend void tag_invoke(unifex::tag_t<unifex::step>, type& self) noexcept {
    return unifex::step(self.pred_state_);
  }

  friend void tag_invoke(unifex::tag_t<unifex::start>, type& self) noexcept {
    unifex::start(self.pred_state_);
  }
};

template <typename Sender, typename Receiver, typename Func>
struct _receiver<Sender, Receiver, Func>::type {
  UNIFEX_NO_UNIQUE_ADDRESS op_t<Sender, Receiver, Func>* op_;

  template <typename Index, typename... Values>
  friend void tag_invoke(unifex::tag_t<unifex::set_index>, const type& self, Index&& idx, Values&&... values) noexcept {
    using result_type = unifex::callable_result_t<Func, Index, Values...>;
    auto& op = *self.op_;
    if constexpr (std::is_void_v<result_type>) {
      if constexpr (noexcept(std::invoke(
                        (Func &&) op.func_, idx, (Values &&) values...))) {
        std::invoke((Func &&) op.func_, idx, (Values &&) values...);
        unifex::set_index(op.receiver_, idx);
      } else {
        UNIFEX_TRY {
          std::invoke((Func &&) op.func_, idx, (Values &&) values...);
          unifex::set_index(op.receiver_, idx);
        } UNIFEX_CATCH (...) {
          unifex::set_error((Receiver &&) op.receiver_, std::current_exception());
        }
      }
    } else {
      if constexpr (noexcept(std::invoke(
                        (Func &&) op.func_, idx, (Values &&) values...))) {
        unifex::set_index(
            (Receiver &&) op.receiver_, 
            (Index&&) idx,
            std::invoke((Func &&) op.func_, idx, (Values &&) values...));
      } else {
        UNIFEX_TRY {
          unifex::set_index(
              (Receiver &&) op.receiver_, 
              idx,
              std::invoke((Func &&) op.func_, idx, (Values &&) values...));
        } UNIFEX_CATCH (...) {
          unifex::set_error((Receiver &&) op.receiver_, std::current_exception());
        }
      }
    }
    // Hack
    unifex::step(const_cast<std::remove_const_t<decltype(op.pred_state_)>&>(op.pred_state_));
    auto uw = unifex::get_unwinder(op.pred_state_);
    unifex::unwind(uw, unifex::set_index, idx);
  }

  friend void tag_invoke(unifex::tag_t<unifex::set_end>, type&& self) noexcept {
    unifex::set_end(std::move(self.op_->receiver_));
    // Hack
    auto uw = unifex::get_unwinder(self.op_->pred_state_);
    unifex::unwind(uw, unifex::set_end);
  }

  template <typename Cpo, typename... Vn>
  friend void tag_invoke(unifex::tag_t<unifex::unwound>, type& self, Cpo, Vn&&...) noexcept {
  }

  template <typename... Values>
  void set_value(Values&&... values) && noexcept {
    using result_type = std::invoke_result_t<Func, Values...>;
    if constexpr (std::is_void_v<result_type>) {
      if constexpr (noexcept(std::invoke(
                        (Func &&) op_->func_, (Values &&) values...))) {
        std::invoke((Func &&) op_->func_, (Values &&) values...);
        unifex::set_value((Receiver &&) op_->receiver_);
      } else {
        UNIFEX_TRY {
          std::invoke((Func &&) op_->func_, (Values &&) values...);
          unifex::set_value((Receiver &&) op_->receiver_);
        } UNIFEX_CATCH (...) {
          unifex::set_error((Receiver &&) op_->receiver_, std::current_exception());
        }
      }
    } else {
      if constexpr (noexcept(std::invoke(
                        (Func &&) op_->func_, (Values &&) values...))) {
        unifex::set_value(
            (Receiver &&) op_->receiver_,
            std::invoke((Func &&) op_->func_, (Values &&) values...));
      } else {
        UNIFEX_TRY {
          unifex::set_value(
              (Receiver &&) op_->receiver_,
              std::invoke((Func &&) op_->func_, (Values &&) values...));
        } UNIFEX_CATCH (...) {
          unifex::set_error((Receiver &&) op_->receiver_, std::current_exception());
        }
      }
    }
    // Hack
    auto uw = unifex::get_unwinder(op_->pred_state_);
    unifex::unwind(uw, unifex::set_value);
  }

  template <typename Error>
  void set_error(Error&& error) && noexcept {
    unifex::set_error((Receiver &&) op_->receiver_, (Error &&) error);
    // Hack
    auto uw = unifex::get_unwinder(op_->pred_state_);
    unifex::unwind(uw, unifex::set_error);
  }

  void set_done() && noexcept {
    unifex::set_done((Receiver &&) op_->receiver_);
    // Hack
    auto uw = unifex::get_unwinder(op_->pred_state_);
    unifex::unwind(uw, unifex::set_done);
  }

  template(typename CPO, typename R)
      (requires is_receiver_query_cpo_v<CPO> AND same_as<R, type>)
  friend auto tag_invoke(CPO cpo, const R& r) noexcept(
      is_nothrow_callable_v<CPO, const Receiver&>)
      -> callable_result_t<CPO, const Receiver&> {
    return std::move(cpo)(std::as_const(r.op_->receiver_));
  }

  template <typename Visit>
  friend void tag_invoke(tag_t<visit_continuations>, const type& r, Visit&& visit) {
    std::invoke(visit, r.op_->receiver_);
  }
};

template <typename Predecessor, typename Func>
struct _sender {
  struct type;
};
template <typename Predecessor, typename Func>
using sender = typename _sender<remove_cvref_t<Predecessor>, std::decay_t<Func>>::type;

template <typename Predecessor, typename Func>
struct _sender<Predecessor, Func>::type {
  UNIFEX_NO_UNIQUE_ADDRESS Predecessor pred_;
  UNIFEX_NO_UNIQUE_ADDRESS Func func_;

private:

  // This helper transforms an argument list into either
  // - type_list<type_list<Result>> - if Result is non-void, or
  // - type_list<type_list<>>       - if Result is void
  template <typename... Args>
  using result = type_list<
    typename detail::result_overload<std::invoke_result_t<Func, Args...>>::type>;

public:

  template <
      template <typename...> class Variant,
      template <typename...> class Tuple>
  using value_types =
      type_list_nested_apply_t<
          sender_value_types_t<Predecessor, concat_type_lists_unique_t, result>,
          Variant,
          Tuple>;

  template <template <typename...> class Variant>
  using error_types =
      typename concat_type_lists_unique_t<
          sender_error_types_t<Predecessor, type_list>,
          type_list<std::exception_ptr>>::template apply<Variant>;

  static constexpr bool sends_done = sender_traits<Predecessor>::sends_done;

  template <typename Receiver>
  using op_t = op_t<Predecessor, Receiver, Func>;

  template <typename Receiver>
  using receiver_t = receiver_t<Predecessor, Receiver, Func>;

  friend constexpr auto tag_invoke(tag_t<blocking>, const type& sender) {
    return blocking(sender.pred_);
  }

  template(typename Sender, typename Receiver)
    (requires same_as<remove_cvref_t<Sender>, type> AND receiver<Receiver>)
  friend auto tag_invoke(tag_t<unifex::connect>, Sender&& s, Receiver&& r)
    noexcept(
      std::is_nothrow_constructible_v<remove_cvref_t<Receiver>, Receiver> &&
      std::is_nothrow_constructible_v<Func, decltype((static_cast<Sender&&>(s).func_))> &&
      std::is_nothrow_constructible_v<Predecessor, decltype((static_cast<Sender&&>(s).pred_))> &&
      is_nothrow_connectable_v<decltype((static_cast<Sender&&>(s).pred_)), receiver_t<remove_cvref_t<Receiver>>>)
      -> op_t<remove_cvref_t<Receiver>> {
    return op_t<remove_cvref_t<Receiver>>{
      static_cast<Sender&&>(s).pred_,
      static_cast<Receiver&&>(r),
      static_cast<Sender&&>(s).func_
    };
  }
};
} // namespace _tfx

namespace _tfx_cpo {
  inline const struct _fn {
  private:
    template <typename Sender, typename Func>
    using _result_t =
      typename conditional_t<
        tag_invocable<_fn, Sender, Func>,
        meta_tag_invoke_result<_fn>,
        meta_quote2<_tfx::sender>>::template apply<Sender, Func>;
  public:
    template(typename Sender, typename Func)
      (requires tag_invocable<_fn, Sender, Func>)
    auto operator()(Sender&& predecessor, Func&& func) const
        noexcept(is_nothrow_tag_invocable_v<_fn, Sender, Func>)
        -> _result_t<Sender, Func> {
      return unifex::tag_invoke(_fn{}, (Sender&&)predecessor, (Func&&)func);
    }
    template(typename Sender, typename Func)
      (requires (!tag_invocable<_fn, Sender, Func>))
    auto operator()(Sender&& predecessor, Func&& func) const
        noexcept(std::is_nothrow_constructible_v<
          _tfx::sender<Sender, Func>, Sender, Func>)
        -> _result_t<Sender, Func> {
      return _tfx::sender<Sender, Func>{(Sender &&) predecessor, (Func &&) func};
    }
    template <typename Func>
    constexpr auto operator()(Func&& func) const
        noexcept(is_nothrow_callable_v<
          tag_t<bind_back>, _fn, Func>)
        -> bind_back_result_t<_fn, Func> {
      return bind_back(*this, (Func &&) func);
    }
  } transform{};
} // namespace _tfx_cpo
using _tfx_cpo::transform;
} // namespace unifex

#include <unifex/detail/epilogue.hpp>

// new interval() algo with support for unwind and sequences
//

#include <memory>
#include <string>
#include <chrono>

#include <unifex/detail/prologue.hpp>
namespace unifex {
namespace _inrvl_cpo {
inline constexpr struct interval_fn {
  template<typename Clock, typename Receiver>
  struct unwinder {
    struct type;
  };
  template<typename Clock, typename Receiver>
  struct op {
    struct tick_receiver;
    struct type;
  };
  template<typename Clock>
  struct sender {
    struct type;
  };
  template <typename TimePoint, typename Duration>
  auto operator()(TimePoint reference, Duration gap) const 
    -> typename sender<typename TimePoint::clock>::type {
    return typename sender<typename TimePoint::clock>::type{
      std::chrono::time_point_cast<typename TimePoint::clock::duration>(reference), 
      std::chrono::duration_cast<typename TimePoint::clock::duration>(gap)
    };
  }
} interval;
template<typename Clock, typename Receiver>
struct interval_fn::op<Clock, Receiver>::tick_receiver {
  friend void tag_invoke(unifex::tag_t<unifex::set_value>, tick_receiver&& self) {
    // Serial.println("sv");
    auto op = self.op_;
    auto tick = op->tick_;
    unifex::set_index(op->receiver_, tick);
    using tickOp = decltype(op->tickOp_);
    op->tickOp_.~tickOp();
    op->tick_ += op->gap_;
    new(&op->tickOp_) auto(unifex::connect(unifex::schedule_at(op->scheduler_, 
      op->tick_
    ), tick_receiver{op}));
    unifex::start(op->tickOp_);
    // Serial.println((long)(op->tick_ - op->scheduler_.now()).count());
    // Serial.println("~sv");
  }
  template<typename Error>
  friend void tag_invoke(unifex::tag_t<unifex::set_error>, tick_receiver&& self, Error&& error) noexcept {
    unifex::set_error(std::move(self.op_->receiver_), (Error&&) error);
  }
  friend void tag_invoke(unifex::tag_t<unifex::set_done>, tick_receiver&& self) noexcept {
    unifex::set_done(std::move(self.op_->receiver_));
  }
  template(typename CPO, typename R)
      (requires unifex::is_receiver_query_cpo_v<CPO> AND
        unifex::same_as<R, tick_receiver> AND
        unifex::is_callable_v<CPO, const Receiver&>)
  friend auto tag_invoke(
      CPO cpo,
      const R& r) noexcept(unifex::is_nothrow_callable_v<
                                      CPO,
                                      const Receiver&>)
      -> unifex::callable_result_t<CPO, const Receiver&> {
    return static_cast<CPO&&>(cpo)(r.op_->receiver_);
  }
  template <typename Func>
  friend void tag_invoke(
      unifex::tag_t<unifex::visit_continuations>,
      const tick_receiver& r,
      Func&& func) {
    std::invoke(func, std::as_const(r.get_receiver()));
  }
  type* op_;
};
template<typename Clock, typename Receiver>
struct interval_fn::unwinder<Clock, Receiver>::type {
  using op_t = typename op<Clock, Receiver>::type;
  op_t* op_;
  template <typename Cpo, typename... Vn>
  friend void tag_invoke(unifex::tag_t<unifex::unwind>, type& self, Cpo cpo, Vn&&... vn) noexcept {
    unifex::unwound(self.op_->receiver_, cpo, (Vn&&) vn...);
  }
};
template<typename Clock, typename Receiver>
struct interval_fn::op<Clock, Receiver>::type {
  using time_point = typename Clock::time_point;
  using duration = typename Clock::duration;
  using unwinder_t = typename unwinder<Clock, Receiver>::type;
  using scheduler = std::remove_cvref_t<unifex::callable_result_t<
    unifex::tag_t<unifex::get_scheduler>, Receiver>>;
  using tick_sender = std::remove_cvref_t<unifex::callable_result_t<
    unifex::tag_t<unifex::schedule_at>, scheduler&, time_point>>;
  using tick_op = unifex::connect_result_t<tick_sender, tick_receiver>;

  time_point reference_;
  time_point tick_;
  duration gap_;
  Receiver receiver_;
  scheduler scheduler_;
  tick_op tickOp_;

  template<typename Receiver2>
  type(
    time_point reference, 
    duration gap, 
    Receiver2&& r, 
    scheduler sched) : 
    reference_(reference),
    tick_(reference),
    gap_(gap),
    receiver_((Receiver2&&)r),
    scheduler_(sched),
    tickOp_(unifex::connect(
      unifex::schedule_at(scheduler_, tick_), 
      tick_receiver{this}))
  {
    // Serial.println("op");
    // Serial.println((long)(tick_ - scheduler_.now()).count());
  }
  ~type() {
    // Serial.println("~op");
  }

  type(const type&) = delete;
  type(type&&) = delete;
  type& operator=(const type&) = delete;
  type& operator=(type&&) = delete;


  friend unwinder_t tag_invoke(unifex::tag_t<unifex::get_unwinder>, const type& self) noexcept {
    return unwinder_t{const_cast<type*>(&self)};
  }

  friend void tag_invoke(unifex::tag_t<unifex::step>, type&) noexcept {
  }

  friend void tag_invoke(unifex::tag_t<unifex::start>, type& self) noexcept {
   unifex::start(self.tickOp_);
  }
};
template<typename Clock>
struct interval_fn::sender<Clock>::type {
  using time_point = typename Clock::time_point;
  using duration = typename Clock::duration;
  time_point reference_;
  duration gap_;

  template <
      template <typename...> class Variant,
      template <typename...> class Tuple>
  using value_types = Variant<Tuple<time_point>>;

  template <template <typename...> class Variant>
  using error_types = Variant<std::exception_ptr>;

  static constexpr bool sends_done = true;

  template<typename Receiver>
  friend auto tag_invoke(unifex::tag_t<unifex::connect>, const type& self, Receiver&& receiver)
    -> typename op<Clock, Receiver>::type {
    auto scheduler = unifex::get_scheduler(receiver);
    return {self.reference_, self.gap_, (Receiver&&)receiver, std::move(scheduler)};
  }
};
} // namespace _inrvl_cpo
using _inrvl_cpo::interval;
} // namespace unifex
#include <unifex/detail/epilogue.hpp>

// new generate() algo with support for unwind and sequences
//

#include <unifex/detail/prologue.hpp>
namespace unifex {
namespace _gen_cpo {
template <typename T>
struct incr {
  friend T operator+(T t, const incr&) noexcept {
    return ++t;
  }
};
inline constexpr struct generate_fn {
  template<typename First, typename Last, typename Step, typename Receiver>
  struct unwinder {
    struct type;
  };
  template<typename First, typename Last, typename Step, typename Receiver>
  struct op {
    struct type;
  };
  template<typename First, typename Last, typename Step>
  struct sender {
    struct type;
  };
  template <typename First, typename Last = First, typename Step = incr<First>>
  auto operator()(
    First first, 
    Last last = std::numeric_limits<First>::max(), 
    Step step = {}) const 
    -> typename sender<First, Last, Step>::type {
    return {first, last, step};
  }
} generate;
template<typename First, typename Last, typename Step, typename Receiver>
struct generate_fn::unwinder<First, Last, Step, Receiver>::type {
  using op_t = typename op<First, Last, Step, Receiver>::type;
  op_t* op_;
  friend void tag_invoke(unifex::tag_t<unifex::unwind>, type& self, tag_t<unifex::set_error>) noexcept {
    unifex::unwound(self.op_->receiver_, unifex::set_error);
  }
  friend void tag_invoke(unifex::tag_t<unifex::unwind>, type& self, tag_t<unifex::set_end>) noexcept {
    unifex::unwound(self.op_->receiver_, unifex::set_end);
  }
  template<typename Index>
  friend void tag_invoke(unifex::tag_t<unifex::unwind>, type& self, tag_t<unifex::set_index>, const Index& i) noexcept {
    unifex::unwound(self.op_->receiver_, unifex::set_index, i);
  }
};
template<typename First, typename Last, typename Step, typename Receiver>
struct generate_fn::op<First, Last, Step, Receiver>::type {
  using first_t = std::decay_t<First>;
  using last_t = std::decay_t<Last>;
  using step_t = std::decay_t<Step>;
  using result_t = decltype(std::declval<first_t>() + std::declval<step_t>());
  using unwinder_t = typename unwinder<First, Last, Step, Receiver>::type;
  const first_t first_;
  const last_t last_;
  const step_t step_;
  result_t idx_;
  std::decay_t<Receiver> receiver_;

  friend unwinder_t tag_invoke(unifex::tag_t<unifex::get_unwinder>, const type& self) noexcept {
    return {const_cast<type*>(&self)};
  }

  friend void tag_invoke(unifex::tag_t<unifex::step>, type& self) noexcept {
    const bool inc = self.first_ < self.idx_;
    const bool dec = self.idx_ < self.first_;
    if (!(self.idx_ < self.last_) && !(self.last_ < self.idx_)) {
      unifex::set_end(std::move(self.receiver_));
    } else if ((inc && self.idx_ < self.last_ && self.first_ < self.idx_) ||
               (dec && self.last_ < self.idx_ && self.idx_ < self.first_)) {
      auto idx = self.idx_;
      self.idx_ = self.idx_ + self.step_;
      if (!(self.idx_ < self.last_) && !(self.last_ < self.idx_)) {
        self.idx_ = self.idx_ + self.step_;
      }
      unifex::set_index(self.receiver_, idx);
    } else {
      self.idx_ = self.last_;
      unifex::set_index(self.receiver_, self.last_);
    }
  }

  friend void tag_invoke(unifex::tag_t<unifex::start>, type& self) noexcept {
    UNIFEX_TRY {
      unifex::set_index(self.receiver_, self.first_);
    } UNIFEX_CATCH (...) {
      unifex::set_error(std::move(self.receiver_), std::current_exception());
    }
  }
};
template<typename First, typename Last, typename Step>
struct generate_fn::sender<First, Last, Step>::type {
  using first_t = std::decay_t<First>;
  using last_t = std::decay_t<Last>;
  using step_t = std::decay_t<Step>;
  using result_t = decltype(std::declval<first_t>() + std::declval<step_t>());
  first_t first_;
  last_t last_;
  step_t step_;

  template <
      template <typename...> class Variant,
      template <typename...> class Tuple>
  using value_types = Variant<Tuple<result_t>>;

  template <template <typename...> class Variant>
  using error_types = Variant<std::exception_ptr>;

  static constexpr bool sends_done = true;

  template<typename Receiver>
  friend auto tag_invoke(unifex::tag_t<unifex::connect>, const type& self, Receiver&& receiver)
    -> typename op<First, Last, Step, Receiver>::type {
    return {self.first_, self.last_, self.step_, self.first_ + self.step_, (Receiver&&)receiver};
  }
};
} // namespace _gen_cpo
using _gen_cpo::generate;
} // namespace unifex
#include <unifex/detail/epilogue.hpp>
