#undef min
#undef max
#undef round
#undef abs

#include <ratio>
#undef min
#undef max
#undef round
#undef abs

#include <type_traits>
#undef min
#undef max
#undef round
#undef abs

#include <limits>
#undef min
#undef max
#undef round
#undef abs

#include <ctime>
#undef min
#undef max
#undef round
#undef abs

#include <chrono>
#undef min
#undef max
#undef round
#undef abs

namespace std {
  template<typename T>
  using remove_cvref = ::std::remove_const<
    typename ::std::remove_volatile<
      typename remove_reference<T>::type>::type>;

  template<typename T>
  using remove_cvref_t = typename remove_cvref<T>::type;
}

/* Definition of struct __locale_struct and __locale_t.
   Copyright (C) 1997-2017 Free Software Foundation, Inc.
   This file is part of the GNU C Library.
   Contributed by Ulrich Drepper <drepper@cygnus.com>, 1997.
   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.
   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, see
   <http://www.gnu.org/licenses/>.  */
#ifndef _BITS_TYPES___LOCALE_T_H
#define _BITS_TYPES___LOCALE_T_H 1
/* POSIX.1-2008: the locale_t type, representing a locale context
   (implementation-namespace version).  This type should be treated
   as opaque by applications; some details are exposed for the sake of
   efficiency in e.g. ctype functions.  */
struct __locale_struct
{
  /* Note: LC_ALL is not a valid index into this array.  */
  struct __locale_data *__locales[13]; /* 13 = __LC_LAST. */
  /* To increase the speed of this solution we add some special members.  */
  const unsigned short int *__ctype_b;
  const int *__ctype_tolower;
  const int *__ctype_toupper;
  /* Note: LC_ALL is not a valid index into this array.  */
  const char *__names[13];
};
typedef struct __locale_struct *__locale_t;
#endif /* bits/types/__locale_t.h */

#include <unifex/config.hpp>
#define UNIFEX_NO_COROUTINES 1
#undef UNIFEX_COROUTINES_HEADER 
