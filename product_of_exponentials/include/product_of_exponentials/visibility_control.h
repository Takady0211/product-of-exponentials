// Copyright (c) 2025 Kazuki Takada
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PRODUCT_OF_EXPONENTIALS__VISIBILITY_CONTROL_H_
#define PRODUCT_OF_EXPONENTIALS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PRODUCT_OF_EXPONENTIALS_EXPORT __attribute__((dllexport))
#define PRODUCT_OF_EXPONENTIALS_IMPORT __attribute__((dllimport))
#else
#define PRODUCT_OF_EXPONENTIALS_EXPORT __declspec(dllexport)
#define PRODUCT_OF_EXPONENTIALS_IMPORT __declspec(dllimport)
#endif
#ifdef PRODUCT_OF_EXPONENTIALS_BUILDING_LIBRARY
#define PRODUCT_OF_EXPONENTIALS_PUBLIC PRODUCT_OF_EXPONENTIALS_EXPORT
#else
#define PRODUCT_OF_EXPONENTIALS_PUBLIC PRODUCT_OF_EXPONENTIALS_IMPORT
#endif
#define PRODUCT_OF_EXPONENTIALS_PUBLIC_TYPE PRODUCT_OF_EXPONENTIALS_PUBLIC
#define PRODUCT_OF_EXPONENTIALS_LOCAL
#else
#define PRODUCT_OF_EXPONENTIALS_EXPORT __attribute__((visibility("default")))
#define PRODUCT_OF_EXPONENTIALS_IMPORT
#if __GNUC__ >= 4
#define PRODUCT_OF_EXPONENTIALS_PUBLIC __attribute__((visibility("default")))
#define PRODUCT_OF_EXPONENTIALS_LOCAL __attribute__((visibility("hidden")))
#else
#define PRODUCT_OF_EXPONENTIALS_PUBLIC
#define PRODUCT_OF_EXPONENTIALS_LOCAL
#endif
#define PRODUCT_OF_EXPONENTIALS_PUBLIC_TYPE
#endif

#endif  // PRODUCT_OF_EXPONENTIALS__VISIBILITY_CONTROL_H_
