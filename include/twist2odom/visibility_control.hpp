// Copyright 2023 Amadeusz Szymko
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TWIST_TO_ODOM__VISIBILITY_CONTROL_HPP_
#define TWIST_TO_ODOM__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(TWIST_TO_ODOM_BUILDING_DLL) || defined(TWIST_TO_ODOM_EXPORTS)
    #define TWIST_TO_ODOM_PUBLIC __declspec(dllexport)
    #define TWIST_TO_ODOM_LOCAL
  #else  // defined(TWIST_TO_ODOM_BUILDING_DLL) || defined(TWIST_TO_ODOM_EXPORTS)
    #define TWIST_TO_ODOM_PUBLIC __declspec(dllimport)
    #define TWIST_TO_ODOM_LOCAL
  #endif  // defined(TWIST_TO_ODOM_BUILDING_DLL) || defined(TWIST_TO_ODOM_EXPORTS)
#elif defined(__linux__)
  #define TWIST_TO_ODOM_PUBLIC __attribute__((visibility("default")))
  #define TWIST_TO_ODOM_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define TWIST_TO_ODOM_PUBLIC __attribute__((visibility("default")))
  #define TWIST_TO_ODOM_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // TWIST_TO_ODOM__VISIBILITY_CONTROL_HPP_
