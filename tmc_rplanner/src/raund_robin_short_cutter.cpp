/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @brief    Short -cut random planner, etc. Shorter the startup generated

#include <algorithm>
#include <tmc_rplanner/raund_robin_short_cutter.hpp>

namespace tmc_rplanner {

/// @func ShortCut
/// @brief Shortcut the orbit and shorten it
///        Brute force method
bool RoundRobinShortCutter::ShortCut(const Path& path_in, Path& path_out) {
  if (bidirectional_) {
    return ShortCutBiDirectional_(skip_, path_in, path_out);
  } else {
    return ShortCutOnce_(skip_, path_in, path_out);
  }
}


/// @func ShortCutOne_
/// @brief Shortcut the orbit and shorten it
///        Brute force method
bool RoundRobinShortCutter::ShortCutOnce_(
    int32_t skip, const Path& path_in, Path& path_out) const {
  if (skip <= 0) {
    throw std::invalid_argument("Skip must be positive.");
  }
  if (path_in.empty()) {
    throw std::invalid_argument("Empty path.");
  }
  path_out.clear();
  // Added the first point
  path_out.push_back(path_in[0]);
  // Shortcut starting point
  size_t start_index = 0;
  while (start_index < path_in.size()-1) {
    bool shortcutted = false;
    for (size_t goal_index = path_in.size() - 1;
         goal_index > start_index;
         goal_index -= skip) {
      Path part_path;
      // Check the end conditions (timeout, etc.)
      if (is_terminate_ && is_terminate_()) {
        path_out = path_in;
        return false;
      }

      // Check if a shortcut is possible
      if (space_->CheckLine(path_in[start_index],
                            path_in[goal_index], delta_, is_terminate_,
                            part_path)) {
        // Copy the shortcut path to PATH_OUT
        if (part_path.size() > 1) {
          std::copy(++part_path.begin(),
                    part_path.end(),
                    std::back_inserter(path_out));
        } else {
          path_out.push_back(part_path[0]);
        }
        start_index = goal_index;
        shortcutted = true;
        break;
      }
    }
    // If you can't shortcut
    if (!shortcutted) {
      // Copy the following points to PATH_OUT
      path_out.push_back(path_in[start_index+1]);
      start_index += 1;
    }
  }
  return true;
}


/// @func ShortCutBiDirectional
/// @brief Shortcut the orbit and shorten it
///        2-PATH general hit method
bool RoundRobinShortCutter::ShortCutBiDirectional_(
    int32_t skip, const Path& path_in, Path& path_out) const {
  if (skip <= 0) {
    throw std::invalid_argument("Skip mut be positive.");
  }
  if (path_in.empty()) {
    throw std::invalid_argument("Empty path.");
  }

  Path from_front;
  if (!ShortCutOnce_(skip, path_in, from_front)) {
    return false;
  }
  std::reverse(from_front.begin(), from_front.end());
  if (!ShortCutOnce_(skip, from_front, path_out)) {
    return false;
  }
  std::reverse(path_out.begin(), path_out.end());
  return true;
}

// end of name space
}  // namespace tmc_rplanner
