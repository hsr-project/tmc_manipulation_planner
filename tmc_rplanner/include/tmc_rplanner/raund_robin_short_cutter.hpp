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
/// @brief    Shortcut by a brute force

#ifndef TMC_MANIPULATION_TMC_RPLANNER_ROUND_ROBIN_SHORT_CUTTER_HPP_
#define TMC_MANIPULATION_TMC_RPLANNER_ROUND_ROBIN_SHORT_CUTTER_HPP_

#include <tmc_rplanner/configuration_space.hpp>
#include <tmc_rplanner/path_short_cutter.hpp>

namespace tmc_rplanner {

/// @class RaundRobinShorCutter
/// @brief Shortcut by a brute force
class RoundRobinShortCutter : public IPathShortCutter {
 public:
  /// @brief Give the planner space
  /// @param space Configuration space
  /// @param delta Exploration carved width
  /// @param bidirectional Shortcut in both directions
  /// @param skip When shortcuts, skip the node with this value
  /// @param is_terminate termination conditions
  RoundRobinShortCutter(ConfigurationSpace::Ptr space,
                        double delta,
                        bool bidirectional,
                        int32_t skip,
                        TerminateConditionFunc is_terminate) :
      space_(space), delta_(delta),
      bidirectional_(bidirectional), skip_(skip),
      is_terminate_(is_terminate) {}
  /// @brief Give the planner space
  /// @param space Configuration space
  /// @param delta Exploration carved width
  /// @param bidirectional Shortcut in both directions
  /// @param skip When shortcuts, skip the node with this value
  RoundRobinShortCutter(ConfigurationSpace::Ptr space,
                        double delta,
                        bool bidirectional,
                        int32_t skip) :
      space_(space), delta_(delta),
      bidirectional_(bidirectional), skip_(skip) {}

  ~RoundRobinShortCutter() {}
  /// Shorter the path
  virtual bool ShortCut(const Path& path_in, Path& path_out);

 private:
  // Copy prohibition
  RoundRobinShortCutter(const RoundRobinShortCutter&);
  RoundRobinShortCutter& operator=(const RoundRobinShortCutter&);
  bool ShortCutOnce_(int32_t skip, const Path& path_in, Path& path_out) const;
  bool ShortCutBiDirectional_(int32_t skip,
                              const Path& path_in, Path& path_out) const;

  const ConfigurationSpace::Ptr space_;
  const double delta_;
  const bool bidirectional_;
  const int32_t skip_;
  TerminateConditionFunc is_terminate_;
};
}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_ROUND_ROBIN_SHORT_CUTTER_HPP_
