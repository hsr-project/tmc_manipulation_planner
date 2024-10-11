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
/// @brief    Planning parameter structure

#include <tmc_rplanner/planner_common.hpp>

namespace tmc_rplanner {

/// @func TreeToPath
/// @brief Extract the track from the state tree
///        Suppose the end of the state tree is the goal
///        If the pass is larger than the size of TREE, the loop
//         Throw an exception because it is
void TreeToPath(const Tree& tree, Path& path_out) {
  TreeToPath(tree, tree.size() - 1, path_out);
}

/// @func TreeToPath
/// @brief Extract the track from the state tree
///        Suppose the end of the state tree is the goal
///        If the pass is larger than the size of TREE, the loop
///        Throw an exception because it is
void TreeToPath(const Tree& tree, uint32_t goal_index, Path& path_out) {
  if (tree.size() <= goal_index) {
    throw std::invalid_argument("Goal index over tree size.");
  }
  Node::WeakPtr node = tree[goal_index];
  path_out.clear();
  Config data;
  while (!node.lock()->parent.expired()) {
    data = node.lock()->data;
    path_out.push_front(data);
    node = node.lock()->parent;
    if (path_out.size() > tree.size()) {
      throw tmc_rplanner::TreeLoop("Configuration Tree is looped!");
    }
  }
  data = node.lock()->data;
  path_out.push_front(data);
}

/// Swap Root
void SwapRoot(Tree& tree, const Node::WeakPtr& root) {
  if (!root.lock()->parent.expired()) {
    SwapRoot(tree, root.lock()->parent);
    root.lock()->parent.lock()->parent = root;
  }
}

/// @brief Change the root of the state tree to the specified one
void ChangeTreeRoot(Tree& tree, const Node::WeakPtr& root) {
  if (!root.lock()->parent.expired()) {
    SwapRoot(tree, root);
  }
  root.lock()->parent = Node::WeakPtr();
}


// end of namespace tmc_rplanner
}  // namespace tmc_rplanner
