// Copyright (c) 2026 JanKim
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

#include <climits>
#include <limits>

#include "gtest/gtest.h"

extern "C"
{
#include "nav2_amcl/pf/pf.hpp"
}

TEST(PfKdTreeTest, RejectsInvalidPose)
{
  pf_kdtree_t * tree = pf_kdtree_alloc(10);
  auto pose = pf_vector_zero();
  pose.v[0] = std::numeric_limits<double>::infinity();

  pf_kdtree_insert(tree, pose, 1.0);

  EXPECT_EQ(tree->node_count, 0);
  pf_kdtree_free(tree);
}

TEST(PfKdTreeTest, HandlesNeighborAtIntegerBoundary)
{
  pf_kdtree_t * tree = pf_kdtree_alloc(10);
  auto pose = pf_vector_zero();
  pose.v[0] = static_cast<double>(INT_MIN) * tree->size[0];

  pf_kdtree_insert(tree, pose, 1.0);
  pf_kdtree_cluster(tree);

  EXPECT_EQ(tree->root->cluster, 0);
  pf_kdtree_free(tree);
}
