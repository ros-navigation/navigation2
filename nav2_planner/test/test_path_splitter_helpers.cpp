// Copyright (c) 2026 Origin
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

#include <gtest/gtest.h>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/classified_path.hpp"
#include "nav2_msgs/msg/classified_path_array.hpp"
#include "nav2_planner/path_splitter.hpp"

// ---------------------------------------------------------------------------
// Test fixture — declared as friend in PathSplitter.
// ---------------------------------------------------------------------------

namespace nav2_planner
{

class PathSplitterHelperTest : public ::testing::Test
{
protected:
  PathSplitter splitter_;

  // Access the private Segment type
  using Segment = PathSplitter::Segment;

  // Wrappers that forward to private methods
  void mergeShortSegments(std::vector<Segment> & segments)
  {
    splitter_.mergeShortSegments(segments);
  }

  nav2_msgs::msg::ClassifiedPathArray buildResult(
    const nav_msgs::msg::Path & path,
    const std::vector<Segment> & segments)
  {
    return splitter_.buildResult(path, segments);
  }

  void setMinSegmentPoses(int val)
  {
    splitter_.min_segment_poses_ = val;
  }

  // Helper to build a path with n poses at x = 0, 1, 2, ...
  static nav_msgs::msg::Path makePath(size_t n)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (size_t i = 0; i < n; ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.pose.position.x = static_cast<double>(i);
      ps.pose.position.y = 0.0;
      ps.pose.position.z = 0.0;
      path.poses.push_back(ps);
    }
    return path;
  }
};

// ===========================================================================
// mergeShortSegments tests
// ===========================================================================

TEST_F(PathSplitterHelperTest, MergeSingleSegmentNoOp)
{
  // A single segment should never be merged regardless of length
  std::vector<Segment> segs = {{0, 0, 2}};
  setMinSegmentPoses(5);
  mergeShortSegments(segs);

  ASSERT_EQ(segs.size(), 1u);
  EXPECT_EQ(segs[0].start_idx, 0u);
  EXPECT_EQ(segs[0].end_idx, 2u);
}

TEST_F(PathSplitterHelperTest, MergeAllLongEnoughNoOp)
{
  // All segments are >= min_segment_poses, nothing to merge
  std::vector<Segment> segs = {
    {0, 0, 10},
    {1, 10, 20},
    {0, 20, 30}
  };
  setMinSegmentPoses(5);
  mergeShortSegments(segs);

  ASSERT_EQ(segs.size(), 3u);
  EXPECT_EQ(segs[0].end_idx, 10u);
  EXPECT_EQ(segs[1].start_idx, 10u);
  EXPECT_EQ(segs[2].start_idx, 20u);
}

TEST_F(PathSplitterHelperTest, MergeShortMiddleIntoLeftNeighbor)
{
  // Middle segment (length 2) is too short, should merge into left neighbor
  std::vector<Segment> segs = {
    {0, 0, 10},   // length 10
    {1, 10, 12},  // length 2 — short
    {0, 12, 22}   // length 10
  };
  setMinSegmentPoses(5);
  mergeShortSegments(segs);

  ASSERT_EQ(segs.size(), 2u);
  // Left neighbor absorbs the short segment
  EXPECT_EQ(segs[0].start_idx, 0u);
  EXPECT_EQ(segs[0].end_idx, 12u);
  EXPECT_EQ(segs[0].class_type, 0u);
  // Right segment unchanged
  EXPECT_EQ(segs[1].start_idx, 12u);
  EXPECT_EQ(segs[1].end_idx, 22u);
}

TEST_F(PathSplitterHelperTest, MergeShortFirstIntoRightNeighbor)
{
  // First segment is too short, should merge into right neighbor
  std::vector<Segment> segs = {
    {1, 0, 2},    // length 2 — short
    {0, 2, 12}    // length 10
  };
  setMinSegmentPoses(5);
  mergeShortSegments(segs);

  ASSERT_EQ(segs.size(), 1u);
  EXPECT_EQ(segs[0].start_idx, 0u);
  EXPECT_EQ(segs[0].end_idx, 12u);
  // Right neighbor's class type is preserved
  EXPECT_EQ(segs[0].class_type, 0u);
}

TEST_F(PathSplitterHelperTest, MergeCascadingShortSegments)
{
  // Multiple short segments that cascade into merges
  std::vector<Segment> segs = {
    {0, 0, 10},   // length 10
    {1, 10, 12},  // length 2 — short
    {2, 12, 14},  // length 2 — short
    {0, 14, 24}   // length 10
  };
  setMinSegmentPoses(5);
  mergeShortSegments(segs);

  // Both short segments should be absorbed
  ASSERT_EQ(segs.size(), 2u);
  EXPECT_EQ(segs[0].end_idx, 14u);
  EXPECT_EQ(segs[1].start_idx, 14u);
}

TEST_F(PathSplitterHelperTest, MergeAllShortExceptOne)
{
  // Everything merges into the one long segment
  std::vector<Segment> segs = {
    {1, 0, 2},    // length 2
    {0, 2, 20},   // length 18
    {1, 20, 22}   // length 2
  };
  setMinSegmentPoses(5);
  mergeShortSegments(segs);

  ASSERT_EQ(segs.size(), 1u);
  EXPECT_EQ(segs[0].start_idx, 0u);
  EXPECT_EQ(segs[0].end_idx, 22u);
}

TEST_F(PathSplitterHelperTest, MergeEmptySegmentsVector)
{
  std::vector<Segment> segs;
  setMinSegmentPoses(5);
  mergeShortSegments(segs);

  EXPECT_TRUE(segs.empty());
}

TEST_F(PathSplitterHelperTest, MergeMinSegmentPosesOne)
{
  // With min_segment_poses=1, even a single-pose segment survives
  std::vector<Segment> segs = {
    {0, 0, 1},
    {1, 1, 2},
    {0, 2, 3}
  };
  setMinSegmentPoses(1);
  mergeShortSegments(segs);

  ASSERT_EQ(segs.size(), 3u);
}

TEST_F(PathSplitterHelperTest, MergePreservesCoverage)
{
  // After any merge, the total range [first.start_idx, last.end_idx) is preserved
  std::vector<Segment> segs = {
    {0, 0, 10},
    {1, 10, 12},
    {2, 12, 13},
    {0, 13, 25}
  };
  setMinSegmentPoses(5);

  size_t original_start = segs.front().start_idx;
  size_t original_end = segs.back().end_idx;

  mergeShortSegments(segs);

  EXPECT_EQ(segs.front().start_idx, original_start);
  EXPECT_EQ(segs.back().end_idx, original_end);

  // Segments should be contiguous
  for (size_t i = 1; i < segs.size(); ++i) {
    EXPECT_EQ(segs[i].start_idx, segs[i - 1].end_idx)
      << "Gap between segment " << i - 1 << " and " << i;
  }
}

// ===========================================================================
// buildResult tests
// ===========================================================================

TEST_F(PathSplitterHelperTest, BuildResultSingleSegment)
{
  auto path = makePath(10);
  std::vector<Segment> segs = {{0, 0, 10}};

  auto result = buildResult(path, segs);

  ASSERT_EQ(result.paths.size(), 1u);
  EXPECT_EQ(result.paths[0].class_type, 0u);
  // Single segment, no overlap needed — gets all 10 poses
  EXPECT_EQ(result.paths[0].path.poses.size(), 10u);
  EXPECT_EQ(result.paths[0].path.header.frame_id, "map");
}

TEST_F(PathSplitterHelperTest, BuildResultTwoSegmentsWithOverlap)
{
  auto path = makePath(20);
  std::vector<Segment> segs = {
    {0, 0, 10},
    {1, 10, 20}
  };

  auto result = buildResult(path, segs);

  ASSERT_EQ(result.paths.size(), 2u);

  // First segment: 10 poses + 1 overlap = 11
  EXPECT_EQ(result.paths[0].path.poses.size(), 11u);
  EXPECT_EQ(result.paths[0].class_type, 0u);

  // The overlap pose should be the first pose of the next segment
  EXPECT_NEAR(
    result.paths[0].path.poses.back().pose.position.x,
    10.0, 1e-9);

  // Second segment: 10 poses, no overlap (last segment)
  EXPECT_EQ(result.paths[1].path.poses.size(), 10u);
  EXPECT_EQ(result.paths[1].class_type, 1u);
}

TEST_F(PathSplitterHelperTest, BuildResultThreeSegmentsOverlapAtEachBoundary)
{
  auto path = makePath(30);
  std::vector<Segment> segs = {
    {0, 0, 10},
    {1, 10, 20},
    {2, 20, 30}
  };

  auto result = buildResult(path, segs);

  ASSERT_EQ(result.paths.size(), 3u);

  // First: 10 + 1 overlap = 11
  EXPECT_EQ(result.paths[0].path.poses.size(), 11u);
  // Middle: 10 + 1 overlap = 11
  EXPECT_EQ(result.paths[1].path.poses.size(), 11u);
  // Last: 10, no overlap
  EXPECT_EQ(result.paths[2].path.poses.size(), 10u);
}

TEST_F(PathSplitterHelperTest, BuildResultClassTypePropagated)
{
  auto path = makePath(20);
  std::vector<Segment> segs = {
    {42, 0, 10},
    {7, 10, 20}
  };

  auto result = buildResult(path, segs);

  EXPECT_EQ(result.paths[0].class_type, 42u);
  EXPECT_EQ(result.paths[1].class_type, 7u);
}

TEST_F(PathSplitterHelperTest, BuildResultHeaderPropagated)
{
  auto path = makePath(10);
  path.header.frame_id = "odom";
  std::vector<Segment> segs = {{0, 0, 10}};

  auto result = buildResult(path, segs);

  EXPECT_EQ(result.paths[0].path.header.frame_id, "odom");
}

TEST_F(PathSplitterHelperTest, BuildResultPosesAreCorrect)
{
  auto path = makePath(10);
  std::vector<Segment> segs = {
    {0, 0, 5},
    {1, 5, 10}
  };

  auto result = buildResult(path, segs);

  // First segment poses should be x = 0,1,2,3,4 + overlap at x=5
  ASSERT_EQ(result.paths[0].path.poses.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(
      result.paths[0].path.poses[i].pose.position.x,
      static_cast<double>(i), 1e-9)
      << "First segment pose " << i << " wrong";
  }

  // Second segment poses should be x = 5,6,7,8,9
  ASSERT_EQ(result.paths[1].path.poses.size(), 5u);
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_NEAR(
      result.paths[1].path.poses[i].pose.position.x,
      static_cast<double>(i + 5), 1e-9)
      << "Second segment pose " << i << " wrong";
  }
}

TEST_F(PathSplitterHelperTest, BuildResultEmptySegments)
{
  auto path = makePath(10);
  std::vector<Segment> segs;

  auto result = buildResult(path, segs);

  EXPECT_TRUE(result.paths.empty());
}

// ===========================================================================
// splitPath edge cases (public API, no classifier needed for empty path)
// ===========================================================================

TEST_F(PathSplitterHelperTest, SplitEmptyPathReturnsEmpty)
{
  nav_msgs::msg::Path empty_path;
  // We need a PoseClassifier but it won't be called for empty path
  // Use a trick: create one without configuring — splitPath returns early
  PoseClassifier dummy_classifier;

  auto result = splitter_.splitPath(empty_path, dummy_classifier, true);

  EXPECT_TRUE(result.classified_poses.empty());
  EXPECT_TRUE(result.classified_path_array.paths.empty());
}

}  // namespace nav2_planner

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
