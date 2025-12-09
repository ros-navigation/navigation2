#include "nav2_costmap_2d/inflation_layer.hpp"
#include "gtest/gtest.h"

namespace nav2_costmap_2d
{

class TestPredictiveLayer : public InflationLayer
{
public:
  // Test access to protected members
  void testProtectedAccess()
  {
    // Configuration
    EXPECT_GE(inflation_radius_, 0.0); 
    EXPECT_GE(cost_scaling_factor_, 0.0);
    EXPECT_GE(inscribed_radius_, 0.0);
    // inflate_unknown_ is false by default in constructor
    EXPECT_FALSE(inflate_unknown_);
    // inflate_around_unknown_ is false by default in constructor
    EXPECT_FALSE(inflate_around_unknown_);
    
    // State
    // need_reinflation_ is initialized in onInitialize usually, but in constructor?
    // Let's check the header/source again.
    // In constructor: need_reinflation_ is NOT initialized in initializer list in the file I read!
    // Wait, let me check the file content I read earlier.
    // Line 62: InflationLayer::InflationLayer() ...
    // It does NOT initialize need_reinflation_ in the constructor body or list in the provided snippet.
    // It IS initialized in onInitialize() to false (Line 119).
    // So checking it before onInitialize might be UB if it's not initialized in header.
    // In header: bool need_reinflation_; (Line 294). No default value.
    // So I should NOT check need_reinflation_ before initialization.
    
    // Cache access
    EXPECT_TRUE(cached_costs_.empty());
    EXPECT_TRUE(cached_distances_.empty());
  }

  // Test access to protected methods
  void testMethodAccess()
  {
    // Access mutex
    auto* mutex = getMutex();
    EXPECT_NE(mutex, nullptr);
    
    // Access computeCost (public inline)
    unsigned char c = computeCost(0.0);
    EXPECT_EQ(c, LETHAL_OBSTACLE);
  }

  // Override virtual methods
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override
  {
    InflationLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
  }

  void onInitialize() override
  {
      InflationLayer::onInitialize();
  }
};

TEST(InflationLayerExtensibilityTest, InheritanceAndAccess)
{
  TestPredictiveLayer layer;
  layer.testProtectedAccess();
  layer.testMethodAccess();
  
  // Verify we can cast to Layer*
  Layer* base_ptr = &layer;
  EXPECT_NE(base_ptr, nullptr);
}

}  // namespace nav2_costmap_2d

