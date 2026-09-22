// Proves both cost layers are real, pluginlib-loadable nav2_costmap_2d::Layer plugins,
// instantiated through the same ClassLoader Nav2's costmap uses - not a mock. Loading
// RestrictedZoneLayer here also guards that adding it did not break SocialLayer's export.
#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include <pluginlib/class_loader.hpp>

#include "nav2_costmap_2d/layer.hpp"

TEST(PluginLoad, BothLayersAreDiscoverableAndInstantiable)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");

  const auto declared = loader.getDeclaredClasses();
  auto has = [&declared](const std::string & name) {
      return std::find(declared.begin(), declared.end(), name) != declared.end();
    };

  ASSERT_TRUE(has("social_nav_costs/RestrictedZoneLayer"))
    << "RestrictedZoneLayer not declared in the nav2_costmap_2d class index";
  ASSERT_TRUE(has("social_nav_costs/SocialLayer"))
    << "SocialLayer export regressed when RestrictedZoneLayer was added";

  EXPECT_NE(loader.createSharedInstance("social_nav_costs/RestrictedZoneLayer"), nullptr);
  EXPECT_NE(loader.createSharedInstance("social_nav_costs/SocialLayer"), nullptr);
}
