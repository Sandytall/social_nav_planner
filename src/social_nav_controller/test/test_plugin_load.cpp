// Proves the controller is a real, pluginlib-loadable nav2_core::Controller. This
// instantiates the class through the same ClassLoader Nav2's controller server uses -
// not a mock.
#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>

#include "nav2_core/controller.hpp"

TEST(PluginLoad, SocialNavControllerIsDiscoverableAndInstantiable)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");

  bool found = false;
  for (const auto & name : loader.getDeclaredClasses()) {
    if (name == "social_nav_controller/SocialNavController") {
      found = true;
      break;
    }
  }
  ASSERT_TRUE(found) << "plugin not declared in the nav2_core class index";

  auto controller = loader.createSharedInstance("social_nav_controller/SocialNavController");
  EXPECT_NE(controller, nullptr);
}
