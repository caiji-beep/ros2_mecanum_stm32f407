#include <gmock/gmock.h>
#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mecanum_hw_interface/mecanum_hw_interface.hpp"

class TestMecanumHWSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 4DOF Mecanum Hardware Interface
    mecanum_hw_interface_4dof_ =
      R"(
        <ros2_control name="MecanumHWSystem4dof" type="system">
          <hardware>
            <plugin>mecanum_hw_interface/MecanumHWSystem</plugin>
          </hardware>

          <!-- Front Left Wheel -->
          <joint name="left_front_wheel_joint">
            <command_interface name="velocity"/>
            <state_interface name="velocity"/>
            <param name="initial_position">0.0</param>
          </joint>

          <!-- Front Right Wheel -->
          <joint name="right_front_wheel_joint">
            <command_interface name="velocity"/>
            <state_interface name="velocity"/>
            <param name="initial_position">0.0</param>
          </joint>

          <!-- Rear Left Wheel -->
          <joint name="left_back_wheel_joint">
            <command_interface name="velocity"/>
            <state_interface name="velocity"/>
            <param name="initial_position">0.0</param>
          </joint>

          <!-- Rear Right Wheel -->
          <joint name="right_back_wheel_joint">
            <command_interface name="velocity"/>
            <state_interface name="velocity"/>
            <param name="initial_position">0.0</param>
          </joint>
        </ros2_control>
      )";
  }

  std::string mecanum_hw_interface_4dof_;
};

TEST_F(TestMecanumHWSystem, load_mecanum_hw_interface_4dof)
{
  // Create URDF combining with robot description
  auto urdf = ros2_control_test_assets::urdf_head + mecanum_hw_interface_4dof_ +
              ros2_control_test_assets::urdf_tail;
  
  // Test loading the hardware interface
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
