#ifndef ESTOP_H
#define ESTOP_H

#include <sensor_msgs/msg/joy.hpp>

/**
 * @brief Emergency stop and manual control class for joystick teleoperation
 *
 * This class provides joystick-based control for emergency stop and manual
 * override of autonomous navigation. Useful for testing and safety.
 *
 * Usage:
 * - Left stick (axes[1]): Forward/backward linear velocity
 * - Left stick (axes[0]): Left/right angular velocity
 * - Button press: Emergency stop or resume
 */
class eStop
{
public:
    /**
     * @brief Constructor
     */
    eStop()
        : linear(0.0)
        , angular(0.0)
        , state(0)
    {
    }

    /**
     * @brief Joystick callback for ROS 2
     * @param joy Shared pointer to Joy message
     */
    void controllerCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        // Scale joystick inputs to velocity commands
        // axes[1] is typically the left stick vertical (forward/backward)
        // axes[0] is typically the left stick horizontal (left/right)
        linear = 0.2 * joy->axes[1];    // Max linear velocity: 0.2 m/s
        angular = 1.2 * joy->axes[0];   // Max angular velocity: 1.2 rad/s

        // Button handling for state changes
        // buttons[0] is typically the 'A' button on Xbox controller
        if (!joy->buttons.empty() && joy->buttons[0] == 1) {
            state = 1 - state; // Toggle state (emergency stop on/off)
        }

        // If emergency stop is active (state == 0), set velocities to zero
        if (state == 0) {
            linear = 0.0;
            angular = 0.0;
        }
    }

    /**
     * @brief Get current linear velocity
     * @return Linear velocity in m/s
     */
    double getLinear() const { return linear; }

    /**
     * @brief Get current angular velocity
     * @return Angular velocity in rad/s
     */
    double getAngular() const { return angular; }

    /**
     * @brief Get emergency stop state
     * @return 0 if stopped, 1 if active
     */
    int getState() const { return state; }

private:
    double linear;   ///< Linear velocity command
    double angular;  ///< Angular velocity command
    int state;       ///< Emergency stop state (0=stopped, 1=active)
};

#endif // ESTOP_H
