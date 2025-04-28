#ifndef ROBOT_SIMULATION_H
#define ROBOT_SIMULATION_H

#include <vector>
#include <stdexcept> // For exceptions
#include <cmath>     // For std::abs, std::copysign

// --- Constants ---
constexpr int NUM_AXES = 3; // Define the number of axes

// --- Enumerations ---
enum class GripperState {
    OPEN,
    CLOSED,
    // Add MOVING if you need to simulate transition time later
};

// --- Structures ---

// Configuration for a single axis
struct AxisConfig {
    double maxLength = 1.0;        // Maximum position (e.g., meters) - Minimum is always 0
    double maxSpeed = 0.5;         // Maximum absolute speed (e.g., m/s)
    double maxAcceleration = 2.0;  // Maximum absolute acceleration (e.g., m/s^2)
    double dampingFactor = 0.1;    // Factor for velocity reduction when acceleration is zero (simulates friction/drag)
};

// Dynamic state of a single axis
struct AxisState {
    double position = 0.0;
    double velocity = 0.0;
    double commandedAcceleration = 0.0; // The acceleration set by the user
    bool endSwitchMinActive = true;     // Active when position is at or below 0
    bool endSwitchMaxActive = false;    // Active when position is at or above maxLength
};


// --- Class Definition ---

class RobotSimulation {
public:
    /**
     * @brief Constructor for the RobotSimulation class.
     * @param configs A vector containing the configuration for each of the NUM_AXES axes.
     * @param initialGripperState The initial state of the gripper.
     * @throws std::invalid_argument if the size of configs does not match NUM_AXES.
     */
    RobotSimulation(const std::vector<AxisConfig>& configs,
                    GripperState initialGripperState = GripperState::OPEN);

    /**
     * @brief Updates the state of all axes and the gripper based on the time elapsed.
     * @param dtSeconds The time step duration in seconds.
     */
    void update(double dtSeconds);

    /**
     * @brief Sets the desired acceleration for a specific axis.
     * The actual acceleration applied will be clamped by the axis's maxAcceleration limit.
     * Setting acceleration to 0 will cause the axis to decelerate due to damping.
     * @param axisIndex The index of the axis (0 to NUM_AXES - 1).
     * @param acceleration The desired acceleration (e.g., m/s^2).
     * @throws std::out_of_range if axisIndex is invalid.
     */
    void setAxisAcceleration(int axisIndex, double acceleration);

    /**
     * @brief Sets the state of the gripper.
     * Currently assumes instantaneous change.
     * @param newState The desired GripperState.
     */
    void setGripperState(GripperState newState);

    // --- Getters ---

    /**
     * @brief Gets the current position of a specific axis.
     * @param axisIndex The index of the axis (0 to NUM_AXES - 1).
     * @return The current position (e.g., meters).
     * @throws std::out_of_range if axisIndex is invalid.
     */
    double getPosition(int axisIndex) const;

    /**
     * @brief Gets the current velocity of a specific axis.
     * @param axisIndex The index of the axis (0 to NUM_AXES - 1).
     * @return The current velocity (e.g., m/s).
     * @throws std::out_of_range if axisIndex is invalid.
     */
    double getVelocity(int axisIndex) const;

    /**
     * @brief Gets the currently commanded acceleration for a specific axis.
     * Note: This is the value set via setAxisAcceleration, not necessarily the
     * acceleration currently being experienced if limits are hit.
     * @param axisIndex The index of the axis (0 to NUM_AXES - 1).
     * @return The commanded acceleration (e.g., m/s^2).
     * @throws std::out_of_range if axisIndex is invalid.
     */
    double getCommandedAcceleration(int axisIndex) const;


    /**
     * @brief Checks if the minimum end switch for a specific axis is active.
     * @param axisIndex The index of the axis (0 to NUM_AXES - 1).
     * @return True if the axis is at or below the minimum position (0), false otherwise.
     * @throws std::out_of_range if axisIndex is invalid.
     */
    bool isEndSwitchMinActive(int axisIndex) const;

    /**
     * @brief Checks if the maximum end switch for a specific axis is active.
     * @param axisIndex The index of the axis (0 to NUM_AXES - 1).
     * @return True if the axis is at or above the maximum position, false otherwise.
     * @throws std::out_of_range if axisIndex is invalid.
     */
    bool isEndSwitchMaxActive(int axisIndex) const;

    /**
     * @brief Gets the current state of the gripper.
     * @return The current GripperState.
     */
    GripperState getGripperState() const;


private:
    // --- Member Variables ---
    AxisConfig configs_[NUM_AXES];     // Configuration for each axis
    AxisState states_[NUM_AXES];       // Dynamic state for each axis
    GripperState gripperState_;        // Current state of the gripper

    // --- Helper Functions ---

    /**
     * @brief Clamps a value between a minimum and maximum limit.
     * @param value The value to clamp.
     * @param minVal The minimum allowed value.
     * @param maxVal The maximum allowed value.
     * @return The clamped value.
     */
    double clamp(double value, double minVal, double maxVal) const;

    /**
    * @brief Validates the axis index.
    * @param axisIndex The index to validate.
    * @throws std::out_of_range if axisIndex is invalid.
    */
    void validateAxisIndex(int axisIndex) const;
};

#endif // ROBOT_SIMULATION_H
