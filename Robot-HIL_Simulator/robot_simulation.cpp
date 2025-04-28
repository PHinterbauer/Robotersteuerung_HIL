#include "robot_simulation.h"
#include <algorithm> // For std::min, std::max
#include <cmath>     // For std::abs, std::copysign

// --- Constructor ---

RobotSimulation::RobotSimulation(const std::vector<AxisConfig>& configs, GripperState initialGripperState)
    : gripperState_(initialGripperState) {
    // Validate input size
    if (configs.size() != NUM_AXES) {
        printf("Configuration vector size must match NUM_AXES.");
    }

    // Copy configuration and initialize state for each axis
    for (int i = 0; i < NUM_AXES; ++i) {
        configs_[i] = configs[i];

        // Ensure config values are valid (e.g., non-negative limits)
        if (configs_[i].maxLength <= 0 || configs_[i].maxSpeed < 0 || configs_[i].maxAcceleration < 0 || configs_[i].dampingFactor < 0) {
             printf("Axis configuration values (length, speed, acceleration, damping) must be non-negative.");
        }
        // Initial state setup
        states_[i].position = 0.0; // Start at the minimum limit
        states_[i].velocity = 0.0;
        states_[i].commandedAcceleration = 0.0;
        states_[i].endSwitchMinActive = true; // Start at min limit
        states_[i].endSwitchMaxActive = false;
    }
}

// --- Update Function ---

void RobotSimulation::update(double dtSeconds) {
    if (dtSeconds <= 0) {
        // No time elapsed or invalid time step
        return;
    }

    for (int i = 0; i < NUM_AXES; ++i) {
        AxisConfig& config = configs_[i];
        AxisState& state = states_[i];

        // 1. Determine Actual Acceleration
        double actualAcceleration = clamp(state.commandedAcceleration, -config.maxAcceleration, config.maxAcceleration);

        // Apply damping if commanded acceleration is near zero (simulates friction/drag)
        // This causes the axis to slow down naturally when not actively accelerating/decelerating
        if (std::abs(state.commandedAcceleration) < 1e-6) { // Use a small tolerance
             // Damping force opposes velocity, proportional to velocity and damping factor
             actualAcceleration -= state.velocity * config.dampingFactor;
             // Re-clamp acceleration just in case damping made it exceed limits (unlikely but safe)
             actualAcceleration = clamp(actualAcceleration, -config.maxAcceleration, config.maxAcceleration);
        }


        // 2. Update Velocity: v = v0 + a * dt
        double previousVelocity = state.velocity; // Store for position calculation if needed
        state.velocity += actualAcceleration * dtSeconds;

        // 3. Clamp Velocity: Apply max speed limit
        state.velocity = clamp(state.velocity, -config.maxSpeed, config.maxSpeed);

        // 4. Update Position: x = x0 + v_avg * dt (using average velocity for better accuracy)
        // double averageVelocity = (previousVelocity + state.velocity) / 2.0;
        // state.position += averageVelocity * dtSeconds;
        // Simpler Euler integration (often sufficient for small dt): x = x0 + v * dt
        state.position += state.velocity * dtSeconds;


        // 5. Check and Apply Position Limits (0 to maxLength)
        bool hitMinLimit = false;
        bool hitMaxLimit = false;

        if (state.position <= 0.0) {
            state.position = 0.0;
            hitMinLimit = true;
        } else if (state.position >= config.maxLength) {
            state.position = config.maxLength;
            hitMaxLimit = true;
        }

        // If a limit was hit, stop motion immediately
        if (hitMinLimit || hitMaxLimit) {
            state.velocity = 0.0;
            // Optionally reset commanded acceleration if you want it to stop trying
            // state.commandedAcceleration = 0.0;
        }

        // 6. Update End Switch Status
        // Use a small tolerance to avoid floating point issues at the exact limits
        double tolerance = 1e-6;
        state.endSwitchMinActive = (state.position <= 0.0 + tolerance);
        state.endSwitchMaxActive = (state.position >= config.maxLength - tolerance);
    }
    // Gripper state is updated instantly via setGripperState for now
}

// --- Setters ---

void RobotSimulation::setAxisAcceleration(int axisIndex, double acceleration) {
    validateAxisIndex(axisIndex);
    // Store the commanded acceleration. Clamping happens during update.
    states_[axisIndex].commandedAcceleration = acceleration;
}

void RobotSimulation::setGripperState(GripperState newState) {
    // Add validation or transition logic here if needed later
    gripperState_ = newState;
}

// --- Getters ---

double RobotSimulation::getPosition(int axisIndex) const {
    validateAxisIndex(axisIndex);
    return states_[axisIndex].position;
}

double RobotSimulation::getVelocity(int axisIndex) const {
    validateAxisIndex(axisIndex);
    return states_[axisIndex].velocity;
}

double RobotSimulation::getCommandedAcceleration(int axisIndex) const {
    validateAxisIndex(axisIndex);
    return states_[axisIndex].commandedAcceleration;
}

bool RobotSimulation::isEndSwitchMinActive(int axisIndex) const {
    validateAxisIndex(axisIndex);
    return states_[axisIndex].endSwitchMinActive;
}

bool RobotSimulation::isEndSwitchMaxActive(int axisIndex) const {
    validateAxisIndex(axisIndex);
    return states_[axisIndex].endSwitchMaxActive;
}

GripperState RobotSimulation::getGripperState() const {
    return gripperState_;
}


// --- Helper Functions ---

double RobotSimulation::clamp(double value, double minVal, double maxVal) const {
    return std::max(minVal, std::min(value, maxVal));
}

void RobotSimulation::validateAxisIndex(int axisIndex) const {
     if (axisIndex < 0 || axisIndex >= NUM_AXES) {
        printf("Invalid axis index provided.");
    }
}
