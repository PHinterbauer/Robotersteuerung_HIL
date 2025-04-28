#include <stdio.h>
#include <stdlib.h> // For abs
#include <math.h>   // For fmod, floor, fabsf, fmaxf, fminf - some might be removable

// Pico SDK Headers
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h" // For clock frequency
#include "hardware/i2c.h"    // Added for I2C
#include "pico/i2c_slave.h" // Use the I2C slave helper library
#include "hardware/irq.h"
#include "hardware/sync.h"   // For save_and_disable_interrupts
#include "hardware/pio.h"    // Include PIO header

// FreeRTOS Headers
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" // For mutexes (optional but good practice)

// Include the generated PIO header
#include "pwm_measure.pio.h"
#include "pwm_in.h"

// --- Include the C++ Robot Simulation Class Header ---
// Note: Ensure your build system links RobotSimulation.cpp
#include "robot_simulation.h"

// --- Configuration ---
const uint SERIAL_BAUD_RATE = 115200;
const uint HIL_UPDATE_RATE_MS = 10;   // How often to run the simulation loop (e.g., 10ms = 100Hz)
const float HIL_UPDATE_RATE_S = (float)HIL_UPDATE_RATE_MS / 1000.0f; // dt in seconds

// --- I2C Peripheral Configuration ---
#define I2C_PERIPHERAL_INSTANCE i2c0
#define I2C_PERIPHERAL_ADDR     0x30
#define I2C_PIN_SDA             4
#define I2C_PIN_SCL             5
#define I2C_BAUDRATE            (100*1000)
#define I2C_REG_X_POS_START     0x00
#define I2C_REG_Y_POS_START     0x04
#define I2C_REG_Z_POS_START     0x08
// #define I2C_REG_X_VEL_START     0x0C // Example for adding velocity
// #define I2C_REG_Y_VEL_START     0x10
// #define I2C_REG_Z_VEL_START     0x14
// #define I2C_REG_GRIPPER_STATE   0x18 // Example for adding gripper state
#define I2C_RAM_BUFFER_SIZE     256

// --- GPIO Definitions ---
// Axis X Pins
#define PIN_X_PWM       0
#define PWM_X_CHANNEL   0 // PIO SM index
#define PIN_X_DIR       1
#define PIN_X_END_MIN   2
#define PIN_X_END_MAX   3
// Axis Y Pins
#define PIN_Y_PWM       6
#define PWM_Y_CHANNEL   1 // PIO SM index
#define PIN_Y_DIR       7
#define PIN_Y_END_MIN   8
#define PIN_Y_END_MAX   9
// Axis Z Pins
#define PIN_Z_PWM       12
#define PWM_Z_CHANNEL   2 // PIO SM index
#define PIN_Z_DIR       13
#define PIN_Z_END_MIN   14
#define PIN_Z_END_MAX   15
// Gripper Pins
#define PIN_GRIP_CMD_OPEN  18
#define PIN_GRIP_CMD_CLOSE 19
#define PIN_GRIP_SNS_OPEN  20
#define PIN_GRIP_SNS_CLOSED 21
// Onboard LED
#ifdef PICO_DEFAULT_LED_PIN
#define PIN_LED PICO_DEFAULT_LED_PIN
#endif

// --- Simulation Parameters (Moved to RobotSimulation initialization) ---
// Removed old constants like AXIS_*_LIMIT_*, PWM_TO_SPEED_FACTOR etc.

// --- Global Robot Simulation Object ---
// Create Axis Configurations based on previous parameters
AxisConfig axis_configs[NUM_AXES] = {
    // Axis X (Index 0)
    { .maxLength = 100.0, .maxSpeed = 80.0, .maxAcceleration = 50.0, .dampingFactor = 0.1 }, // Set length relative to 0 (0 to 100)
    // Axis Y (Index 1)
    { .maxLength = 100.0, .maxSpeed = 80.0, .maxAcceleration = 50.0, .dampingFactor = 0.1 }, // Set length relative to 0 (0 to 100)
    // Axis Z (Index 2)
    { .maxLength = 150.0, .maxSpeed = 80.0, .maxAcceleration = 50.0, .dampingFactor = 0.1 }  // Set length relative to 0 (0 to 150)
};

// Instantiate the RobotSimulation object globally
// The constructor needs to be called from C++, typically via an init function
// or by ensuring global constructors are run. For simplicity with C main:
RobotSimulation* g_robot_simulation = nullptr; // Pointer to the object

// --- I2C Peripheral State --- (Unchanged)
static uint8_t i2c_ram_buffer[I2C_RAM_BUFFER_SIZE];
static volatile uint8_t i2c_ram_address = 0;
static volatile bool i2c_address_set = false;

// --- PIO PWM Input Reading --- (Unchanged)
PwmIn* p_PwmInput;


// --- Helper Functions ---

/**
 * @brief Sends the full simulated robot state over serial USB.
 * Format: Xp,Yp,Zp,Xv,Yv,Zv,Xa,Ya,Za,Xmin,Xmax,Ymin,Ymax,Zmin,Zmax,Grip
 * Where:
 * p = position (float, 2 decimal places)
 * v = velocity (float, 3 decimal places)
 * a = commanded acceleration (float, 3 decimal places)
 * min/max = end switch state (int, 0=inactive, 1=active)
 * Grip = gripper state (int, 0=CLOSED, 1=OPEN)
 */
void send_robot_state_serial() {
    if (!g_robot_simulation) {
        printf("Error: Robot simulation object not initialized.\n");
        fflush(stdout);
        return; // Guard against null pointer
    }

    // Get Gripper State and map to integer
    int gripper_int_state = (g_robot_simulation->getGripperState() == GripperState::OPEN) ? 1 : 0;

    // Get End Switch States and map to integers
    int x_min_end = g_robot_simulation->isEndSwitchMinActive(0) ? 1 : 0;
    int x_max_end = g_robot_simulation->isEndSwitchMaxActive(0) ? 1 : 0;
    int y_min_end = g_robot_simulation->isEndSwitchMinActive(1) ? 1 : 0;
    int y_max_end = g_robot_simulation->isEndSwitchMaxActive(1) ? 1 : 0;
    int z_min_end = g_robot_simulation->isEndSwitchMinActive(2) ? 1 : 0;
    int z_max_end = g_robot_simulation->isEndSwitchMaxActive(2) ? 1 : 0;

    // Print all data in a single comma-separated line
    printf("%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d\n",
           // Positions (X, Y, Z)
           g_robot_simulation->getPosition(0),
           g_robot_simulation->getPosition(1),
           g_robot_simulation->getPosition(2),
           // Velocities (X, Y, Z)
           g_robot_simulation->getVelocity(0),
           g_robot_simulation->getVelocity(1),
           g_robot_simulation->getVelocity(2),
           // Commanded Accelerations (X, Y, Z)
           g_robot_simulation->getCommandedAcceleration(0),
           g_robot_simulation->getCommandedAcceleration(1),
           g_robot_simulation->getCommandedAcceleration(2),
           // End Switches (Xmin, Xmax, Ymin, Ymax, Zmin, Zmax)
           x_min_end, x_max_end,
           y_min_end, y_max_end,
           z_min_end, z_max_end,
           // Gripper State
           gripper_int_state
          );

    fflush(stdout); // Ensure data is sent immediately
}

/**
 * @brief Union to easily access bytes of a float. (Unchanged)
 */
typedef union {
    float f; // Use float for position data
    double d; // Use double for velocity/acceleration if needed
    uint8_t bytes[sizeof(double)]; // Size to max possible type used
} NumberUnion;

/**
 * @brief Updates the I2C RAM buffer with the current robot state.
 * Reads data directly from the global RobotSimulation object.
 * IMPORTANT: Call this within a critical section (interrupts disabled).
 */
void update_i2c_ram_buffer() {
    if (!g_robot_simulation) return; // Guard against null pointer

    NumberUnion data_union;
    float position;

    // --- Positions ---
    // X Position
    position = (float)g_robot_simulation->getPosition(0); // Cast double to float
    data_union.f = position;
    for (size_t i = 0; i < sizeof(float); ++i) {
        if ((I2C_REG_X_POS_START + i) < I2C_RAM_BUFFER_SIZE) {
            i2c_ram_buffer[I2C_REG_X_POS_START + i] = data_union.bytes[i];
        }
    }
    // Y Position
    position = (float)g_robot_simulation->getPosition(1);
    data_union.f = position;
    for (size_t i = 0; i < sizeof(float); ++i) {
        if ((I2C_REG_Y_POS_START + i) < I2C_RAM_BUFFER_SIZE) {
            i2c_ram_buffer[I2C_REG_Y_POS_START + i] = data_union.bytes[i];
        }
    }
    // Z Position
    position = (float)g_robot_simulation->getPosition(2);
    data_union.f = position;
    for (size_t i = 0; i < sizeof(float); ++i) {
        if ((I2C_REG_Z_POS_START + i) < I2C_RAM_BUFFER_SIZE) {
            i2c_ram_buffer[I2C_REG_Z_POS_START + i] = data_union.bytes[i];
        }
    }

    
}




/**
 * @brief FreeRTOS Task function to run the HIL simulation using RobotSimulation class.
 */
void hil_simulation_task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(HIL_UPDATE_RATE_MS);

    // Ensure the global robot object is initialized
    if (!g_robot_simulation) {
         printf("!!! FATAL: Robot Simulation object not initialized!\n");
         vTaskDelete(NULL); // Delete self
         return; // Should not be reached
    }

    printf("--- HIL Simulation Task Started ---\n");
    fflush(stdout);

    // Initialize outputs based on initial simulation state
    gpio_put(PIN_X_END_MIN, g_robot_simulation->isEndSwitchMinActive(0));
    gpio_put(PIN_X_END_MAX, g_robot_simulation->isEndSwitchMaxActive(0));
    gpio_put(PIN_Y_END_MIN, g_robot_simulation->isEndSwitchMinActive(1));
    gpio_put(PIN_Y_END_MAX, g_robot_simulation->isEndSwitchMaxActive(1));
    gpio_put(PIN_Z_END_MIN, g_robot_simulation->isEndSwitchMinActive(2));
    gpio_put(PIN_Z_END_MAX, g_robot_simulation->isEndSwitchMaxActive(2));
    bool is_gripper_open = (g_robot_simulation->getGripperState() == GripperState::OPEN);
    gpio_put(PIN_GRIP_SNS_OPEN, is_gripper_open);
    gpio_put(PIN_GRIP_SNS_CLOSED, !is_gripper_open);

    // Initialize I2C RAM buffer (Protected)
    uint32_t status = save_and_disable_interrupts();
    update_i2c_ram_buffer();
    restore_interrupts(status);

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // --- Read Inputs ---
        float pwm_readings[NUM_AXES][3]; // Store readings for X, Y, Z
        bool dir_inputs[NUM_AXES];

        // Read PWM and Direction for each axis
        for (int i = 0; i < NUM_AXES; ++i) {
            uint pwm_pin;
            uint dir_pin;
            if (i == 0) { // Axis X
                 dir_pin = PIN_X_DIR;
            } else if (i == 1) { // Axis Y
                 dir_pin = PIN_Y_DIR;
            } else { // Axis Z
                 dir_pin = PIN_Z_DIR;
            }
            // Use PIO SM index (0, 1, 2) for reading PWM
            p_PwmInput->read_PWM(pwm_readings[i], i);
            dir_inputs[i] = gpio_get(dir_pin);
        }

        // Read Gripper commands
        bool cmd_open = gpio_get(PIN_GRIP_CMD_OPEN);
        bool cmd_close = gpio_get(PIN_GRIP_CMD_CLOSE);

        // --- Set Commands for RobotSimulation ---
        // Set Axis Accelerations
        for (int i = 0; i < NUM_AXES; ++i) {
             float duty_cycle = fmaxf(0.0f, fminf(1.0f, pwm_readings[i][2])); // Clamp duty cycle
             bool move_positive = dir_inputs[i];
             // Calculate commanded acceleration based on duty cycle and max configured acceleration
             double commanded_accel = duty_cycle * axis_configs[i].maxAcceleration * (move_positive ? 1.0 : -1.0);
             g_robot_simulation->setAxisAcceleration(i, commanded_accel);
        }

        // Set Gripper State (Instantaneous based on commands)
        // Prioritize CLOSE command if both are active (adjust logic if needed)
        if (cmd_close) {
            g_robot_simulation->setGripperState(GripperState::CLOSED);
        } else if (cmd_open) {
             g_robot_simulation->setGripperState(GripperState::OPEN);
        }
        // If neither command is active, the gripper state remains as it was.


        // --- Run Simulation Step ---
        g_robot_simulation->update(HIL_UPDATE_RATE_S); // Use dt in seconds


        // --- Update Outputs ---

        // Update End Switch GPIOs
        gpio_put(PIN_X_END_MIN, g_robot_simulation->isEndSwitchMinActive(0));
        gpio_put(PIN_X_END_MAX, g_robot_simulation->isEndSwitchMaxActive(0));
        gpio_put(PIN_Y_END_MIN, g_robot_simulation->isEndSwitchMinActive(1));
        gpio_put(PIN_Y_END_MAX, g_robot_simulation->isEndSwitchMaxActive(1));
        gpio_put(PIN_Z_END_MIN, g_robot_simulation->isEndSwitchMinActive(2));
        gpio_put(PIN_Z_END_MAX, g_robot_simulation->isEndSwitchMaxActive(2));

        // Update Gripper Sensor GPIOs
        is_gripper_open = (g_robot_simulation->getGripperState() == GripperState::OPEN);
        gpio_put(PIN_GRIP_SNS_OPEN, is_gripper_open);
        gpio_put(PIN_GRIP_SNS_CLOSED, !is_gripper_open);

        // Update I2C RAM Buffer (Protected)
        status = save_and_disable_interrupts();
        update_i2c_ram_buffer();
        restore_interrupts(status);

        // Send State over Serial (for monitoring/debugging)
        send_robot_state_serial();

        // --- Wait for next cycle ---
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief Initializes a GPIO pin for a specific function (input/output). (Unchanged)
 */
void setup_gpio(uint pin, bool is_output, bool pullup, bool pulldown) {
    gpio_init(pin);
    if (is_output) {
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0); // Initialize output low
    } else {
        gpio_set_dir(pin, GPIO_IN);
        gpio_set_pulls(pin, pullup, pulldown);
    }
}

/**
 * @brief I2C Slave event handler using the pico-sdk i2c_slave library. (Unchanged)
 */
static void i2c_slave_event_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    // --- This function remains unchanged ---
    switch (event) {
        case I2C_SLAVE_RECEIVE:
            if (!i2c_address_set) {
                i2c_ram_address = i2c_read_byte_raw(i2c);
                if (i2c_ram_address >= I2C_RAM_BUFFER_SIZE) i2c_ram_address = 0;
                i2c_address_set = true;
            } else {
                (void)i2c_read_byte_raw(i2c); // Discard extra bytes
            }
            break;
        case I2C_SLAVE_REQUEST:
            if (i2c_address_set && i2c_ram_address < I2C_RAM_BUFFER_SIZE) {
                i2c_write_byte_raw(i2c, i2c_ram_buffer[i2c_ram_address]);
                i2c_ram_address++;
                if (i2c_ram_address >= I2C_RAM_BUFFER_SIZE) i2c_ram_address = 0;
            } else {
                i2c_write_byte_raw(i2c, 0xFF); // Send dummy data
            }
            break;
        case I2C_SLAVE_FINISH:
            i2c_address_set = false;
            break;
        default:
            break;
    }
}

// --- Function to Initialize the Global Robot Object ---
// This needs to be called before the scheduler starts.
// It uses 'new' which requires C++ context.
#ifdef __cplusplus
extern "C" {
#endif
void initialize_robot_simulation() {
    // Create std::vector from C array for constructor
    std::vector<AxisConfig> configs_vec(axis_configs, axis_configs + NUM_AXES);
    // Use 'new' to create the object on the heap
    g_robot_simulation = new RobotSimulation(configs_vec, GripperState::CLOSED); // Start closed
    printf("RobotSimulation object initialized.\n");
}
#ifdef __cplusplus
}
#endif


int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("\n--- Pico HIL Simulator (RobotSimulation Class) Initializing ---\n");
    fflush(stdout);

    // --- Initialize Robot Simulation Object ---
    // IMPORTANT: This calls the C++ constructor
    initialize_robot_simulation();
    if (!g_robot_simulation) {
         printf("!!! FATAL: Failed to initialize RobotSimulation object.\n");
         while(1); // Halt
    }

    // --- Initialize I2C Peripheral --- (Unchanged)
    printf("Initializing I2C Peripheral (Slave)...\n");
    i2c_init(I2C_PERIPHERAL_INSTANCE, I2C_BAUDRATE);
    gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_PIN_SDA);
    gpio_pull_up(I2C_PIN_SCL);
    i2c_slave_init(I2C_PERIPHERAL_INSTANCE, I2C_PERIPHERAL_ADDR, &i2c_slave_event_handler);
    printf("I2C Slave Initialized. Address: 0x%02X, SDA: %d, SCL: %d\n", I2C_PERIPHERAL_ADDR, I2C_PIN_SDA, I2C_PIN_SCL);
    fflush(stdout);

    // --- Initialize GPIOs --- (Unchanged except for initial state setting)
    printf("Initializing GPIOs...\n");
    // Axis X
    setup_gpio(PIN_X_DIR, false, false, false); // Input
    setup_gpio(PIN_X_END_MIN, true, false, false); // Output
    setup_gpio(PIN_X_END_MAX, true, false, false); // Output
    // Axis Y
    setup_gpio(PIN_Y_DIR, false, false, false);
    setup_gpio(PIN_Y_END_MIN, true, false, false);
    setup_gpio(PIN_Y_END_MAX, true, false, false);
    // Axis Z
    setup_gpio(PIN_Z_DIR, false, false, false);
    setup_gpio(PIN_Z_END_MIN, true, false, false);
    setup_gpio(PIN_Z_END_MAX, true, false, false);
    // Gripper
    setup_gpio(PIN_GRIP_CMD_OPEN, false, false, true); // Input Pull-down
    setup_gpio(PIN_GRIP_CMD_CLOSE, false, false, true); // Input Pull-down
    setup_gpio(PIN_GRIP_SNS_OPEN, true, false, false); // Output
    setup_gpio(PIN_GRIP_SNS_CLOSED, true, false, false); // Output
    #ifdef PIN_LED
    setup_gpio(PIN_LED, true, false, false);
    gpio_put(PIN_LED, 1);
    #endif
    // Initial GPIO states are set within the task now, after robot init.
    printf("GPIOs Initialized.\n");
    fflush(stdout);

    // --- Initialize PIO for PWM Inputs --- (Unchanged)
    printf("Initializing PWM Inputs...\n");
    const int NUM_OF_PINS = 3;
    uint pin_list[NUM_OF_PINS] = {PIN_X_PWM, PIN_Y_PWM, PIN_Z_PWM};
    // Allocate PwmIn object on the heap using 'new' since it's C++
    p_PwmInput = new PwmIn(pin_list, NUM_OF_PINS);

    while(true)
    {
        //print duty cycle of PIN_X_PWM to printf
        float readings[3];
        p_PwmInput->read_PWM(readings, 0);
        printf("DC: %f\n", readings[2]);
        sleep_ms(300);
    }
    printf("PWM Input Initialized.\n");
    fflush(stdout);

    // --- Create FreeRTOS Tasks --- (Unchanged)
    printf("Creating FreeRTOS Tasks...\n");
    BaseType_t sim_task_status = xTaskCreate(
        hil_simulation_task,
        "HILSimTask",
        4096, // Stack size
        NULL,
        configMAX_PRIORITIES - 1,
        NULL
    );
    if (sim_task_status != pdPASS) {
        printf("!!! FATAL: Failed to create HIL simulation task.\n");
        while(1) { /* Error loop */ }
    } else {
        printf("Tasks created successfully.\n");
    }
    fflush(stdout);

    #ifdef PIN_LED
    gpio_put(PIN_LED, 0);
    #endif

    // --- Start FreeRTOS Scheduler --- (Unchanged)
    printf("--- Starting FreeRTOS Scheduler ---\n");
    fflush(stdout);
    vTaskStartScheduler();

    // --- Should not reach here --- (Unchanged)
    while(1);
    return 0; // Should never return
}