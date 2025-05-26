#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

// I2C configuration
#define I2C_SDA 0 // GPIO pin for I2C SDA
#define I2C_SCL 1 // GPIO pin for I2C SCL
#define I2C_ADDR 0x30 // I2C address of the slave device

// GPIO pin definitions for buttons and sensors
#define Button_X_Inc 20 // GPIO pin for X increment button
#define Button_X_Dec 21 // GPIO pin for X decrement button
#define Button_Y_Inc 18 // GPIO pin for Y increment button
#define Button_Y_Dec 19 // GPIO pin for Y decrement button
#define Button_Z_Inc 16 // GPIO pin for Z increment button
#define Button_Z_Dec 17 // GPIO pin for Z decrement button
#define Button_Grabber 22 // GPIO pin for grabber button

#define PWM_X_Out 2 // GPIO pin for X motor PWM output
#define DIR_X_Out 3 // GPIO pin for X motor direction output
#define PWM_Y_Out 10 // GPIO pin for Y motor PWM output
#define DIR_Y_Out 11 // GPIO pin for Y motor direction output
#define PWM_Z_Out 12 // GPIO pin for Z motor PWM output
#define DIR_Z_Out 13 // GPIO pin for Z motor direction output
#define GPIO_Grabber_Open_Out 14 // GPIO pin for grabber output
#define GPIO_Grabber_Close_Out 15 // GPIO pin for grabber output

#define SENSOR_Grabber_Open_In 26 // GPIO pin for grabber status
#define SENSOR_Grabber_Close_In 27 // GPIO pin for grabber status
#define MIN_X_In 4 // GPIO pin for X min limit switch
#define MAX_X_In 5 // GPIO pin for X max limit switch
#define MIN_Y_In 6 // GPIO pin for Y min limit switch
#define MAX_Y_In 7 // GPIO pin for Y max limit switch
#define MIN_Z_In 8 // GPIO pin for Z min limit switch
#define MAX_Z_In 9 // GPIO pin for Z max limit switch

// General configuration
#define PWM_WRAP_VALUE 255 // PWM wrap value for 8-bit resolution

#define NORMAL_TASK_STACK_SIZE 1000 // Stack size for FreeRTOS tasks
#define NORMAL_TASK_PRIORITY 1 // Priority for FreeRTOS tasks

#define AXIS_MAX_LIMIT 100 // Maximum axis limit
#define AXIS_MIN_LIMIT 0 // Minimum axis limit
#define Z_AXIS_MAX_LIMIT 150 // Maximum Z-axis limit since it is different

// ================================================================
#define ENABLE_CONSOLE_FOR_INPUT 0 // Flag to use console for input
// ================================================================

#define MAX_COMMANDS 100 // Maximum number of commands for console input
#define MAX_COMMAND_LENGTH 3000 // Maximum length of input string for command

// Enumeration for axis states
enum AxisState 
{
    IDLE_AXIS,
    INCREASE,
    HOLD,
    DECREASE,
    SLOW
};

// Enumeration for grabber states
enum GrabberState 
{
    IDLE_GRABBER,
    OPEN,
    CLOSE
};

// Struct for command data
typedef struct 
{
    int x; // X-axis position
    int y; // Y-axis position
    int z; // Z-axis position
    int grabber_status; // Grabber state: 0 = closed, 1 = open
} Command;

// Struct for axis configuration
typedef struct 
{
    int min_limit; // Minimum position limit
    int max_limit; // Maximum position limit
    int position;  // Current position
    enum AxisState state; // Axis state
} Axis;

// Struct for grabber configuration
typedef struct 
{
    int status; // 0 = closed, 1 = open
    enum GrabberState state; // Grabber state
} Grabber;

// Axis and grabber definitions
Axis x_axis = {AXIS_MIN_LIMIT, AXIS_MAX_LIMIT, 0, IDLE_AXIS}; // X-axis configuration
Axis y_axis = {AXIS_MIN_LIMIT, AXIS_MAX_LIMIT, 0, IDLE_AXIS}; // Y-axis configuration
Axis z_axis = {AXIS_MIN_LIMIT, Z_AXIS_MAX_LIMIT, 0, IDLE_AXIS}; // Z-axis configuration
Grabber grabber = {0, IDLE_GRABBER};  // Grabber configuration

// Command definitions
Command command_list[MAX_COMMANDS]; // List of commands
int command_count = 0; // Number of commands in the list
int target_x = 0; // Target X-axis position
int target_y = 0; // Target Y-axis position
int target_z = 0; // Target Z-axis position
int target_grabber = 0; // Target grabber state

// Initializes GPIO pins for input
void init_gpio_input(int pin, bool pull_up)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (pull_up) 
    {
        gpio_pull_up(pin);
    }
}

// Initializes GPIO pins for output
void init_gpio_output(int pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
}

// Sets up PWM on a given GPIO pin with a specified duty cycle
void init_pwm_output(int gpio_pin) 
{
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_wrap(slice_num, PWM_WRAP_VALUE);
    pwm_set_enabled(slice_num, true);
}

// Removes unwanted spaces from the input string
void remove_all_spaces(char* input)
{
    char* temp = input; 
    char* output = input;

    while (*temp != '\0') 
    {
        if (*temp != ' ') 
        {
            *output++ = *temp; // Copy non-space characters
        }
        temp++;
    }
    *output = '\0'; // Null-terminate the modified string
}

// Parses a command string and fills the Command struct
void check_command_format(char* input, Command* command)
{
    char* single_command = strtok(input, ";"); // Split the input string into individual commands

    while (single_command != NULL)
    {
        remove_all_spaces(single_command); // Remove spaces from the command
        if (sscanf(single_command, "x%d,y%d,z%d,grabber%d", &command->x, &command->y, &command->z, &command->grabber_status) == 4)
        {
            if (command->x < AXIS_MIN_LIMIT || command->x > AXIS_MAX_LIMIT ||
                command->y < AXIS_MIN_LIMIT || command->y > AXIS_MAX_LIMIT ||
                command->z < AXIS_MIN_LIMIT || command->z > Z_AXIS_MAX_LIMIT ||
                (command->grabber_status != 0 && command->grabber_status != 1))
            {
                printf("Values out of range: 'x %d, y %d, z %d, grabber %d' -> Skipped.\n", command->x, command->y, command->z, command->grabber_status);
            }
            else if (command_count < MAX_COMMANDS) // Ensure command list is not full
            {
                command_list[command_count++] = *command; // Add valid command to the list
                printf("Command added: 'x %d, y %d, z %d, grabber %d'\n", command->x, command->y, command->z, command->grabber_status);
            }
            else
            {
                printf("Command list is full. Cannot add more commands.\n");
            }
        } 
        else
        {
            printf("Invalid command format: '%s' -> Skipped.\n", single_command);
        }
        single_command = strtok(NULL, ";"); // Get the next command
    }
}

// Displays the command list
void display_commands()
{
    printf("Stored commands:\n");
    for (int i = 0; i < command_count; i++)
    {
        printf("Command %d: 'x %d, y %d, z %d, grabber %d'\n", i + 1, command_list[i].x, command_list[i].y, command_list[i].z, command_list[i].grabber_status);
    }
}

// Executes the commands in the command list
void execute_commands()
{   
    printf("Executing commands:\n");
    for (int i = 0; i < command_count; i++)
    {
        target_x = command_list[i].x; // Set target X-axis position
        target_y = command_list[i].y; // Set target Y-axis position
        target_z = command_list[i].z; // Set target Z-axis position
        target_grabber = command_list[i].grabber_status; // Set target grabber state

        printf("Executing command %d: 'x %d, y %d, z %d, grabber %d'\n", i + 1, target_x, target_y, target_z, target_grabber);

        // Move axes to target positions
        while (x_axis.position != target_x || y_axis.position != target_y || z_axis.position != target_z)
        {
            if (x_axis.position < target_x) 
            {
                gpio_put(Button_X_Inc, 1); // Increment X-axis
            } 
            else if (x_axis.position > target_x) 
            {
                gpio_put(Button_X_Dec, 1); // Decrement X-axis
            } 
            else 
            {
                gpio_put(Button_X_Inc, 0); // Stop X-axis movement
                gpio_put(Button_X_Dec, 0);
            }

            if (y_axis.position < target_y) 
            {
                gpio_put(Button_Y_Inc, 1); // Increment Y-axis
            } 
            else if (y_axis.position > target_y) 
            {
                gpio_put(Button_Y_Dec, 1); // Decrement Y-axis
            } 
            else 
            {
                gpio_put(Button_Y_Inc, 0); // Stop Y-axis movement
                gpio_put(Button_Y_Dec, 0);
            }

            if (z_axis.position < target_z) 
            {
                gpio_put(Button_Z_Inc, 1); // Increment Z-axis
            } 
            else if (z_axis.position > target_z) 
            {
                gpio_put(Button_Z_Dec, 1); // Decrement Z-axis
            } 
            else 
            {
                gpio_put(Button_Z_Inc, 0); // Stop Z-axis movement
                gpio_put(Button_Z_Dec, 0);
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Control grabber state
        if (grabber.status != target_grabber) 
        {
            gpio_put(Button_Grabber, 1); // Activate grabber button
        } 
        else
        {
            gpio_put(Button_Grabber, 0); // Deactivate grabber button
        }
    }
    command_count = 0; // Clear the command list after execution
}

// Reads console input and adds commands to the command list
void console_input_handler(void* nothing)
{   
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for console to be ready
    printf("====================================================================================================================\n");
    printf("Input format: x <int %d - %d>, y <int %d - %d>, z <int %d - %d>, grabber <int 0 closed 1 open>\n", AXIS_MIN_LIMIT, AXIS_MAX_LIMIT, AXIS_MIN_LIMIT, AXIS_MAX_LIMIT, AXIS_MIN_LIMIT, Z_AXIS_MAX_LIMIT);
    printf("Enter command or 'run' to execute or 'display' to show commands use ';' to write multiple commands in a single line.\n");
    printf("====================================================================================================================\n");
    while (true)
    {
        char input[MAX_COMMAND_LENGTH]; // Buffer for user input
        char temp; // Temporary character for reading input
        int i = 0;

        printf("Enter command, 'run', or 'display': \n");
        while (i <(MAX_COMMAND_LENGTH - 1)) // Read input until buffer is full
        {
            scanf("%c", &temp);
            if (temp == '\n') // Stop reading on newline
            {
                break;
            }
            input[i++] = temp; // Store character in buffer
        }
        input[i] = '\0'; // Null-terminate the input string

        if (strcmp(input, "run") == 0) // Check if user entered "run"
        {
            execute_commands(); // Execute stored commands
        } 
        else if (strcmp(input, "display") == 0) // Check if user entered "display"
        {
            display_commands(); // Display stored commands
        } 
        else // Assume input is a command
        {
            Command command;
            check_command_format(input, &command); // Parse and validate the command
        }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Reads position data from the I2C slave device
void i2c_controller(void* nothing)
{
    while (true)
    {
        uint8_t reg_addr = 0x00;

        int x_axis_data = 0;
        i2c_write_blocking(i2c0, I2C_ADDR, &reg_addr, 1, true);
        i2c_read_blocking(i2c0, I2C_ADDR, (uint8_t*) &x_axis_data, 4, false);
        x_axis.position = x_axis_data; // Update x-axis position

        reg_addr = 0x04;
        int y_axis_data = 0;
        i2c_write_blocking(i2c0, I2C_ADDR, &reg_addr, 1, true);
        i2c_read_blocking(i2c0, I2C_ADDR, (uint8_t*) &y_axis_data, 4, false);
        y_axis.position = y_axis_data; // Update y-axis position

        reg_addr = 0x08;
        int z_axis_data = 0;
        i2c_write_blocking(i2c0, I2C_ADDR, &reg_addr, 1, true);
        i2c_read_blocking(i2c0, I2C_ADDR, (uint8_t*) &z_axis_data, 4, false);
        z_axis.position = z_axis_data; // Update z-axis position
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Monitors the grabber's open/close status using sensors
void read_grabber_status(void* nothing)
{
    while (true)
    {
        if (gpio_get(SENSOR_Grabber_Open_In) == 1)
        {
            grabber.status = 1;
        }
        else if (gpio_get(SENSOR_Grabber_Close_In) == 1)
        {
            grabber.status = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Controls the grabber's open/close mechanism based on button input
void grabber_controller(void* nothing)
{
    while (true)
    {
        switch (grabber.state)
        {
           case IDLE_GRABBER:
                if (grabber.status == 0 && gpio_get(Button_Grabber) == 1)
                {
                    grabber.state = OPEN; // Transition to OPEN state if button is pressed and grabber is closed
                }
                else if (grabber.status == 1 && gpio_get(Button_Grabber) == 1)
                {
                    grabber.state = CLOSE; // Transition to CLOSE state if button is pressed and grabber is open
                }
            break;

            case OPEN:
                gpio_put(GPIO_Grabber_Open_Out, 1); // Open the grabber
                gpio_put(GPIO_Grabber_Close_Out, 0);
                grabber.state = IDLE_GRABBER; // Return to idle state
            break;

            case CLOSE:
                gpio_put(GPIO_Grabber_Open_Out, 0); 
                gpio_put(GPIO_Grabber_Close_Out, 1); // Close the grabber
                grabber.state = IDLE_GRABBER; // Return to idle state
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Controls the X-axis motor based on button input and position limits
// void x_axis_controller(void* nothing)
// {
//     int duty_cycle = 0; // Initialize duty cycle for PWM control

//     while (true)
//     {
//         switch (x_axis.state)
//         {
//             case IDLE_AXIS:
//                 if (gpio_get(Button_X_Inc) == 0) // Check if increment button is pressed
//                 {
//                     x_axis.state = INCREASE; // Transition to INCREASE state
//                 }
//                 else if (gpio_get(Button_X_Dec) == 0) // Check if decrement button is pressed
//                 {
//                     x_axis.state = DECREASE; // Transition to DECREASE state
//                 }
//                 break;

//             case INCREASE:
//                 if (x_axis.position <= x_axis.max_limit && gpio_get(MAX_X_In) == 0 && gpio_get(Button_X_Inc) == 0) // Ensure within limits
//                 {
//                     // gpio_put(DIR_X_Out, 1); // Set direction to increase
//                     // if (duty_cycle < PWM_WRAP_VALUE) // Gradually increase duty cycle
//                     // {
//                     //     duty_cycle++;
//                     // }
//                     // pwm_set_gpio_level(PWM_X_Out, duty_cycle); // Apply PWM signal
//                     // gpio_put(DIR_X_Out, 1); // Set direction to increase
//                     // if (duty_cycle < PWM_WRAP_VALUE) // Gradually increase duty cycle
//                     // {
//                     //     for (int i = 0; i < 10; i++) // Gradually increase duty cycle in steps
//                     //     {
//                     //         pwm_set_gpio_level(PWM_X_Out, duty_cycle); // Apply PWM signal
//                     //         duty_cycle ++;
//                     //         vTaskDelay(pdMS_TO_TICKS(10)); // Delay to allow gradual increase
//                     //     }
//                     // }
//                     // duty_cycle = 0; // Reset duty cycle after applying
//                     // pwm_set_gpio_level(PWM_X_Out, duty_cycle); // Hold current Velocity
//                     // This case should increase the x acis pwm signal gradualy for a short ammount of time if the button is being held pressed after that time it should switch to hold
//                     gpio_put(DIR_X_Out, 1); // Set direction to increase
//                     if (duty_cycle < PWM_WRAP_VALUE) // Gradually increase duty cycle
//                     {
//                         for (int i = 0; i < 10; i++) // Gradually increase duty cycle in steps
//                         {
//                             pwm_set_gpio_level(PWM_X_Out, duty_cycle); // Apply PWM signal
//                             duty_cycle ++;
//                             vTaskDelay(pdMS_TO_TICKS(10)); // Delay to allow gradual increase
//                         }
//                         x_axis.state = HOLD; // Switch to HOLD state after gradual increase
//                     }
//                 }
//                 else
//                 {
//                     x_axis.state = SLOW; // Transition to SLOW state if limit is reached
//                 }
//                 break;

//             case HOLD:
//                 if (x_axis.position <= x_axis.max_limit && gpio_get(MAX_X_In) == 0 && gpio_get(Button_X_Inc) == 0) // Ensure within limits
//                 {
//                     duty_cycle = 0;
//                     pwm_set_gpio_level(PWM_X_Out, duty_cycle); // Hold current Velocity
//                 }
//                 else
//                 {
//                     x_axis.state = SLOW; // Transition to SLOW state if limit is reached
//                 }
//                 break;

//             case DECREASE:
//                 if (x_axis.position >= x_axis.min_limit && gpio_get(MIN_X_In) == 0 && gpio_get(Button_X_Dec) == 0) // Ensure within limits
//                 {
//                     // gpio_put(DIR_X_Out, 0); // Set direction to decrease
//                     // if (duty_cycle < PWM_WRAP_VALUE) // Gradually increase duty cycle
//                     // {
//                     //     duty_cycle++;
//                     // }
//                     // pwm_set_gpio_level(PWM_X_Out, duty_cycle); // Apply PWM signal
//                     gpio_put(DIR_X_Out, 1); // Set direction to increase
//                     if (duty_cycle < PWM_WRAP_VALUE) // Gradually increase duty cycle
//                     {
//                         for (int i = 0; i < 10; i++) // Gradually increase duty cycle in steps
//                         {
//                             pwm_set_gpio_level(PWM_X_Out, duty_cycle); // Apply PWM signal
//                             duty_cycle --;
//                             vTaskDelay(pdMS_TO_TICKS(10)); // Delay to allow gradual increase
//                         }
//                         x_axis.state = HOLD; // Switch to HOLD state after gradual increase
//                     }
//                 }
//                 else
//                 {
//                     x_axis.state = SLOW; // Transition to SLOW state if limit is reached
//                 }
//                 break;

//             case SLOW: 
//                 int last_position = x_axis.position;
//                 // Gradually decrease duty cycle to slow down the axis
//                 while (duty_cycle >= 0)
//                 {
//                     pwm_set_gpio_level(PWM_X_Out, duty_cycle);
//                     duty_cycle--;
//                     vTaskDelay(pdMS_TO_TICKS(10)); // Slightly longer delay for smoother deceleration

//                     // Check if the axis position has stopped changing
//                     if (x_axis.position == last_position)
//                     {
//                         // If position hasn't changed, stop the motor
//                         break;
//                     }
//                     last_position = x_axis.position;
//                 }
//                 pwm_set_gpio_level(PWM_X_Out, 0); // Ensure motor is stopped
//                 duty_cycle = 0;
//                 x_axis.state = IDLE_AXIS;
//                 break;
//         }
//         printf("Increasing X-axis: Position %d, Duty Cycle %d, STATE: %s, max_limit: %d, min_limit %d\n", x_axis.position, duty_cycle, x_axis.state == IDLE_AXIS ? "IDLE" : (x_axis.state == INCREASE ? "INCREASE" : (x_axis.state == DECREASE ? "DECREASE" : "SLOW")), x_axis.max_limit, x_axis.min_limit);

//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }
void x_axis_controller(void* nothing)
{
    int duty_cycle = 0; // Initialize duty cycle for PWM control

    while (true)
    {
        switch (x_axis.state)
        {
            case IDLE_AXIS:
                // Wait for increment or decrement button press, only if not at limits
                if (gpio_get(Button_X_Inc) == 0 && x_axis.position < x_axis.max_limit && gpio_get(MAX_X_In) == 0)
                {
                    x_axis.state = INCREASE;
                }
                else if (gpio_get(Button_X_Dec) == 0 && x_axis.position > x_axis.min_limit && gpio_get(MIN_X_In) == 0)
                {
                    x_axis.state = DECREASE;
                }
                break;

            case INCREASE:
                // Accelerate by increasing PWM a few steps if button is held and not at limit
                if (gpio_get(Button_X_Inc) == 0 && x_axis.position < x_axis.max_limit && gpio_get(MAX_X_In) == 0)
                {
                    gpio_put(DIR_X_Out, 1); // Set direction to increase
                    for (int i = 0; i < 10 && duty_cycle < PWM_WRAP_VALUE; i++)
                    {
                        duty_cycle++;
                        pwm_set_gpio_level(PWM_X_Out, duty_cycle);
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    x_axis.state = HOLD;
                }
                else
                {
                    x_axis.state = SLOW;
                }
                break;

            case DECREASE:
                // Accelerate by increasing PWM a few steps if button is held and not at limit
                if (gpio_get(Button_X_Dec) == 0 && x_axis.position > x_axis.min_limit && gpio_get(MIN_X_In) == 0)
                {
                    gpio_put(DIR_X_Out, 0); // Set direction to decrease
                    for (int i = 0; i < 10 && duty_cycle < PWM_WRAP_VALUE; i++)
                    {
                        duty_cycle++;
                        pwm_set_gpio_level(PWM_X_Out, duty_cycle);
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    x_axis.state = HOLD;
                }
                else
                {
                    x_axis.state = SLOW;
                }
                break;

            case HOLD:
                // Hold velocity: set PWM to 0, keep direction, wait for button release or limit
                pwm_set_gpio_level(PWM_X_Out, 0);
                duty_cycle = 0;
                if ((gpio_get(Button_X_Inc) == 1 && gpio_get(Button_X_Dec) == 1) ||
                    x_axis.position >= x_axis.max_limit || gpio_get(MAX_X_In) == 1 ||
                    x_axis.position <= x_axis.min_limit || gpio_get(MIN_X_In) == 1)
                {
                    x_axis.state = SLOW;
                }
                // Stay in HOLD as long as button is held and no limit is reached
                break;

            case SLOW:
                // Decelerate: decrease PWM until no movement is detected, then go to IDLE
                // int last_position = x_axis.position;
                while (duty_cycle > 0)
                {
                    pwm_set_gpio_level(PWM_X_Out, duty_cycle);
                    duty_cycle--;
                    vTaskDelay(pdMS_TO_TICKS(20));
                    // if (x_axis.position == last_position)
                    // {
                    //     break; // No movement detected
                    // }
                    // last_position = x_axis.position;
                }
                pwm_set_gpio_level(PWM_X_Out, 0);
                duty_cycle = 0;
                x_axis.state = IDLE_AXIS;
                break;
        }
        printf("Increasing X-axis: Position %d, Duty Cycle %d, STATE: %s, max_limit: %d, min_limit %d, position: %d\n", x_axis.position, duty_cycle, x_axis.state == IDLE_AXIS ? "IDLE" : (x_axis.state == INCREASE ? "INCREASE" : (x_axis.state == DECREASE ? "DECREASE" : "SLOW")), x_axis.max_limit, x_axis.min_limit, x_axis.position);


        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Controls the Y-axis motor based on button input and position limits (for further info look at x_axis_controller)
void y_axis_controller(void* nothing)
{
    int duty_cycle = 0;

    while (true)
    {
        switch (y_axis.state)
        {
            case IDLE_AXIS:
                if (gpio_get(Button_Y_Inc) == 0)
                {
                    y_axis.state = INCREASE;
                }
                else if (gpio_get(Button_Y_Dec) == 0)
                {
                    y_axis.state = DECREASE;
                }
                break;

            case INCREASE:
                if (y_axis.position < y_axis.max_limit && gpio_get(MAX_Y_In) == 0 && gpio_get(Button_Y_Inc) == 0)
                {
                    gpio_put(DIR_Y_Out, 1);
                    if (duty_cycle < PWM_WRAP_VALUE)
                    {
                        duty_cycle++;
                    }
                    pwm_set_gpio_level(PWM_Y_Out, duty_cycle);
                }
                else                
                {
                    y_axis.state = SLOW;
                }
                break;

            case DECREASE:
                if (y_axis.position > y_axis.min_limit && gpio_get(MIN_Y_In) == 0 && gpio_get(Button_Y_Dec) == 0)
                {
                    gpio_put(DIR_Y_Out, 0);
                    if (duty_cycle < PWM_WRAP_VALUE)
                    {
                        duty_cycle++;
                    }
                    pwm_set_gpio_level(PWM_Y_Out, duty_cycle);
                }
                else
                {
                    y_axis.state = SLOW;
                }
                break;

            case SLOW:
                pwm_set_gpio_level(PWM_Y_Out, 0);
                duty_cycle = 0;
                y_axis.state = IDLE_AXIS;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Controls the Z-axis motor based on button input and position limits (for further info look at x_axis_controller)
void z_axis_controller(void* nothing)
{
    int duty_cycle = 0;

    while (true)
    {
        switch (z_axis.state)
        {
            case IDLE_AXIS:
                if (gpio_get(Button_Z_Inc) == 0)
                {
                    z_axis.state = INCREASE;
                }
                else if (gpio_get(Button_Z_Dec) == 0)
                {
                    z_axis.state = DECREASE;
                }
                break;

            case INCREASE:
                if (z_axis.position < z_axis.max_limit && gpio_get(MAX_Z_In) == 0 && gpio_get(Button_Z_Inc) == 0)
                {
                    gpio_put(DIR_Z_Out, 1);
                    if (duty_cycle < PWM_WRAP_VALUE)
                    {
                        duty_cycle++;
                    }
                    pwm_set_gpio_level(PWM_Z_Out, duty_cycle);
                }
                else
                {
                    z_axis.state = SLOW;
                }
                break;

            case DECREASE:
                if (z_axis.position > z_axis.min_limit && gpio_get(MIN_Z_In) == 0 && gpio_get(Button_Z_Dec) == 0)
                {
                    gpio_put(DIR_Z_Out, 0);
                    if (duty_cycle < PWM_WRAP_VALUE)
                    {
                        duty_cycle++;
                    }
                    pwm_set_gpio_level(PWM_Z_Out, duty_cycle);
                }
                else
                {
                    z_axis.state = SLOW;
                }
                break;

            case SLOW:
                pwm_set_gpio_level(PWM_Z_Out, 0);
                duty_cycle = 0;
                z_axis.state = IDLE_AXIS;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Configures GPIO pins for input/output and initializes pull-up resistors
void setup_gpio()
{
    // List of GPIO pins for input that require pull-up resistors
    const int gpio_input_pins_pull_up[] = {
        Button_X_Inc, 
        Button_X_Dec, 
        Button_Y_Inc, 
        Button_Y_Dec,
        Button_Z_Inc, 
        Button_Z_Dec, 
        Button_Grabber,
        SENSOR_Grabber_Open_In, 
        SENSOR_Grabber_Close_In
    };

    // List of GPIO pins for input without pull-up resistors
    const int gpio_input_pins[] = {
        MIN_X_In, 
        MAX_X_In, 
        MIN_Y_In, 
        MAX_Y_In,
        MIN_Z_In, 
        MAX_Z_In
    };

    // List of GPIO pins for output
    const int gpio_output_pins[] = {
        DIR_X_Out, 
        DIR_Y_Out,
        DIR_Z_Out, 
        GPIO_Grabber_Open_Out, 
        GPIO_Grabber_Close_Out
    };

    // List of GPIO pins for PWM output
    const int pwm_output_pins[] = {
        PWM_X_Out, 
        PWM_Y_Out, 
        PWM_Z_Out
    };

    // Initialize GPIO pins
    for (int i : gpio_input_pins_pull_up)
    {
        init_gpio_input(i, true);
    }

    for (int i : gpio_input_pins)
    {
        init_gpio_input(i, false);
    }

    for (int i : gpio_output_pins)
    {
        init_gpio_output(i);
    }

    for (int i : pwm_output_pins)
    {
        init_pwm_output(i);
    }
}

// Main function initializes GPIO and starts FreeRTOS tasks
int main()
{   
    stdio_init_all();
    
    setup_gpio();

    if (ENABLE_CONSOLE_FOR_INPUT) // Only start console input handler if enabled
    {
    xTaskCreate(console_input_handler, "console_input_handler", NORMAL_TASK_STACK_SIZE, NULL, NORMAL_TASK_PRIORITY, NULL);
    }
    // xTaskCreate(i2c_controller, "i2c_controller", NORMAL_TASK_STACK_SIZE, NULL, NORMAL_TASK_PRIORITY, NULL);
    // xTaskCreate(read_grabber_status, "read_grabber_status", NORMAL_TASK_STACK_SIZE, NULL, NORMAL_TASK_PRIORITY, NULL);
    xTaskCreate(x_axis_controller, "x_axis_controller", NORMAL_TASK_STACK_SIZE, NULL, NORMAL_TASK_PRIORITY, NULL);
    // xTaskCreate(y_axis_controller, "y_axis_controller", NORMAL_TASK_STACK_SIZE, NULL, NORMAL_TASK_PRIORITY, NULL);
    // xTaskCreate(z_axis_controller, "z_axis_controller", NORMAL_TASK_STACK_SIZE, NULL, NORMAL_TASK_PRIORITY, NULL);
    // xTaskCreate(grabber_controller, "grabber_controller", NORMAL_TASK_STACK_SIZE, NULL, NORMAL_TASK_PRIORITY, NULL);

    vTaskStartScheduler();
}
