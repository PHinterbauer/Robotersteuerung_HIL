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
#define TASK_STACK_SIZE 1000 // Stack size for FreeRTOS tasks
#define TASK_PRIORITY 1 // Priority for FreeRTOS tasks
#define AXIS_MAX_LIMIT 100 // Maximum axis limit
#define AXIS_MIN_LIMIT 0 // Minimum axis limit
#define Z_AXIS_MAX_LIMIT 150 // Maximum Z-axis limit since it is different
#define USE_CONSOLE_FOR_INPUT 0 // Flag to use console for input
#define MAX_COMMANDS 100 // Maximum number of commands for console input

// Enumeration for axis states
enum AxisState 
{
    IDLE_AXIS,
    INCREASE,
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
    float x; // X-axis position
    float y; // Y-axis position
    float z; // Z-axis position
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
    if (pull_up) {
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
void set_pwm(int gpio_pin, int duty_cycle) 
{
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_wrap(slice_num, PWM_WRAP_VALUE);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio_pin), duty_cycle); 
    pwm_set_enabled(slice_num, true);
}

// Parses a command string and fills the Command struct
bool check_command_format(const char* input, Command* command)
{
    if (sscanf(input, "x %d, y %d, z %d, grabber %d", &command->x, &command->y, &command->z, &command->grabber_status) == 4)
    {
        return true;
    } else
    {
        return false;
    }
}

// Displays the command list
void display_commands()
{
    printf("Stored commands:\n");
    for (int i = 0; i < command_count; i++)
    {
        printf("Command %d: x %d, y %d, z %d, grabber %d\n", i + 1, command_list[i].x, command_list[i].y, command_list[i].z, command_list[i].grabber_status);
    }
}

// Executes the commands in the command list
void execute_commands()
{   
    printf("Executing commands:\n");
    for (int i = 0; i < command_count; i++)
    {
        target_x = command_list[i].x;
        target_y = command_list[i].y;
        target_z = command_list[i].z;
        target_grabber = command_list[i].grabber_status;

        printf("Executing command %d: x %f, y %f, z %f, grabber %d\n", i + 1, target_x, target_y, target_z, target_grabber);

        while (x_axis.position != target_x || y_axis.position != target_y || z_axis.position != target_z)
        {
            if (x_axis.position < target_x) 
            {
                gpio_put(Button_X_Inc, 1); 
            } else if (x_axis.position > target_x) 
            {
                gpio_put(Button_X_Dec, 1); 
            } else {
                gpio_put(Button_X_Inc, 0);
                gpio_put(Button_X_Dec, 0);
            }

            if (y_axis.position < target_y) 
            {
                gpio_put(Button_Y_Inc, 1);
            } else if (y_axis.position > target_y) 
            {
                gpio_put(Button_Y_Dec, 1);
            } else {
                gpio_put(Button_Y_Inc, 0);
                gpio_put(Button_Y_Dec, 0);
            }

            if (z_axis.position < target_z) 
            {
                gpio_put(Button_Z_Inc, 1);
            } else if (z_axis.position > target_z) 
            {
                gpio_put(Button_Z_Dec, 1);
            } else {
                gpio_put(Button_Z_Inc, 0);
                gpio_put(Button_Z_Dec, 0);
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (grabber.status != target_grabber) 
        {
            gpio_put(Button_Grabber, 1);
        } else
        {
            gpio_put(Button_Grabber, 0);
        }
    }
    command_count = 0; // Clear the command list after execution
}

// Reads console input and adds commands to the command list
void console_input_handler(void* nothing)
{   
    if (USE_CONSOLE_FOR_INPUT)
    {
        printf("Input format: x <float>, y <float>, z <float>, grabber <int 0 closed 1 open>\n");
        printf("Enter command or 'run' to execute or 'display' to show commands\n");
        while (true)
        {
            char input[256];
            printf("Enter command, 'run', or 'display': ");
            scanf(" %[^\n]", input); // Read the entire line of input

            if (strcmp(input, "run") == 0)
            {
                execute_commands();
            } else if (strcmp(input, "display") == 0)
            {
                display_commands();
            }
            else
            {
                if (command_count < MAX_COMMANDS)
                {
                    Command command;
                    if (check_command_format(input, &command))
                    {
                        command_list[command_count++] = command;
                        printf("Command added: x %f, y %f, z%f, grabber %d\n", command.x, command.y, command.z, command.grabber_status);
                    } else
                    {
                        printf("Invalid command format. Please try again.\n");
                    }
                } else
                {
                    printf("Command list is full. Cannot add more commands.\n");
                }
            }
        
        vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// Reads position data from the I2C slave device
void i2c_controller(void* nothing)
{
    while (true)
    {
        uint8_t reg_addr = 0x00;

        int x_axis_data = 0;
        if (i2c_write_blocking(i2c0, I2C_ADDR, &reg_addr, 1, true) < 0)
        {
            printf("I2C write failed for x-axis\n");
        }
        if (i2c_read_blocking(i2c0, I2C_ADDR, (uint8_t*) &x_axis_data, 4, false) < 0)
        {
            printf("I2C read failed for x-axis\n");
        }
        x_axis.position = x_axis_data;

        reg_addr = 0x04;
        int y_axis_data = 0;
        if (i2c_write_blocking(i2c0, I2C_ADDR, &reg_addr, 1, true) <0)
        {
            printf("I2C write failed for y-axis\n");
        }
        if (i2c_read_blocking(i2c0, I2C_ADDR, (uint8_t*) &y_axis_data, 4, false) < 0)
        {
            printf("I2C read failed for y-axis\n");
        }
        y_axis.position = y_axis_data;

        reg_addr = 0x08;
        int z_axis_data = 0;
        if (i2c_write_blocking(i2c0, I2C_ADDR, &reg_addr, 1, true) < 0)
        {
            printf("I2C write failed for z-axis\n");
        }
        if (i2c_read_blocking(i2c0, I2C_ADDR, (uint8_t*) &z_axis_data, 4, false) < 0)
        {
            printf("I2C read failed for z-axis\n");
        }
        z_axis.position = z_axis_data;
        
        vTaskDelay(pdMS_TO_TICKS(100));
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
                    grabber.state = OPEN;
                }
                else if (grabber.status == 1 && gpio_get(Button_Grabber) == 1)
                {
                    grabber.state = CLOSE;
                }
            break;

            case OPEN:
                gpio_put(GPIO_Grabber_Open_Out, 1);
                gpio_put(GPIO_Grabber_Close_Out, 0);
                grabber.state = IDLE_GRABBER;
            break;

            case CLOSE:
                gpio_put(GPIO_Grabber_Open_Out, 0);
                gpio_put(GPIO_Grabber_Close_Out, 1);
                grabber.state = IDLE_GRABBER;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Controls the X-axis motor based on button input and position limits
void x_axis_controller(void* nothing)
{
    int duty_cycle = 0;
    while (true)
    {
        switch (x_axis.state)
        {
            case IDLE_AXIS:
                if (gpio_get(Button_X_Inc) == 1)
                {
                    x_axis.state = INCREASE;
                }
                else if (gpio_get(Button_X_Dec) == 1)
                {
                    x_axis.state = DECREASE;
                }
            break;

            case INCREASE:
                if (x_axis.position < x_axis.max_limit && x_axis.position > x_axis.min_limit && gpio_get(MAX_X_In) == 0 && gpio_get(MIN_X_In) == 0)
                {
                    gpio_put(DIR_X_Out, 1);
                    if (duty_cycle < 255)
                    {
                        duty_cycle++;
                    }
                    set_pwm(PWM_X_Out, duty_cycle);
                }
                else
                {
                    x_axis.state = SLOW;
                }
            break;

            case DECREASE:
                if (x_axis.position < x_axis.max_limit && x_axis.position > x_axis.min_limit)
                {
                    gpio_put(DIR_X_Out, 0);
                    if (duty_cycle > 0)
                    {
                        duty_cycle--;
                    }
                    set_pwm(PWM_X_Out, duty_cycle);
                }
                else if (gpio_get(MAX_X_In) == 1 && gpio_get(MIN_X_In) == 1)
                {
                    gpio_put(DIR_X_Out, 0);
                    if (duty_cycle > 0)
                    {
                        duty_cycle--;
                    }
                    set_pwm(PWM_X_Out, duty_cycle);
                }
                else
                {
                    x_axis.state = SLOW;
                }
            break;

            case SLOW:
                set_pwm(PWM_X_Out, 0);
                x_axis.state = IDLE_AXIS;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Controls the Y-axis motor based on button input and position limits
void y_axis_controller(void* nothing)
{
    int duty_cycle = 0;
    while (true)
    {
        switch (y_axis.state)
        {
            case IDLE_AXIS:
                if (gpio_get(Button_Y_Inc) == 1)
                {
                    y_axis.state = INCREASE;
                }
                else if (gpio_get(Button_Y_Dec) == 1)
                {
                    y_axis.state = DECREASE;
                }
            break;

            case INCREASE:
                if (y_axis.position < y_axis.max_limit && y_axis.position > y_axis.min_limit && gpio_get(MAX_Y_In) == 0 && gpio_get(MIN_Y_In) == 0)
                {
                    gpio_put(DIR_Y_Out, 1);
                    if (duty_cycle < 255)
                    {
                        duty_cycle++;
                    }
                    set_pwm(PWM_Y_Out, duty_cycle);
                }
                else
                {
                    y_axis.state = SLOW;
                }
            break;

            case DECREASE:
                if (y_axis.position < y_axis.max_limit && y_axis.position > y_axis.min_limit)
                {
                    gpio_put(DIR_Y_Out, 0);
                    if (duty_cycle > 0)
                    {
                        duty_cycle--;
                    }
                    set_pwm(PWM_Y_Out, duty_cycle);
                }
                else
                {
                    y_axis.state = SLOW;
                }
            break;

            case SLOW:
                set_pwm(PWM_Y_Out, 0);
                y_axis.state = IDLE_AXIS;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Controls the Z-axis motor based on button input and position limits
void z_axis_controller(void* nothing)
{
    int duty_cycle = 0;
    while (true)
    {
        switch (z_axis.state)
        {
            case IDLE_AXIS:
                if (gpio_get(Button_Z_Inc) == 1)
                {
                    z_axis.state = INCREASE;
                }
                else if (gpio_get(Button_Z_Dec) == 1)
                {
                    z_axis.state = DECREASE;
                }
            break;

            case INCREASE:
                if (z_axis.position < z_axis.max_limit && z_axis.position > z_axis.min_limit && gpio_get(MAX_Z_In) == 0 && gpio_get(MIN_Z_In) == 0)
                {
                    gpio_put(DIR_Z_Out, 1);
                    if (duty_cycle < 255)
                    {
                        duty_cycle++;
                    }
                    set_pwm(PWM_Z_Out, duty_cycle);
                }
                else
                {
                    z_axis.state = SLOW;
                }
            break;

            case DECREASE:
                if (z_axis.position < z_axis.max_limit && z_axis.position > z_axis.min_limit)
                {
                    gpio_put(DIR_Z_Out, 0);
                    if (duty_cycle > 0)
                    {
                        duty_cycle--;
                    }
                    set_pwm(PWM_Z_Out, duty_cycle);
                }
                else
                {
                    z_axis.state = SLOW;
                }
            break;

            case SLOW:
                set_pwm(PWM_Z_Out, 0);
                z_axis.state = IDLE_AXIS;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Configures GPIO pins for input/output and initializes pull-up resistors
void setup_gpio()
{
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

    const int gpio_input_pins[] = {
        MIN_X_In, 
        MAX_X_In, 
        MIN_Y_In, 
        MAX_Y_In,
        MIN_Z_In, 
        MAX_Z_In
    };

    const int gpio_output_pins[] = {
        PWM_X_Out, 
        DIR_X_Out, 
        PWM_Y_Out, 
        DIR_Y_Out,
        PWM_Z_Out, 
        DIR_Z_Out, 
        GPIO_Grabber_Open_Out, 
        GPIO_Grabber_Close_Out
    };

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
}

// Main function initializes GPIO and starts FreeRTOS tasks
int main()
{   
    stdio_init_all();

    setup_gpio();

    xTaskCreate(i2c_controller, "i2c_controller", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(read_grabber_status, "read_grabber_status", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(x_axis_controller, "x_axis_controller", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(y_axis_controller, "y_axis_controller", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(z_axis_controller, "z_axis_controller", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(grabber_controller, "grabber_controller", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);

    vTaskStartScheduler();
}
