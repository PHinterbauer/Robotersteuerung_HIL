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

// PWM configuration and FreeRTOS task settings
#define PWM_WRAP_VALUE 255 // PWM wrap value for 8-bit resolution
#define TASK_STACK_SIZE 1000 // Stack size for FreeRTOS tasks

// Enumeration for axis states
enum AxisState {
    IDLE_AXIS,
    INCREASE,
    DECREASE,
    SLOW
};

// Enumeration for grabber states
enum GrabberState {
    IDLE_GRABBER,
    OPEN,
    CLOSE
};

// Struct for axis configuration
typedef struct {
    int min_limit; // Minimum position limit
    int max_limit; // Maximum position limit
    int position;  // Current position
    enum AxisState state; // Axis state
} Axis;

// Struct for grabber configuration
typedef struct {
    int status; // 0 = closed, 1 = open
    enum GrabberState state; // Grabber state
} Grabber;

// Axis and grabber definitions
Axis x_axis = {0, 100, 0, IDLE_AXIS}; // X-axis configuration
Axis y_axis = {0, 100, 0, IDLE_AXIS}; // Y-axis configuration
Axis z_axis = {0, 150, 0, IDLE_AXIS}; // Z-axis configuration
Grabber grabber = {0, IDLE_GRABBER};  // Grabber configuration

// Sets up PWM on a given GPIO pin with a specified duty cycle
void set_pwm(int gpio_pin, int duty_cycle) 
{
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_wrap(slice_num, PWM_WRAP_VALUE);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio_pin), duty_cycle); 
    pwm_set_enabled(slice_num, true);
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
        x_axis.position = x_axis_data;

        reg_addr = 0x04;
        int y_axis_data = 0;
        i2c_write_blocking(i2c0, I2C_ADDR, &reg_addr, 1, true);
        i2c_read_blocking(i2c0, I2C_ADDR, (uint8_t*) &y_axis_data, 4, false);
        y_axis.position = y_axis_data;

        reg_addr = 0x08;
        int z_axis_data = 0;
        i2c_write_blocking(i2c0, I2C_ADDR, &reg_addr, 1, true);
        i2c_read_blocking(i2c0, I2C_ADDR, (uint8_t*) &z_axis_data, 4, false);
        z_axis.position = z_axis_data;
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
    }
}

// Configures GPIO pins for input/output and initializes pull-up resistors
void setup_gpio()
{
    gpio_init(I2C_SDA);
    gpio_set_dir(I2C_SDA, GPIO_IN);
    gpio_init(I2C_SCL);
    gpio_set_dir(I2C_SCL, GPIO_IN);

    gpio_init(Button_X_Inc);
    gpio_set_dir(Button_X_Inc, GPIO_IN);
    gpio_pull_up(Button_X_Inc);
    gpio_init(Button_X_Dec);
    gpio_set_dir(Button_X_Dec, GPIO_IN);
    gpio_pull_up(Button_X_Dec);
    gpio_init(Button_Y_Inc);
    gpio_set_dir(Button_Y_Inc, GPIO_IN);
    gpio_pull_up(Button_Y_Inc);
    gpio_init(Button_Y_Dec);
    gpio_set_dir(Button_Y_Dec, GPIO_IN);
    gpio_pull_up(Button_Y_Dec);
    gpio_init(Button_Z_Inc);
    gpio_set_dir(Button_Z_Inc, GPIO_IN);
    gpio_pull_up(Button_Z_Inc);
    gpio_init(Button_Z_Dec);
    gpio_set_dir(Button_Z_Dec, GPIO_IN);
    gpio_pull_up(Button_Z_Dec);
    gpio_init(Button_Grabber);
    gpio_set_dir(Button_Grabber, GPIO_IN);
    gpio_pull_up(Button_Grabber);

    gpio_init(SENSOR_Grabber_Open_In);
    gpio_set_dir(SENSOR_Grabber_Open_In, GPIO_IN);
    gpio_init(SENSOR_Grabber_Close_In);
    gpio_set_dir(SENSOR_Grabber_Close_In, GPIO_IN);
    gpio_init(MIN_X_In);
    gpio_set_dir(MIN_X_In, GPIO_IN);
    gpio_init(MAX_X_In);
    gpio_set_dir(MAX_X_In, GPIO_IN);
    gpio_init(MIN_Y_In);
    gpio_set_dir(MIN_Y_In, GPIO_IN);
    gpio_init(MAX_Y_In);
    gpio_set_dir(MAX_Y_In, GPIO_IN);
    gpio_init(MIN_Z_In);
    gpio_set_dir(MIN_Z_In, GPIO_IN);
    gpio_init(MAX_Z_In);
    gpio_set_dir(MAX_Z_In, GPIO_IN);

    gpio_init(PWM_X_Out);
    gpio_set_dir(PWM_X_Out, GPIO_OUT);
    gpio_init(DIR_X_Out);
    gpio_set_dir(DIR_X_Out, GPIO_OUT);
    gpio_init(PWM_Y_Out);
    gpio_set_dir(PWM_Y_Out, GPIO_OUT);
    gpio_init(DIR_Y_Out);
    gpio_set_dir(DIR_Y_Out, GPIO_OUT);
    gpio_init(PWM_Z_Out);
    gpio_set_dir(PWM_Z_Out, GPIO_OUT);
    gpio_init(DIR_Z_Out);
    gpio_set_dir(DIR_Z_Out, GPIO_OUT);
    gpio_init(GPIO_Grabber_Open_Out);
    gpio_set_dir(GPIO_Grabber_Open_Out, GPIO_OUT);
    gpio_init(GPIO_Grabber_Close_Out);
    gpio_set_dir(GPIO_Grabber_Close_Out, GPIO_OUT);
}

// Main function initializes GPIO and starts FreeRTOS tasks
int main()
{   
    stdio_init_all();
    setup_gpio();

    xTaskCreate(i2c_controller, "i2c_controller", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(read_grabber_status, "read_grabber_status", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(x_axis_controller, "x_axis_controller", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(y_axis_controller, "y_axis_controller", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(z_axis_controller, "z_axis_controller", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(grabber_controller, "grabber_controller", TASK_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();
}
