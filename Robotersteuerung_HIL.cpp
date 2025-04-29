#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

float x, y, z; // x, y, z coordinates of the robot arm
int grabber = 0; // 0 = closed, 1 = open
int run = 0; // 0 = not running, 1 = running

// Initialize command line buffer
#define MAX_COMMANDS 10
float x_commands[MAX_COMMANDS], y_commands[MAX_COMMANDS], z_commands[MAX_COMMANDS];
int grabber_commands[MAX_COMMANDS];
int command_count = 0;
char command[100];

SemaphoreHandle_t mutex = 0;

// Get user input for the robot arm's position and grabber state
// In following format: x <float>, y <float>, z <float>, grabber <int>
// Newline or semicolon for each command
// When entering "run" the saved commands will be executed in sequence
void get_user_input(void* nothing) 
{
    printf("Enter x, y, z coordinates and grabber state (0 or 1) in following format: \n");
    printf("x <float>, y <float>, z <float>, grabber <int>\n");
    printf("Press Enter to submit your command. And enter \"run\" to execute saved commands.\n");

    while (1)
    {
        if (xSemaphoreTake(mutex, portMAX_DELAY))
        {
            if (scanf("x %f, y %f, z %f, grabber %d", &x, &y, &z, &grabber) == 4)
            {
                if (command_count < MAX_COMMANDS) 
                {
                    x_commands[command_count] = x;
                    y_commands[command_count] = y;
                    z_commands[command_count] = z;
                    grabber_commands[command_count] = grabber;
                    command_count++;
                } 
                else 
                {
                    printf("Command buffer full. Cannot save more commands.\n");
                }
            } 
            else 
            {
                if (scanf("%s", command) == 1)
                {
                    if (strcmp(command, "run") == 0)
                    {
                        run = 1;
                    }
                } 
                else
                {
                    printf("Invalid command. Please try again.\n");
                }
            }
            xSemaphoreGive(mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void execute_commands(void* nothing)
{
    while (1)
    {
        if (xSemaphoreTake(mutex, portMAX_DELAY))
        {
            if (run == 1)
            {
                for (int i = 0; i < command_count; i++)
                {
                    printf("Executing command %d: x = %f, y = %f, z = %f, grabber = %d\n", i + 1, x_commands[i], y_commands[i], z_commands[i], grabber_commands[i]);
                }
                run = 0; // Reset run flag after executing commands
                command_count = 0; // Clear command buffer after execution
            }
            xSemaphoreGive(mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(2000); // Wait for the serial connection to establish

    mutex = xSemaphoreCreateMutex();

    printf("===========================\n");
    printf("Robot Arm Control Pannel\n");
    printf("===========================\n");

    xTaskCreate(get_user_input, "get_user_input", 1000, NULL, 1, NULL);
    xTaskCreate(execute_commands, "execute_commands", 1000, NULL, 1, NULL);

    vTaskStartScheduler();
}

// TODO:
// ADD SEMAPHORE MUTEX
