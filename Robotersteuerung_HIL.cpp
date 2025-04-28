#include <stdio.h>
#include "pico/stdlib.h"

int main()
{
    float x, y, z;
    int grabber;
    // Float for x, y and z movement and int for grabber (0 - closed, 1 - open)
    scanf("x %f, y %f, z %f, grabber %d\n", &x, &y, &z, &grabber);

    printf("x: %f, y: %f, z: %f, grabber: %d\n", x, y, z, grabber);
}
