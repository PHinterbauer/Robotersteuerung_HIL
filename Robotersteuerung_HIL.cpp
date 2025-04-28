#include <stdio.h>
#include "pico/stdlib.h"

int main()
{
    stdio_init_all();

    float x, y, z;
    int grabber;

    printf("Bitte Werte eingeben im Format: x <float>, y <float>, z <float>, grabber <0 oder 1>\n");

    if (scanf("x %f, y %f, z %f, grabber %d", &x, &y, &z, &grabber) != 4) {
        printf("Fehlerhafte Eingabe!\n");
        return 1;  // Fehlercode
    }

    printf("Eingabewerte:\n");
    printf("x: %f\n", x);
    printf("y: %f\n", y);
    printf("z: %f\n", z);
    printf("grabber: %d\n", grabber);

    return 0;
}
