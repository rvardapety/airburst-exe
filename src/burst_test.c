#include <stdio.h>
#include "airburst.h"

int main (void) {

    bool state = true;
    float target_height = 0.80;

    airburst_init();
    airburst_distance_init();

    while (state) {
        float distance = airburst_get_distance_m();
        printf("distance m is %f\n", (double)distance);
        if (distance < target_height) {
            airburst_bust_bomb();
            printf("boom at %f\n", (double)distance);
            state = false;
        }
    }

    airburst_distance_destroy();
    airburst_destroy();

    return 0;
}
