#include "esp32_cam.h"
#include <stdio.h>

int main(void)
{
    printf("PROGRAM STARTED\n");
    fflush(stdout);

    printf("Before init\n");
    fflush(stdout);

    if (!esp32_cam_init()) {
        printf("INIT FAILED\n");
        fflush(stdout);
        return 1;
    }

    printf("INIT SUCCESS\n");
    fflush(stdout);

    cam_frame_t frame = {0};

    printf("Before get_frame\n");
    fflush(stdout);

    if (!esp32_cam_get_frame(&frame)) {
        printf("GET FRAME FAILED\n");
        fflush(stdout);

        esp32_cam_deinit();
        return 1;
    }

    printf("GET FRAME SUCCESS\n");
    printf("Image size: %u bytes\n", frame.length);
    fflush(stdout);

    esp32_cam_free_frame(&frame);
    esp32_cam_deinit();

    printf("PROGRAM FINISHED\n");
    fflush(stdout);

    return 0;
}