#include "kalman_arm_controller/can_driver.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <exception>
#include <chrono>

int main()
{

    printf("Testing can_driver.cpp\r\n");
    if (CAN_driver::init())
        return 1;
    while (1)
    {
        printf("Reading CAN data\r\n");
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        try
        {
            CAN_driver::read();
        }
        catch (const std::exception &e)
        {
            printf("Caught exception: %s\r\n", e.what());
            CAN_driver::close();
            return 1;
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        printf("Done reading CAN data\r\n");
        printf("Active joints status:\r\n");
        for (int i = 0; i < 6; i++)
        {
            printf("Joint %d:\r\n\tVelocity: %d\r\n\tPosition: %d\r\n", i, CAN_vars::joints[i].fastStatus.velocity, CAN_vars::joints[i].fastStatus.position);
        }
        printf("\r\n");

        printf("Reading took %ld [us]\r\n\r\n", std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());
        usleep(1000000);
    }
    CAN_driver::close();
    return 0;
}
