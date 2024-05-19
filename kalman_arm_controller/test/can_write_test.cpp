#include "../hardware/include/kalman_arm_controller/can_driver.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <exception>
#include <thread>
#include <chrono>

int main()
{
    printf("Testing can_driver.cpp\r\n");
    CAN_driver::init();

    // Write velocity command to enable the joints
    int k = 4;
    // uint16_t can_id = (k << 7) + CMD_VELOCITY;
    // uint8_t data[LEN_CMD_VELOCITY] = {0xfa, 0x02, 0xff, 0xff, 0xd9, 0x09};
    // for (int i = 0; i < 10; i++)
    // {
    //     CAN_driver::write_data(can_id, data, LEN_CMD_VELOCITY);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
    // uint16_t can_id = (k << 7) + CMD_SETPOINT;
    // uint8_t data_pos[LEN_CMD_SETPOINT] = {0x00, 0x02, 0x00, 0x00};

    // for (int i = 0; i < 100; i++)
    // {
    //     write_data(can_id, data_pos, LEN_CMD_SETPOINT);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    // uint8_t data_pos2[LEN_CMD_SETPOINT] = {0xfa, 0x02, 0xff, 0xff, 0x00, 0x00, 0xa0, 0xc9, 0xfa, 0xff, 0xcd, 0x00};

    // for (int i = 0; i < 100; i++)
    // {
    //     CAN_driver::write_data(can_id, data_pos2, LEN_CMD_SETPOINT);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    uint32_t setpoint_cnt = 0;

    printf("Setting joints setpoints\r\n");

    while (1)
    {

        setpoint_cnt++;
        for (int i = 0; i < 4; i++)
        {
            CAN_vars::joints[i].moveSetpoint.position_deg = (float)setpoint_cnt * 0.36f;
        }
        for (int i = 4; i < 6; i++)
        {
            CAN_vars::joints[i].moveSetpointDiff.position_deg = (float)setpoint_cnt * 0.36f;
        }
        printf("Writing CAN data\r\n");
        // try
        // {
        //     if (CAN_driver::write())
        //         break;
        // }
        // catch (const std::exception &e)
        // {
        //     printf("Caught exception: %s\r\n", e.what());
        //     CAN_driver::close();
        //     return 1;
        // }
        printf("Written!\r\n\r\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // if (setpoint_cnt == 100)
        //     break;
    }
    CAN_driver::close();
    return 0;
}