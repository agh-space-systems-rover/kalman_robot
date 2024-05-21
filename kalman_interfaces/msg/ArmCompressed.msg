# bit mask of joints which are sent in this command. LSB bit is joint 1, etc. If 1 then joint is sent, if 0 then joint is not sent
uint8 joints_mask

# joint speed from 0 to 200 (corresponding to -1 to 1 of gamepad axis)
uint8[] joints_data