# kalman_arc

Package for ARC competition functionality.

## RSCP rqt publisher

The package includes a development-only rqt plugin that builds an RSCP
`RequestEnvelope`, serializes it, applies COBS framing, appends the `0x00`
delimiter, and publishes the resulting `std_msgs/msg/UInt8MultiArray` to
`rscp/serial/rx`.

After building and sourcing the workspace, start it with:

```bash
rqt --standalone kalman_arc/RscpPublisher
```

It is also available in the regular rqt plugin menu under
`Plugins > Kalman > RSCP Publisher`.

## For RSCP with raspi

``bash
rqt --standalone kalman_arc/RscpOverTCP
```
