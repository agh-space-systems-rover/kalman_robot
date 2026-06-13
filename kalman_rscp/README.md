# kalman_rscp

## RSCP rqt publisher

The package includes a small development-only rqt plugin that builds an RSCP
`RequestEnvelope`, serializes it, applies COBS framing, appends the `0x00`
delimiter, and publishes the resulting `std_msgs/msg/UInt8MultiArray` to
`rscp/serial/rx`.

After building and sourcing the workspace, start it with:

```bash
rqt --standalone kalman_rscp/RscpPublisher
```

It is also available in the regular rqt plugin menu under
`Plugins > Kalman > RSCP Publisher`.
