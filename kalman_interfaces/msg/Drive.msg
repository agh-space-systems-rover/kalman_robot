float32 speed # m/s; maximum wheel speed
float32 inv_radius # 1/m; turn radius; + = left, - = right
float32 sin_angle # [-1, 1]; translational wheel angle; + = left, - = right; Values outside of [-1, 1] might be interpreted as over-translation - the wheels will collectively turn over 90 degrees by arcsin(2 - abs(sin_angle)) degrees. In this case inv_radius will be smoothly reduced to zero.
float32 rotation # rad/s; in-place rotation speed; if non-zero, other fields are ignored; + = left, - = right
