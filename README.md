# Logi_g29
From package '[Logi_g29](https://github.com/ISC-Project-Phoenix/logi_g29)'
# File
`./src/logi_g29.cpp`

## Summary 
 Logitech g29 force feedback wheel device driver.

This node fills the role of both a joy driver and something like joy_telop_twist.

## Topics

### Publishes
- `/ack_vel`: AckermannDrive output from the wheel. Speed will be set to brake speed if brake is pressed, else throttle if throttle is pressed, it cannot ever be both
- `/cmd_vel`: Twist output from the wheel. Same caveats as /ack_vel

### Subscribes
- `/joy`: Raw joy messages from the wheel. This should come from joy_node (included in this packages launch file)

## Params
- `max_throttle_speed`: Velocity in m/s output when throttle is fully pressed.
- `max_brake_speed`: Velocity in m/s output when brake is fully pressed. Should be negitive.
- `max_steering_rad`: Maximum steering wheel angle in radians the node will accept.
Steering over this value will output this value. Assumed to be symmetric.

- `wheelbase`: Wheelbase of the vehicle in meters.

## Potential Improvements
This node is generic across projects via libackermann, although currently it only has
support for phoenix. To use in another project, first add the steering ratio to libackermann, then add a parameter for project to this node.
 

# Launch 
 `./launch/logi_g29.launch.py` 
 Launches both logi_g29 and joy_node configured properly. 

## Args
- `max_braking_speed`: Same as node
- `max_throttle_speed`: Same as node
- `max_steering_rad`: Same as node
- `wheelbase`: Same as node

# Misc 
 This node does not directly pass the wheel angle as a joystick angle, but rather first passes the steering angle
through a steering ratio to emulate a steering rack. This yields a virtual ackermann wheel angle for an ackermann message
, which is then converted into a twist message.
 
