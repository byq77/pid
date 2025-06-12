# pid

A PID controller for ROS2.

## Usage

Launch simulation:

```shell
ros2 launch pid servo_sim.launch.py
```

## Pid Controller Parameters

Default Config

```yaml
pid_controller:
  ros__parameters:
    Kd: 0.0
    Ki: 0.0
    Kp: 1.0
    angle_error: false
    cutoff_frequency: -1.0
    effort_limit: '{-1000.0, 1000.0}'
    pid_enabled: true
    rate: 200.0
    windup_limit: '{-1000.0, 1000.0}'

```

### rate

Rate in Hz at which the PID controller updates.

* Type: `double`
* Default Value: 200.0
* Read only: True

*Constraints:*

* greater than or equal to 10.0

*Additional Constraints:*

### Kp

Proportional gain for the PID controller.

* Type: `double`
* Default Value: 1.0

### Ki

Integral gain for the PID controller.

* Type: `double`
* Default Value: 0.0

### Kd

Derivative gain for the PID controller.

* Type: `double`
* Default Value: 0.0

### effort_limit

Effort limits for the PID controller.

* Type: `double_array`
* Default Value: {-1000.0, 1000.0}

*Constraints:*

* length must be equal to 2

*Additional Constraints:*

### windup_limit

Windup limit for the PID controller.

* Type: `double_array`
* Default Value: {-1000.0, 1000.0}

*Constraints:*

 - length must be equal to 2

*Additional Constraints:*



### cutoff_frequency

LPF cutoff frequency.

* Type: `double`
* Default Value: -1.0

### angle_error

If true, maintain angular error between -pi and pi.

* Type: `bool`
* Default Value: false

### pid_enabled

Enable or disable the PID controller.

* Type: `bool`
* Default Value: true
