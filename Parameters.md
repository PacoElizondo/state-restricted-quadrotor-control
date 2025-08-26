In this file, a descriptive table of the different control gains will be given.

## Restricted Controller


| Parameter      | Value             |
|----------------|-------------------|
| `trans_kp_0`   | [10, 10, 10]      |
| `trans_kd_0`   | [6, 6, 6]         |
| `ang_kp_0`     | [90, 90, 90]      |
| `ang_kd_0`     | [40, 40, 40]      |
| `lambda`       | [30, 80, 1.5, 0.5]|
| `alpha`        | [3, 0.05, 1, 0.05]|


## Adaptive Controller 


| Parameter      | Value             |
|----------------|-------------------|
| `trans_kp_0`   | [70, 70, 7]       |
| `trans_kd_0`   | [40, 40, 4]       |
| `ang_kp_0`     | [90, 90, 90]      |
| `ang_kd_0`     | [40, 40, 40]      |
| `lambda`       | [8, 1, 4, 0.5]    |
| `alpha`        | [4, 0.05, 1, 0.05]|

## PD Controller

| Parameter      | Value             |
|----------------|-------------------|
| `trans_kp_0`   | [7, 7, 7]         |
| `trans_kd_0`   | [3.75, 3.75, 3.75]|
| `ang_kp_0`     | [90, 90, 90]      |
| `ang_kd_0`     | [40, 40, 40]      |
| `lambda`       | N/A               |
| `alpha`        | N/A               |




| Parameter      | Description           |
|----------------|----------------------------------------|
| `trans_kp_0`   | Proportional gain of base controller for traslational dynamics  at time t=0                    |
| `trans_kd_0`   | Derivative gain of base controller for traslational dynamics at time t=0                         |
| `ang_kp_0`     | Proportional gain of base controller for rotational dynamics at time t=0                       |
| `ang_kd_0`     | Derivative gain of base controller for rotational dynamics at time t=0                           |
| `lambda`       | Adaptive or restrcitive controller gains for traslational dynamics                           |
| `alpha`        | Adaptive or restrcitive controller gains for rotational dynamics                             |

