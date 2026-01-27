# Inertial Sense CAN Driver - Default Parameters

To enable this driver edit these files before building:

| Parameter      | File                                    | Line | Default        | Enabled Value |
|----------------|----------------------------------------|------|----------------|---------------|
| IS_ENABLE      | AP_InertialSenseCAN.cpp                | ~39  | 0              | 1             |
| CAN_D1_PROTOCOL| AP_CANManager_CANDriver_Params.cpp     | ~34  | DroneCAN       | InertialSenseCAN (15) |
| CAN_P1_DRIVER  | AP_CANIfaceParams.cpp                  | ~29  | HAL_CAN_DRIVER_DEFAULT (0) | 1 |
