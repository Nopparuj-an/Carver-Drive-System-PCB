# Carver-Drive-System-PCB
Main controller PCB for the Carver Outdoor Project

This PCB is responsible for interfacing the main PC (with ROS2) to control the steering motor and brake motor using PID, also with many other functions as follow:

#### Functions available on this board
- Reset MCU
- Odrive Hall Passthrough
- Gear Input
- Mode Switch
- Brake Input
- Direction Input
- Throttle Input
- 24V Sense
- 48V Sense
- UART1 PC Communications
- UART2 RS485 Steering Absolute Encoder
- Emergency Input
- UART2
- WS2812
- I2C
- Brake Motor Closed Loop Control
- Brake Current Input
- Steering Relay

#### DEPRECATED
- QEI Brake Encoder
- CANBUS

## Images
Before assembly (pre-assembled with some SMD components from factory)

![image](https://github.com/Nopparuj-an/Carver-Drive-System-PCB/assets/47713359/20e4e528-4de7-4263-be06-bf312b386abb)

After assembly

![image](https://github.com/Nopparuj-an/Carver-Drive-System-PCB/assets/47713359/b121e738-4cd2-4bcb-bb1b-30b9f794bec3)

## PCB fabrications
The schematics and design is available in EasyEDA format in the release tab.
