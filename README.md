# Repo for Glider
This is repository for Glider

# Folder Tree for This Repository
```bash
├───BB_program_latest               ###(BB program)
│   └───Baru
│       
├───Logs                            ###(latest log files)
├───program_ros_glider_underdevelop ###(ROS program for Glider)
│   └───catkin_ws
│     
├───Simulation                      ###(simulation pitch and depth for glider)
│   └───GaneshBlueDepthandpitchsimulation
│
└───TS_program_latest               ###(Main program for glider in TS)
    └───dhimas
```

## Latest version:
### BB
BB program is managed by Ardian, last modified by Jason to support TCPIP communication (December 2023). Last BB program is on Baru/BBrevs3.3.cpp, codes include sensor data acquisition for DVL, Alti, MiniCT, and IMU. The sensor using serial communication while communication to TS can be choosen TCP/IP or Serial communication. If using serial, baud rate for TS is 9600.

### TS
TS program is maintaned by Jason, last modified by Jason to support TCPIP communication (December 2023)

### ROS
ROS program is maintained by Ardian and Jason, last progress has finished building topics and setting up communication for arduino. But other programs like sensor acquisition, navigation, guidance and control is still empty. Next work should migrate the programs to ROS (TS should be upgraded to TS-4900 to support ROS)

Progress
- Topics
  - ![100%](https://progress-bar.dev/100) lib
  - ![0%](https://progress-bar.dev/0) implementation    
  
- Arduino Comm
  - ![100%](https://progress-bar.dev/100) lib
  - ![0%](https://progress-bar.dev/0) implementation  
  
- To Do
  - ![0%](https://progress-bar.dev/0) Navigation  
  - ![0%](https://progress-bar.dev/0) Control
  - ![0%](https://progress-bar.dev/0) Guidance

### Arduino
Arduino program is maintained by Ardian, last modified by Jason to fix bug of moving mass motor (remove stopping code).




