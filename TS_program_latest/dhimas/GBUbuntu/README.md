# Final (?) Version


run : `./run` 

build : `make`

# Progress

- Communication 
  - ![90%](https://progress-bar.dev/90) GCS   
  - ![50%](https://progress-bar.dev/80) BB  
  - ![70%](https://progress-bar.dev/70) AR  
- PID 
  - ![80%](https://progress-bar.dev/100) lib  
  - ![0%](https://progress-bar.dev/50) implementation  
- Navigation
  - ![100%](https://progress-bar.dev/100) lib  
  - ![0%](https://progress-bar.dev/50) implementation  
- Guidance
  - ![100%](https://progress-bar.dev/100) lib   
  - ![0%](https://progress-bar.dev/50) implementation 
- ![60%](https://progress-bar.dev/75) FSM
- Logging 
  - ![100%](https://progress-bar.dev/100) lib
  - ![40%](https://progress-bar.dev/40) implementation  
- Failsafe
  - ![100%](https://progress-bar.dev/100) battery
  - ![100%](https://progress-bar.dev/100) connection loss
  - ![100%](https://progress-bar.dev/100) internal error
  - ![40%](https://progress-bar.dev/40) recovery
 

Structure : 

```
GaneshBlue
├── build
│   ├── apps
│   │   └── GaneshBlue
│   └── objects
├── configurations
│   ├── ballast.cfg
│   ├── bladder.cfg
│   ├── rudder.cfg
│   ├── AR.cfg
│   ├── BB.cfg
│   ├── GS.cfg
│   └── gliderConstants.cfg
├── include
│   ├── DebugTools.hpp
│   ├── main.hpp
│   ├── moduleCommunication
│   │   ├── OperationParameter.hpp
│   │   └── OperationWaypoint.hpp
│   ├── moduleControl
│   │   ├── Eigen
│   │   ├── NavGdn.hpp
│   │   └── PID.hpp
│   ├── moduleGlider
│   │   ├── libGlider.hpp
│   │   └── libSensor.hpp
│   └── Utils.hpp
├── logs
├── Makefile
└── src
    ├── main.cpp
    ├── moduleCommunication
    │   ├── OperationParameter.cpp
    │   └── OperationWaypoint.cpp
    ├── moduleControl
    │   ├── NavGdn.cpp
    │   └── PID.cpp
    ├── moduleGlider
    │   ├── libGlider.cpp
    │   └── libSensor.cpp
    └── Utils.cpp
```

Config:
  change PID and glider constants value in file located in `configurations/`