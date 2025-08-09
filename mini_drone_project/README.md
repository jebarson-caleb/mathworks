# Mini Drone Simulink Project

## Overview
This is a comprehensive Simulink project for modeling, controlling, and simulating a mini quadcopter drone. The project includes advanced control algorithms, dynamic modeling, and simulation scenarios based on MathWorks best practices.

## Project Structure
```
mini_drone_project/
├── models/                 # Main Simulink models
│   ├── main_drone_model.slx       # Top-level drone simulation
│   ├── drone_dynamics.slx         # 6-DOF drone dynamics
│   ├── sensor_fusion.slx          # IMU and sensor modeling
│   └── environment_model.slx      # Wind and disturbance modeling
├── controllers/            # Control system models
│   ├── pid_controller.slx         # PID control implementation
│   ├── lqr_controller.slx         # Linear Quadratic Regulator
│   ├── mpc_controller.slx         # Model Predictive Control
│   └── attitude_controller.slx    # Attitude control subsystem
├── subsystems/            # Reusable subsystem blocks
│   ├── motor_dynamics.slx         # Motor and propeller model
│   ├── battery_model.slx          # Battery discharge model
│   └── payload_dynamics.slx      # Payload effects
├── scripts/               # MATLAB initialization scripts
│   ├── initialize_drone.m         # Drone parameter initialization
│   ├── control_tuning.m           # Controller parameter tuning
│   ├── simulation_setup.m         # Simulation configuration
│   └── analysis_tools.m           # Post-simulation analysis
├── data/                  # Model parameters and test data
│   ├── drone_parameters.mat       # Physical drone parameters
│   ├── flight_test_data.mat       # Real flight test data
│   └── reference_trajectories.mat # Reference flight paths
└── test_scenarios/        # Test and validation scenarios
    ├── hover_test.slx             # Hovering stability test
    ├── trajectory_following.slx   # Path following test
    ├── disturbance_rejection.slx  # Wind disturbance test
    └── payload_drop.slx           # Payload delivery test
```

## Features
- **6-DOF Drone Dynamics**: Complete nonlinear quadcopter dynamics
- **Advanced Control**: PID, LQR, and MPC controllers
- **Sensor Fusion**: IMU, GPS, and vision-based estimation
- **Environmental Modeling**: Wind disturbances and turbulence
- **Real-time Simulation**: Hardware-in-the-loop capabilities
- **Validation Scenarios**: Comprehensive test suite

## Requirements
- MATLAB R2023a or later
- Simulink
- Aerospace Blockset
- Control System Toolbox
- DSP System Toolbox
- Simulink Control Design
- Computer Vision Toolbox (optional for vision features)

## Getting Started
1. Open MATLAB and navigate to the project directory
2. Run `startup.m` to initialize the project
3. Open `models/main_drone_model.slx` to start simulation
4. Explore different controllers in the `controllers/` folder
5. Run test scenarios from `test_scenarios/` folder

## Model Overview
The drone model includes:
- Rigid body dynamics with 6 degrees of freedom
- Four brushless motor dynamics
- Propeller aerodynamics
- Battery discharge characteristics
- Sensor noise and bias modeling
- Environmental disturbances

## Control Strategies
1. **PID Control**: Traditional approach with separate controllers for position and attitude
2. **LQR Control**: Optimal linear control for enhanced performance
3. **MPC Control**: Model predictive control for constraint handling
4. **Cascade Control**: Hierarchical control structure (position → attitude → motor commands)

## Simulation Scenarios
- Takeoff and landing sequences
- Hovering with disturbances
- Trajectory tracking (waypoint navigation)
- Emergency scenarios (motor failure, low battery)
- Payload delivery missions

## Validation and Testing
The project includes comprehensive validation through:
- Unit tests for individual subsystems
- Integration tests for complete system
- Monte Carlo simulations for robustness analysis
- Comparison with real flight test data

## Contributing
When adding new features or models:
1. Follow MathWorks modeling guidelines
2. Include proper documentation
3. Add corresponding test scenarios
4. Update this README with new features

## License
This project is for educational and research purposes.

## Contact
For questions or contributions, please refer to the MathWorks documentation and community forums.
