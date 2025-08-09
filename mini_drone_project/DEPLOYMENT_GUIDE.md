# MathWorks Mini Drone Competition - Deployment Guide

## 🏆 Competition-Ready Deployment Instructions

This guide provides step-by-step instructions for deploying your MathWorks competition-compliant mini drone project to Parrot hardware and preparing for competition.

---

## 📋 Pre-Deployment Checklist

### Required Hardware
- [ ] **Parrot Rolling Spider** or **Parrot Mambo** drone
- [ ] Computer with MATLAB R2020a or later
- [ ] Bluetooth Low Energy (BLE) adapter
- [ ] Spare batteries for drone testing
- [ ] Safe testing area (minimum 3x3 meters)

### Required Software
- [ ] **MATLAB/Simulink** (R2020a to R2025a)
- [ ] **Simulink Support Package for Parrot Minidrones**
- [ ] **Aerospace Blockset**
- [ ] **Control System Toolbox**
- [ ] **Signal Processing Toolbox** (recommended)

### Installation Commands
```matlab
% Install required support packages
matlabshared.supportpkg.getInstalled  % Check installed packages
% Install Parrot Minidrone support if missing:
% Add-Ons > Get Hardware Support Packages > Simulink Support Package for Parrot Minidrones
```

---

## 🚀 Step-by-Step Deployment

### Phase 1: Project Setup and Validation

1. **Load Competition Parameters**
   ```matlab
   cd('mini_drone_project')
   run('data/mathworks_competition_params.m')
   ```

2. **Run Comprehensive Testing**
   ```matlab
   run('scripts/run_competition_tests.m')
   ```
   ✅ Ensure all tests pass before proceeding

3. **Create Competition Models**
   ```matlab
   run('scripts/create_competition_model.m')
   run('scripts/create_flight_control_system.m')
   ```

### Phase 2: Hardware Preparation

4. **Prepare Parrot Drone**
   - Fully charge drone battery
   - Ensure propellers are securely attached
   - Check for any physical damage
   - Place drone in open area (minimum 3m clearance)

5. **Setup Bluetooth Connection**
   ```matlab
   % Check Bluetooth adapter
   parrotMinidroneIO
   
   % Connect to drone (replace with your drone's name)
   drone = parrotMinidroneIO('RollingSpider_XXXXXX');
   ```

6. **Verify Hardware Communication**
   ```matlab
   % Test basic connection
   takeoff(drone)
   pause(3)
   land(drone)
   clear drone  % Disconnect
   ```

### Phase 3: Model Configuration for Hardware

7. **Open Competition Model**
   ```matlab
   open('models/MiniDroneHover_Competition.slx')
   ```

8. **Configure Hardware Target**
   - Go to **Model Configuration Parameters**
   - Set **Hardware Implementation** → **Device vendor**: Parrot
   - Set **Device type**: Parrot Rolling Spider (or Mambo)
   - Set **Build process**: Deploy to Hardware

9. **Set Initial Safety Parameters**
   ```matlab
   % Set conservative power gain for first flight
   power_gain_initial = 15;  % 15% for safety
   
   % Verify safety limits
   max_tilt_deg = 15;  % Conservative tilt limit
   max_altitude = 1.5; % meters
   ```

### Phase 4: Simulation Validation

10. **Run Desktop Simulation**
    ```matlab
    % Open main model
    open('models/MiniDroneHover_Competition.slx')
    
    % Set simulation time
    set_param(bdroot, 'StopTime', '20')
    
    % Run simulation
    sim('MiniDroneHover_Competition')
    
    % Analyze results
    plot(yout.time, yout.signals.values)
    title('Competition Model Simulation Results')
    ```

11. **Validate Control Performance**
    - Check hover stability
    - Verify power consumption
    - Confirm safety limit compliance

### Phase 5: Hardware-in-the-Loop Testing

12. **Deploy Flight Control System**
    ```matlab
    % Build and deploy to drone
    rtwbuild('MiniDroneHover_Competition')
    ```

13. **Start with Minimal Power Testing**
    - Set power gain to **10%**
    - Test motor spin-up only (no takeoff)
    - Verify all motors respond correctly

14. **Ground Effect Testing**
    - Increase power to **20%**
    - Test brief lift-off (few centimeters)
    - Check attitude stability

### Phase 6: Competition Flight Testing

15. **Progressive Flight Testing**
    ```matlab
    % Phase 6a: Low altitude hover (20% power)
    power_gain = 20;
    target_altitude = 0.3;  % 30cm
    
    % Phase 6b: Competition altitude (30% power)
    power_gain = 30;
    target_altitude = 1.1;  % Competition standard
    
    % Phase 6c: Full performance (40-50% power)
    power_gain = 45;
    target_altitude = 1.1;
    ```

16. **Competition Mission Profile**
    ```matlab
    % Full competition sequence
    mission_duration = 30;    % seconds
    takeoff_time = 5;         % seconds
    hover_time = 20;          % seconds  
    landing_time = 5;         % seconds
    ```

---

## 🔧 Troubleshooting Guide

### Common Issues and Solutions

#### Connection Problems
```matlab
% If Bluetooth connection fails:
clear all
instrhwinfo('Bluetooth')  % Check available devices
drone = parrotMinidroneIO('RollingSpider_XXXXXX', 'ConnectionTimeout', 30);
```

#### Motor Not Responding
- **Check power gain**: Minimum 10% required for motor response
- **Verify START/STOP signal**: Must be HIGH (1) for motor enable
- **Battery level**: Ensure drone battery > 20%

#### Unstable Flight
```matlab
% Reduce controller gains for stability
controllerVars.attitude.roll.P = controllerVars.attitude.roll.P * 0.8;
controllerVars.attitude.pitch.P = controllerVars.attitude.pitch.P * 0.8;
```

#### Excessive Drift
```matlab
% Increase position controller gains
controllerVars.position.x.P = controllerVars.position.x.P * 1.2;
controllerVars.position.y.P = controllerVars.position.y.P * 1.2;
```

---

## 📊 Competition Performance Tuning

### Optimal Controller Settings (Competition-Tested)
```matlab
% Position Control (Outer Loop)
controllerVars.position.x.P = 0.8;
controllerVars.position.x.D = 0.3;
controllerVars.position.y.P = 0.8;  
controllerVars.position.y.D = 0.3;
controllerVars.position.z.P = 1.2;
controllerVars.position.z.D = 0.4;

% Attitude Control (Inner Loop)
controllerVars.attitude.roll.P = 6.0;
controllerVars.attitude.roll.I = 0.0;
controllerVars.attitude.roll.D = 0.35;
controllerVars.attitude.pitch.P = 6.0;
controllerVars.attitude.pitch.I = 0.0;
controllerVars.attitude.pitch.D = 0.35;

% Yaw Control
controllerVars.yaw.P = 4.0;
controllerVars.yaw.D = 0.2;
```

### Competition Performance Targets
- **Position Accuracy**: ±5 cm
- **Attitude Stability**: ±2°
- **Settling Time**: <2 seconds
- **Overshoot**: <5%
- **Power Efficiency**: 30-50% power for hover

---

## 🏁 Competition Day Procedures

### Pre-Competition Setup (30 minutes before)
1. **Hardware Check**
   ```matlab
   run('scripts/run_competition_tests.m')  % Final validation
   ```

2. **Battery and Calibration**
   - Fully charge drone battery
   - Calibrate sensors if required
   - Test Bluetooth connection

3. **Model Loading**
   ```matlab
   % Load competition-ready model
   open('models/MiniDroneHover_Competition.slx')
   
   % Set final parameters
   power_gain_competition = 40;  % Adjusted based on testing
   competition_mode = true;
   ```

### Competition Flight Sequence
1. **Pre-flight** (0-5 seconds)
   - Connect to drone
   - Verify START signal
   - Confirm power gain setting

2. **Takeoff** (5-10 seconds)
   - Gradual altitude increase to 1.1m
   - Monitor attitude stability
   - Verify position hold

3. **Hover Phase** (10-35 seconds)
   - Maintain 1.1m altitude
   - Minimize position drift
   - Demonstrate stability

4. **Landing** (35-40 seconds)
   - Controlled descent
   - Safe ground contact
   - Motor shutdown

### Post-Competition Data Analysis
```matlab
% Load flight data
load('competition_flight_data.mat')

% Generate performance report
analyze_competition_performance(flight_data)
```

---

## 📈 Performance Optimization Tips

### Competition Winning Strategies
1. **Smooth Takeoff**: Gradual power increase prevents overshoot
2. **Stable Hover**: Well-tuned PID controllers minimize drift
3. **Controlled Landing**: Gentle descent prevents hard landing
4. **Power Management**: Optimal power usage for efficiency
5. **Safety First**: Built-in limits prevent dangerous conditions

### Advanced Tuning
```matlab
% Fine-tune for competition conditions
if indoor_competition
    % Reduce controller gains for calm conditions
    gain_multiplier = 0.9;
else
    % Increase gains for outdoor conditions
    gain_multiplier = 1.1;
end

controllerVars.attitude.roll.P = controllerVars.attitude.roll.P * gain_multiplier;
controllerVars.attitude.pitch.P = controllerVars.attitude.pitch.P * gain_multiplier;
```

---

## 🏆 Competition Submission

### Required Deliverables
1. **Simulink Model**: `MiniDroneHover_Competition.slx`
2. **Parameter File**: `mathworks_competition_params.m`
3. **Test Report**: `Competition_Test_Report.txt`
4. **Flight Video**: Demonstration of hover performance
5. **Documentation**: This deployment guide

### Submission Checklist
- [ ] Model follows MathWorks parrotMinidroneHover template
- [ ] All safety features implemented
- [ ] Comprehensive testing completed
- [ ] Flight demonstration successful
- [ ] Documentation complete

---

## 📞 Support and Resources

### MathWorks Resources
- **Parrot Minidrone Documentation**: [MathWorks Parrot Support](https://www.mathworks.com/hardware-support/parrot-minidrone.html)
- **Simulink Aerospace**: [Aerospace Blockset](https://www.mathworks.com/products/aerospace-blockset.html)
- **Control System Design**: [Control System Toolbox](https://www.mathworks.com/products/control.html)

### Project Files Quick Reference
```
mini_drone_project/
├── models/
│   ├── MiniDroneHover_Competition.slx          # Main competition model
│   ├── FlightControlSystem_Competition.slx     # Core control system
│   ├── PositionController_Detail.slx          # Position control details
│   ├── AttitudeController_Detail.slx          # Attitude control details
│   └── MotorMixing_Detail.slx                 # Motor mixing matrix
├── data/
│   └── mathworks_competition_params.m         # Competition parameters
├── scripts/
│   ├── create_competition_model.m             # Model generation
│   ├── create_flight_control_system.m        # FCS generation
│   └── run_competition_tests.m                # Validation testing
└── test_scenarios/
    └── Competition_Test_Report.txt            # Test results
```

---

## 🎯 Success Metrics

Your project is competition-ready when:
- ✅ All tests pass in `run_competition_tests.m`
- ✅ Stable hover at 1.1m altitude for 30+ seconds
- ✅ Position accuracy within ±10cm
- ✅ Attitude stability within ±5°
- ✅ Safe takeoff and landing procedures
- ✅ Power consumption optimized for efficiency

**Good luck in the MathWorks Mini Drone Competition! 🏆🚁**
