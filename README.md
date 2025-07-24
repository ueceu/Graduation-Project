# Graduation-Project
The aim of this project is to develop a temperature control system based on a microcontroller platform using three distinct control strategies: PID, On/Off, and Slow PWM.
The project focuses on the implementation, comparison, and analysis of these control algorithms through practical experimentation and system modeling. This work contributes to the field of applied control systems by targeting to develop a prototype of industrial heating controller.

INTRODUCTION

This project focuses on one of the fundamental mechanisms in industrial automation: temperature control. The system is designed as a closed-loop environment, where the temperature is regulated using three control strategies: PID (Proportional Integral Derivative), On/Off, and Slow PWM.


SIMULATION STUDIES

Within the scope of the project, simulation studies were conducted in MATLAB Simulink and Better Serial Plotter environments to observe the behavior of the temperature control system in advance and to analyze the effects of control algorithms. We calculated transfer function of the plant by use of real time temperature data and System Identification Toolbox. Using ready-made software packages, the system was modeled under conditions similar to the real setup. This approach allowed for a reliable analysis of system behavior prior to physical experimental testing.

<img width="1064" height="369" alt="image" src="https://github.com/user-attachments/assets/1c66962f-e01e-4285-b03f-f231fc1dc109" />
<img width="424" height="278" alt="image" src="https://github.com/user-attachments/assets/9ab3952f-7f16-4d0e-a23e-99e5dcfbfb6a" />



PID TUNING

In the simulation environment, PID tuning was performed to optimize the system's dynamic response. By adjusting the proportional (Kp), integral (Ki), and derivative (Kd) gains, the system was calibrated to reach the setpoint with minimal overshoot and steady-state error. The final optimized parameters were determined as Kp = 1.6, Ki = 0.2, and Kd = 3.5, resulting in a stable and responsive control performance. Graph showing 87.12% similarity between our real system data and the transfer function we modeled.

<img width="460" height="380" alt="image" src="https://github.com/user-attachments/assets/bb459ad0-b453-4c5c-8c78-f9ffddae8b23" />
<img width="694" height="432" alt="image" src="https://github.com/user-attachments/assets/5f8d59dd-604e-47ab-815b-ec08e6f0e8d6" />



PROJECT IMPLEMENTATION & DESIGN

The system begins by measuring temperature using a PT100 sensor. This sensor’s analog resistance changes with temperature, and the MAX31865 module converts these changes into precise digital signals. These signals are sent to an Arduino microcontroller, which acts as the central control unit.

Depending on the selected control method—On/Off, PID, or Slow PWM—the Arduino calculates the appropriate response. It then drives a Solid State Relay (SSR), which switches a heating element (a light bulb powered by a 12V DC supply) on or off, or modulates it through PWM.

An I2C-based LCD screen and four control buttons (Menu, Up, Down, Confirm) form the Human Machine Interface (HMI). This interface allows the user to navigate menus, view live temperature data, and enter control parameters such as set temperature, hysteresis, or PID constants (Kp, Ki, Kd). Through this interface, the user can monitor and adjust the system in real time.

<img width="867" height="615" alt="image" src="https://github.com/user-attachments/assets/7156f341-7478-4442-a9f1-579c72203f87" />



SYSTEM WORKFLOW

• User Interface: The user selects the control mode (On/Off, PID, or Slow PWM) using an LCD and buttons.
• Temperature Sensing: PT100 sensor detects temperature; MAX31865 converts it to digital via SPI.
• Control Logic: Arduino processes data and runs the selected control algorithm.
• Output Control: A signal is sent to the SSR to control the 12V heater (bulb).
• Feedback Loop: Temperature is updated continuously, and the system maintains it around the setpoint.
• Monitoring: Live data is shown on the LCD and via serial monitor.



SOFTWARE IMPLEMENTATION 

* PID

In PID mode, the user can input custom PID parameters (Kp, Ki, Kd) and set a desired temperature for the system to maintain. PID control evaluates the error signal using proportional, integral and derivative components. It uses PID coefficients (Kp, Ki, Kd) to optimize the system performance. PID control output:

𝒖(𝒕) = 𝑲𝒑 𝒆(𝒕) + 𝑲𝒊 ∫ 𝒆(𝒕)𝒅𝒕 + 𝑲𝒅 (𝒅𝒆(𝒕)/𝒅𝒕)

The advantages of this algorithm are fast response, error correction over time and increased system stability.

<img width="519" height="210" alt="image" src="https://github.com/user-attachments/assets/7a8e1348-8406-4879-a8cd-da6bf1daa995" />
<img width="475" height="207" alt="image" src="https://github.com/user-attachments/assets/649b007d-c999-4670-a62c-615f006d6fcd" />
<img width="736" height="347" alt="image" src="https://github.com/user-attachments/assets/c9139603-154a-4b45-8296-b8d788e52dcc" />



* ON/OFF

In On/Off mode, the user can set both the target temperature and the hysteresis value.

<img width="736" height="348" alt="image" src="https://github.com/user-attachments/assets/9ba45c08-3b48-4eed-9bf3-bbefc105588a" />



* SLOW PWM

In Slow PWM mode, only the desired temperature is required, and PWM signals are generated to control the heating element.

<img width="614" height="291" alt="image" src="https://github.com/user-attachments/assets/e06e9881-8cf5-4b0b-9acb-2fc314e2891a" />


