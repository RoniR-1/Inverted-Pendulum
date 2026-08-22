# Inverted Pendulum Balancing System

An automated mechatronic project built with an Arduino Uno. The goal of this system is to balance a freely rotating rod upright on a cart by constantly moving the cart left and right. 

It uses real-time feedback from a 12-bit magnetic angle encoder over I2C and a PID control loop running inside a fast execution frame to keep the pendulum balanced on a rail.

![Inverted Pendulum Demo](demo.gif) put and pics here

---

## Hardware Used

* **Microcontroller:** Arduino Uno Rev3
* **Angle Sensor:** AS5600 12-Bit Magnetic Rotary Encoder over I2C (400 kHz fast mode). Using a magnetic encoder allows full 360-degree rotation, unlike a standard potentiometer which is limited to roughly 270 degrees.
* **Encoder Magnet:** Upgraded to a 6 mm diametrically magnetized magnet (the 4 mm magnet that came with the sensor was too small to reliably register).
* **Motor:** NEMA 17 Stepper Motor (24V / 1.5A, 1.8° step angle).
* **Motor Driver:** TMC2209 Stepper Driver.
* **Limit Switch:** OMRON SS-5GL microswitch for zero-position homing.
* **Rail & Cart Setup:** 
  * 100 cm 2020 V-slot aluminum extrusion rail.
  * GT2 16-tooth timing pulley attached to the motor on one end.
  * GT2 idler pulley attached to a metal rod and ball bearing on the other end.
  * Custom 3D-printed end mounts and a gantry plate (cart) sliding on top of the rail, connected via a GT2 belt.
  * Pendulum rod and magnetic encoder mounted directly on top of the cart.
* **Power Supply:** 24V / 5A DC power supply adapter.
* **Miscellaneous:** Breadboard and jumper wires.

---

## Software & Control Architecture

### 1. 400 Hz Control Loop
The main code loop is set to execute every **2.5 ms (400 Hz)**. Each cycle performs four steps:
1. **Read Sensor:** Reads the 12-bit raw angle from the AS5600 encoder over I2C.
2. **Calculate Error:** Compares the current angle against the target upright angle set during calibration.
3. **Filter Derivative Noise:** Applies a **Low-Pass Filter ($\alpha = 0.25$)** to the derivative term. This smooths out small sensor reading spikes and prevents the motor from vibrating or chattering.
4. **Drive Motor:** Calculates the required step timing and pulses the motor step pin evenly across the remaining loop time using direct port manipulation (`PORTB`) for maximum execution speed.

### 2. Automated Homing & Calibration
* **Position Homing:** On startup, the cart drives left until it hits the limit switch to find its `0 cm` home point. It then moves to the calculated center of the rail (`45 cm`). Safety checks in code automatically cut motor power if the cart moves past `0 cm` or beyond the safe rail length.
* **Angle Calibration:** To calibrate vertical zero, hold the pendulum upright by hand and press the limit switch button. The code reads 50 angle samples over 250 ms, averages them to lock in the target balance angle, and starts the PID loop.

---

## Limitations & Lessons Learned

### System Bottlenecks
* **Cart Speed:** The main performance limitation is motor and cart responsiveness. The cart currently moves too slowly to recover if the pendulum tilts past a slight angle. 
* **Torque & Speed Attempts:** Upgraded the motor from 0.15 Nm to 0.55 Nm of torque and increased system voltage from 12V to 24V. While performance improved, it still feels sluggish—likely due to a combination of overall cart weight, mechanical friction on the rail, and stepper motor torque drop-off at higher speeds.
* **Off-Axis Pendulum Wobble:** The pendulum rod slightly wobbles forward and backward (toward and away from the camera axis), which introduces slight reading inaccuracies into the magnetic encoder.

### Practical & Electrical Tips
* **Power Capacitor:** Always wire a capacitor in parallel across the power supply input lines near the motor driver to protect the electronics from voltage spikes.
* **Wiring Safety:** Never plug or unplug wires while the power supply is turned on, especially high-current motor wires. (Fried my first motor driver this way.)
* **3D Printing Design:** Always factor in printing tolerances and think carefully about assembly steps before printing mechanical mounts.

---

## Author

**Roni Rachid**  
B.Sc. Computer Science & Engineering Student | Delft University of Technology (TU Delft)  
* **Contact:** [ronirachid459@gmail.com](mailto:ronirachid459@gmail.com) | [LinkedIn Profile](https://www.linkedin.com/in/roni-rachid-4ab896378)
