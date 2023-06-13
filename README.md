Hello! I made this project to showcase a bit of what I've learned throughout my career, and maybe build my knowledge 
of FOC motor control along the way. Below, you'll find a summary of the project as well as an in depth explanation of
the control process.

# Summary 
This project is an FOC motor controller for the ESP32S3, using the DRV8329A gate driver. This will use Single-Shunt 
current reconstruction, Clarke and Park transforms, a Luenberger Observer, a Phase-Locked Loop (PLL), PID controllers, 
Inverse Clarke and Park transforms, and Space-Vector Modulation (SVM). These modules flow together to make the 
overarching control scheme.

The control loop starts with the 3 phase current measurements using Single-Shunt current reconstruction. Once these 
are obtained, the Clarke transform converts them to the Alpha-Beta domain. These values, along with the stator voltage 
in the Alpha-Beta domain (obtained from % duty cycle of the PWMs), are then input into the Luenberger Observer, which 
estimates the Alpha and Beta components of the motor's back-emf. This allows us to calculate the rotor angle relative
to the Alpha-Beta axes, which is then input into the PLL for rotational speed calculation and angle noise filtering.
After this, the angle is used in our Park Transform, to get our measured direct and quadrature current (Id and Iq). The 
speed calculation provides feedback to our Speed PID loop, which provides a setpoint for our Id and Iq PID loops. With 
the measured Id and Iq values as provide feedback, the current PID loops can then output a direct and quadrature voltage.
This converts all the way back to three phase through the Inverse Park and Clarke transforms (saving the Alpha-Beta
domain values for the next Observer loop). Finally, the SVM module converts these voltages to PWM duty cycles, applying
our desired voltage to the motor.

Check out the wiki for more details on the execution of each of these modules!

