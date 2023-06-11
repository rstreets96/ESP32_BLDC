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

Continue below for more details on the execution of each of these modules!

## Single-Shunt Current Reconstruction 
Single-Shunt current reconstruction is the use of one current shunt resistor on the DC bus to take precisely timed 
current measurements. Since we know the state of the gates in our inverter circuit, we can determine the current in 
each phase. Figure 1 shows an example state below.

![SingleShunt1 Image](/Images/SingleShunt1.PNG)
#### Figure 1. Positive A Switching State [\(Source\)](https://www.ti.com/lit/an/spract7/spract7.pdf?ts=1683678738002)


As we can see, all of the dc current measured in this state flows in the positive direction through phase A, so we 
know the phase A current! The current then splits through phases B and C on the low side, so no information for those
phases is gained. For another example, let's take a look at Figure 2.

![SingleShunt2 Image](/Images/SingleShunt2.PNG)
#### Figure 2. Negative C Switching State [\(Source\)](https://www.ti.com/lit/an/spract7/spract7.pdf?ts=1683678738002)


From this image, we see that the current splits on the high side (through phases A and B) this time. On the low side, 
however, all of the current flows through phase C. This means that our dc current is equal to the negative of the phase
C current. See below for a full list of switching states and the phase current that's measured in them.

![SingleShunt3 Image](/Images/SingleShunt3.PNG)
#### Figure 3. Single Shunt State Table [\(Source\)](https://www.ti.com/lit/an/spract7/spract7.pdf?ts=1683678738002)

There are timing considerations that become important when using this method, but we'll get to those later.

## Clarke and Park Transforms
When controlling a 3-phase motor, things can get complicated fast. This can be more easily seen in Figure 4 below. The
rotating light-blue line is our goal current vector, the resultant of the A, B, and C vectors. To make it simpler, the 
Clarke and Park transforms are able to take these three AC values and effectively control them with two DC values. With 
these two values, we can then manipulate the motor however we want and the transforms will handle the complexity 
for us!

![3 Phase GIF](/Images/ThreePhase.gif)
#### Figure 4. Rotating 3 Phase GIF [\(Source\)](https://www.mathworks.com/solutions/electrification/clarke-and-park-transforms.html)

### Clarke Transform
The first step is the Clarke Transform. This transforms the system from the three phases and axes into a two phase system. 
As shown below, the light blue vector already becomes simpler to create. Since this now resembles a Cartesian coordinate 
plane (Y vs X), this is also the domain that is most often used for calculating the rotor's current angle (more on that later).
Instead of x and y, we use alpha and beta.

![Alpha-Beta GIF](/Images/AlphaBeta.gif)
#### Figure 5. Rotating Alpha-Beta GIF [\(Source\)](https://www.mathworks.com/solutions/electrification/clarke-and-park-transforms.html)

The equations used to complete this transform come from simple trigonometric functions, but are often displayed in matrix forms,
which can get confusing. Below, I've included the simpler form of the equations from my code. Here, phases.a, .b and .c are the 
three phase components, MATH_ONE_OVER_THREE is 1/3, and MATH_ONE_OVER_ROOT_THREE is 1/sqrt(3).

![Clarke Math Code](/Images/ClarkeMath.PNG)

### Park Transform
From the output of the Clarke transform, we can then use the Park transform to implement a rotating frame of reference. This means 
that our two DC values will simply rotate along with our desired resultant vector. See below for a visualization of this concept.

![Dir-Quad GIF](/Images/DirQuad.gif)
#### Figure 6. Rotating Direct-Quadrature GIF [\(Source\)](https://www.mathworks.com/solutions/electrification/clarke-and-park-transforms.html)

With this new transformation, a new variable that needs to be tracked gets introduced: the angle between the rotating frame of reference
and the stationary frame of reference from before. In the animation above, this angle is always increasing from 0 to 2PI, and then dropping 
back to 0. When the d and q axes are at the same points as the alpha and beta axes from before, the angle is 0. With the knowledge of this 
angle, we can set constant values on our d and q axes, and the output will be the same resultant vector. See below for the equations of this
transform, where theta_rad is the angle between references and alpha_beta.alpha and .beta are the alpha and beta outputs of the Clarke
transform.

![Park Math Code](/Images/ParkMath.PNG)

## Rotor Angle Tracking
To make use of the Park transform I just described, we need the angle of the rotor relative to that stationary Alpha-Beta grid. There are 
many ways to go about this, but they all can be split into two groups: Sensored and Sensorless.

For the sensored method, you need a motor equipped with a position sensor. These sensors can be things like encoders or a set of three hall 
sensors. While this method allows for simple angle detection at any motor speed, it has its downsides. The sensors can add another failure point
to the motor, and the resolution of the angle reading can also be lacking (i.e. the common set of three hall sensors have six different states,
so the rotor angle is only known to be in a 60 degree sector).

Sensorless motor control, however, can provide a more continuous rotor angle approximation without adding extra hardware to the motor. Its
downside is that it often cannot determine rotor position at low speeds or when stopped. Because of this, sensors are often used in combination 
with this method, providing rough angles at low speeds and switching to sensorless at higher speeds.

The sensorless motor control also adds some complexity to the control process. It is often done by tracking the back-emf of the motor, which 
increases in magnitude as the motor spins faster. Unfortunately, the back-emf is not an easily measured signal. In Figure 7 below, the Ra is 
the resistance of the windings, La is the inductance of the windings, and the e(t) function is the back-emf that we're interested in. 
Unfortunately, we only have access to Va(t) and ia(t), so we'll have to determine the back-emf from these values. This is often done by a 
complex control system known as an observer. In this project, I've chosen to use a Luenberger observer. I'll attempt to explain how this works
in the next section.

![Motor Equivalent Circuit](/Images/MotorEquivalentCircuit.PNG)

#### Figure 7. Equivalent Circuit of a Motor [\(Source\)](https://www.researchgate.net/figure/Equivalent-circuit-of-an-armature-controlled-dc-motor_fig1_3050910)

### Luenberger Observer
![Luenberger Observer Form](/Images/Luenberger1.PNG)
#### Figure 8. Luenberger Observer Form [\(Source\)](https://www.ece.rutgers.edu/~gajic/psfiles/observers.pdf)

In Figure 8 above, you can find the general form of a Luenberger observer. 

![Luenberger Observer with Error](/Images/Luenberger2.PNG)
#### Figure 9. Luenberger Observer Form [\(Source\)](https://www.ece.rutgers.edu/~gajic/psfiles/observers.pdf)

![Luenberger Observer Stability](/Images/Luenberger3.PNG)
#### Figure 10. Luenberger Observer Stability [\(Source\)](https://www.ece.rutgers.edu/~gajic/psfiles/observers.pdf)

![Luenberger Observer Motor Equations](/Images/Luenberger4.PNG)
#### Figure 11. Motor Model Fit into Luenberger Observer Form [\(Source\)](https://pcimasia-expo.cn.messefrankfurt.com/content/dam/messefrankfurt-redaktion/pcim_asia/download/ppt_pac2021/Qianbao%20Mi.pdf)
