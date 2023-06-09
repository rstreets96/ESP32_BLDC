# ESP32_BLDC #
Hello! I made this project to showcase a bit of what I've learned throughout my career, and maybe build my knowledge 
of FOC motor control along the way. Below, you'll find a summary of the project as well as an in depth explanation of
the control process.

## Summary 
This project is an FOC motor controller for the ESP32S3, using the DRV8329A gate driver. This will use Single-Shunt 
current reconstruction, Clarke and Park transforms, a Luenberger Observer, PID controllers, Inverse Clarke and Park
transforms, and Space-Vector Modulation (SVM).

## Control Loop Flow
### Single-Shunt Current Reconstruction 
- - - -
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

### Clarke and Park Transforms
- - - -
When controlling a 3-phase motor, things can get complicated fast. This can be more easily seen in Figure 4 below. The
rotating light-blue line is our goal current vector, the resultant of the A, B, and C vectors. To make it simpler, the 
Clarke and Park transforms are able to take these three AC values and effectively control them with two DC values. With 
these two values, we can then manipulate the motor however we want and the transforms will handle the complexity 
for us!

![3 Phase GIF](/Images/ThreePhase.GIF)
#### Figure 4. Rotating 3 Phase GIF [\(Source\)](https://www.mathworks.com/solutions/electrification/clarke-and-park-transforms.html)

####Clarke Transform
The first step is the Clarke Transform. This transforms the system from the three phases and axes into a two phase system. 
As shown below, the light blue vector already becomes simpler to create. Since this now resembles a Cartesian coordinate 
plane (Y vs X), this is also the domain that is most often used for calculating the rotor's current angle (more on that later).
Instead of x and y, we use alpha and beta.

![Alpha-Beta GIF](/Images/AlphaBeta.GIF)
#### Figure 5. Rotating Alpha-Beta GIF [\(Source\)](https://www.mathworks.com/solutions/electrification/clarke-and-park-transforms.html)

The equations used to complete this transform come from simple trigonometric functions, but are often displayed in matrix forms,
which can get confusing. Below, I've included the simpler form of the equations from my code. Here, phases.a, .b and .c are the 
three phase components, MATH_ONE_OVER_THREE is 1/3, and MATH_ONE_OVER_ROOT_THREE is 1/sqrt(3).

![Clarke Math Code](/Images/ClarkeMath.PNG)
#### Figure 6. Clarke Transform Code Snippet

####Park Transform
From the output of the Clarke transform, we can then use the Park transform to implement a rotating frame of reference. This means 
that our two DC values will simply rotate along with our desired resultant vector. See below for a visualization of this concept.

![Dir-Quad GIF](/Images/DirQuad.GIF)
#### Figure 7. Rotating Direct-Quadrature GIF [\(Source\)](https://www.mathworks.com/solutions/electrification/clarke-and-park-transforms.html)

With this new transformation, a new variable that needs to be tracked gets introduced: the angle between the rotating frame of reference
and the stationary frame of reference from before. In the animation above, this angle is always increasing from 0 to 2PI, and then dropping 
back to 0. When the d and q axes are at the same points as the alpha and beta axes from before, the angle is 0. With the knowledge of this 
angle, we can set constant values on our d and q axes, and the output will be the same resultant vector. See below for the equations of this
transform, where theta_rad is the angle between references and alpha_beta.alpha and .beta are the alpha and beta outputs of the Clarke
transform.

![Park Math Code](/Images/ParkMath.PNG)
#### Figure 8. Park Transform Code Snippet