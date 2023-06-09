# ESP32_BLDC #
Hello! I made this project to showcase a bit of what I've learned throughout my career, and maybe build my knowledge 
of FOC motor control along the way. Below, you'll find a summary of the project as well as an in depth explanation of
the control process.

## Summary ##
This project is an FOC motor controller for the ESP32S3, using the DRV8329A gate driver. This will use Single-Shunt 
current reconstruction, Clarke and Park transforms, a Luenberger Observer, PID controllers, Inverse Clarke and Park
transforms, and Space-Vector Modulation (SVM).

## Control Loop Flow ##
### Single-Shunt Current Reconstruction ###

Single-Shunt current reconstruction is the use of one current shunt resistor on the DC bus to take precisely timed 
current measurements. Since we know the state of the gates in our inverter circuit, we can determine the current in 
each phase. Figure 1 shows an example state below.

![SingleShunt1 Image](images/SingleShunt1.png)
#### Figure 1. Positive A Switching State ####


As we can see, all of the dc current measured in this state flows in the positive direction through phase A, so we 
know the phase A current! The current then splits through phases B and C on the low side, so no information for those
phases is gained. For another example, let's take a look at Figure 2.

![SingleShunt2 Image](images/SingleShunt2.png)
#### Figure 1. Negative C Switching State ####


From this image, we see that the current splits on the high side (through phases A and B) this time. On the low side, 
however, all of the current flows through phase C. This means that our dc current is equal to the negative of the phase
C current.