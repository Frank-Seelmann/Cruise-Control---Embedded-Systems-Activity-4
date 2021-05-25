# Cruise-Control---Embedded-Systems-Activity-4

Introduction

For this activity, students were tasked with iterating on their previous activity by adding
cruise control functionality. The cruise control system is meant to, when activated, ignore
the input signal which would determine the output to the motor and instead hold the
motor at the same frequency. It also needs to compensate for resistances that change the
frequency of the motor in order to maintain the selected frequency.

Design

The setup for this activity is largely the same as the previous: the NUCLEO board takes
in analog input, outputs a proportional PWM output to a ULN2003 Darlington Transistor
which connects to the motor and battery, thus driving the motor at a speed determined by
the original analog input. There is also an input capture device which is used to measure
the period between revolutions of the motor, which is used to calculate the frequency and
RPMs of the motor, and output this information to the LCD. There are two hardware
additions for this activity: a switch and a potentiometer.

The switch is used to determine whether cruise control is on or off, and uses PA8. The
program checks the status of the switch and changes a cruise control flag, “cc,”
accordingly. The potentiometer is used as a rheostat, i.e. variable resistance, between the
battery and the motor. This is used to provide an electrical resistance (in lieu of a physical
resistance) to the motor, which affects the speed of the motor, which the cruise control
system will then need to compensate for.

When the cruise control is enabled, the cc flag is set and the current frequency of the
motor is saved in another variable, “ccFrequency.” Since the flag is set, the PWM output
will now be increased or decreased based on current frequency and the ccFrequency. If
the frequency is less than the ccFrequency, the PWM output is increased by 50, and if it is
greater than the ccFrequency, the PWM output is decreased by 50. This is simple
“bang-bang” or “on-ff” control. In order to get the current speed to the target speed faster,
there is some additional logic, which is a check on how far the frequency and the
ccFrequency are from one another. If the difference is more than 20% of the ccFrequency,
the PWM output is increased/decreased by an additional 200 (for a total of 250). This
means the motor will accelerate or decelerate to the desired speed quickly until it is
within 20% of it, at which point it will begin approaching the desired speed more slowly.

Results

As shown in the verification video, https://www.youtube.com/watch?v=PKVoc73Y1wY, the cruise 
control functionality works very well, including the increased acceleration/deceleration when 
over ±20% of the target frequency. When the resistance on the motor is increased the motor 
slows down, then the cruise control compensates for it by increasing the PWM output, and 
vice versa when the resistance is decreased.

Conclusion

The system works as anticipated, achieving the goals set plus some extra logic to make it
work better.
