# IO-2015
Input/Output card for Teensy 2 ++ used during Cup of Robotics 2015 / Eurobot

This controller handle 12 servo, 1 elevator (encoder, motor, reset switch) and many switchs. It uses two power lines: 5V for logic and 5V for power like servos witch can't be powered by a microcontroller alone.

This module communicate with serial over USB to get instructions and push informations about coders, pwm, etc. The protocol send and receive only one C-structure in each direction containing all information needed. It reads and writes values as soon as it can and communicate each 100ms with the main board.

![](https://raw.githubusercontent.com/werdroid/IO-2015/master/pcb_io2015/pcb_io_2015.png)
