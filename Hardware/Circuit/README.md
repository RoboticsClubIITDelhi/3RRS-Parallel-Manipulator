This folder contains the circuit diagram.<br>
<h2>Important points</h2>
<ul>
  <li>
   Encoders work based on interrupts, so use a microcontroller with enough interrupt pins; not all pins can handle interrupts.
   In Arduino Uno, there are only 2 interrupt pins which is why we used Arduino Mega, which has 6 pins.
  </li>
  <li>
    Might have to switch the motor driver to a bigger capacity model if using both channels of the driver and using heavy motors.
  </li>
</ul>
