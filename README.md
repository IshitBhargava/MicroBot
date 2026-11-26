# MicroBot
Just a small maze solving robot, configured with manual control as well.

# Goal
Aims to solve a 10ft x 10ft maze with 22cm wall separation (gives around 13 cells) autonomously using built-in sensors and the optimistic flood fill algorithm to efficiently reach the end goal.

# Hardware
<ul>
<li>N20 motors with 60mm omnidirectional wheels.
<li>TB6612FNG drivers
<li>Pi Pico 2 as the MCU
<li>4x VL53L0X sensors for wall detection
<li>GY87 module for sensors.
<li>RC control via PWM from a receiver with 4 or more channels (uses three only)
</ul>

# chassis
fully 3D printed and designed in Fusion (formerly Fusion 360). Refer to <a href="/MicroBot/">this directory</a> for the files.
