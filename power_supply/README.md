# Power Supply

This is a simple PCB to test creating several voltages that are used on the Dishy1Pi PCB. Once this cirction is tested as working, the relevant schematic parts will be moved over to the DishyPi1 schematic. The voltages needed are:
- 12V to power each of the 2 encoders. This voltage is supplied from an external power supply and protected by a PTC fuse on the Dishy1Pi PCB. Also reverse polatiry protected.
- 5V to power the logic circuity on the Dishy1Pi PCB. This voltage is generated on this power supply PCB from the 12-24V input voltage using a buck converter.


I need to learn how to design a power supply for a PCB board and get some experience with power supply design.

Project Goals:
- Design a power supply PCB that can take an input voltage range of 12-24VDC
- Output a stable 5VDC at up to 1A
Output clean 12V for each of the 2 encoders
- Create a schematic and PCB layout for the power supply that can be manufactured by JLCPCB.
- Get a board manufactureed and assembled by JLCPCB and test it, verify it works.
- Move the design of the power circuit to the dishy1pi project.

KiCad will be used for this project.

Used TI's WEBENCH Power Designer to design the power supply around the TPS561201DDCR buck converter IC. Link: https://webench.ti.com/power-designer/

