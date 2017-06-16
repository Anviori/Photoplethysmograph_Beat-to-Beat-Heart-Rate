## Photoplethysmograph: Beat-to-Beat Heart Rate With Auto-calibration

### Table of Contents
[Introduction] (##Introduction)

## Introduction: 

A plethysmograph is an instrument for measuring changes in volume within an 
organ of the body. A photoplethysmograph uses optics to achieve this volumetric 
measurement. The signal obtained from such a measurement is called a photoplethys
mogram (PPG).The skin is richly profused, so it is relatively easy to use an LED
 and photodiode or phototransis-tor to detect changes in the blood volume in the
 finger tip caused by the pressure pulse of the car-diac cycle.For example, in a 
 transmissive absorption approach to this measurement, an LED placed on one side
 of the finger tip transmits light through the finger tip and the light is 
 detected on the other side by a phototransistor. The pressure pulse of the 
 cardiac cycle increases the blood volume in the finger tip and more light is 
 absorbed. This reduces the light reaching the phototransistor and causes its 
 collector voltage to increase. The signal at the transistor’s collector is the 
 photoplethys-mogram signal.The DC component of the signal obtained is 
 attributable to the bulk absorption of the skin tissue. The AC component is 
 directly attributable to variation in blood volume in the skin caused by the 
 pressure pulse of the cardiac cycle. The height of AC component of the 
 photoplethysmogram is proportional to the pulse pressure, the difference 
 between the systolic and diastolic pressure in the arteries.Quite a bit of 
 medical information can be derived from a photoplethysmogram. However, the two 
 most commonly measured or derived quantities are heart rate and blood oxygen 
 saturation (SpO2).In this laboratory you will add an analog front end circuit, 
 that is basically a photoplethysmo-graph, to your system. From this you will 
 use input capture to measure the beat-to-beat heart rate.
 
One drawback of the photoplethysmograph system from Laboratory 10 was that the 
100 ohm potentiometer (R1), which sets the current through the LN66F infrared 
LEDs, may need to be adjusted for different individuals. This adjustment is 
necessary to get a voltage in the range 0.3V to 1.0V at the collector, VCE, of 
the LTR-3208E phototransistor when an individual’s finger tip is placed between 
the infrared LEDs and phototransistor.For the system to be more convenient, 
this adjustment needs to be automatic. One way to do this is with an analog 
feedback control circuit. However, since the primary focus of this course is 
on embedded systems design, not analog circuit design, we need to find a way 
to do this calibration automatically in the digital domain.Any feedback 
control system needs a setpoint, the value you want your system to achieve. 
For your particular circuit from Laboratory 10 there was an ideal value for 
the LTR-3208E’s VCEwhen someone’s finger tip was between the infrared diodes 
and phototransistor. A voltage divider connected to the supply voltage, which 
consists of a fixed resistor and a mechanical potentiome-ter, can be adjusted 
to create a voltage equal to the ideal VCE. This voltage is the setpoint 
voltage. The expectation is that this mechanical potentiometer would be 
adjusted when the system is built and would not, typically, require subsequent 
adjustment.Next you need a way to electronically, not mechanically, adjust the 
current through the LEDs so that the LTR-3208E’s VCE is equal to the setpoint 
voltage. To accomplish this, the mechanical potentiometer R1, from Laboratory 
10, is replaced by a ZTX555 PNP transistor in a high side drive configuration. 
This is similar to the approach used to digit drive the multiplexed 
seven-seg-ment LED display in previous laboratories. However, instead of 
having a single fixed base resis-tor, the base resistance consists of a fixed 
resistor in series with a MAX5402 digital potentiometer. 

A digital potentiometer is also called a digitally controlled variable 
resistor (VR) or an RDAC. The high end (H) of the digital potentiometer is 
connected to the fixed resistor and the wiper (W) is connected to PC1. PC1 
must be 0 to turn the infrared LEDs ON. The low end (L) of the MAX5402 is 
left unconnected. By changing the value of resistance between W and H, the 
ZTX555’s base current can be changed. This changes the ZTX555’s collector 
current and, in turn, the LTR-3208E’s VCE.An 8-bit value must be sent, 
serially, to the MAX5402 to set its resistance. These 8 bits determine the 
position of the MAX5402’s wiper. An 8-bit value allows one of 256 positions 
to be specified. The 8-bit value is sent serially using the serial peripheral 
interface (SPI) protocol. Thus, your pro-gram can set the resistance between 
H and W of the MAX5402.The ATmega16A contains an analog comparator that can 
be used to compare the LTR-3208E’s VCE with the setpoint voltage. The 
comparator’s output indicates whether VCE is above or below the setpoint. By 
looking at the comparator’s output, your program can determine whether to 
decrease the ZTX555’ base resistance to decrease LTR-3208E’s VCE, or increase 
the base resis-tance to increase the LTR-3208E’s VCE.A better approach for 
comparing VCE to its setpoint value would be to simply make a connection from 
the collector of the LTR-3208E to one of the ATmega16A’s analog-to-digital 
converter (ADC) inputs and measure the LTR-3208E’s VCE. The measured value 
could then be compared with a program constant equal to the setpoint value. 
However, learning to use the ATmega16A’s comparator is simpler than learning 
to use its ADC.The objective of automatic calibration will be accomplished in 
the laboratory in three tasks. The first two tasks are just for you to become 
familiar with the operation of the MAX5402 and the operation of the 
ATmega16A’s analog comparator.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=YOUTUBE_VIDEO_ID_HERE
" target="_blank"><img src="http://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg" 
alt="Project Demo" width="240" height="180" border="10" /></a>