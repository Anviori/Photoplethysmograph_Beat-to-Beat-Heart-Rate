## Photoplethysmograph: Beat-to-Beat Heart Rate With Auto-calibration

### Table of Contents

- [Photoplethysmograph: Beat-to-Beat Heart Rate With Auto-calibration](#photoplethysmograph:-beat-to-beat-heart-rate-with-auto-calibration)
- [Introduction](#introduction)
- [Schematics](#schematics)
- [Demo Video](#demo-video)

## Introduction 

AVR Assembler was used to program an ATmega16A to build an embedded system 
(A plethysmograph) that monitors heart rate with auto-calibration.

*A plethysmograph is an instrument for measuring changes in volume within an 
organ of the body.*

The system uses optics to obtain a photoplethysmogram (PPG) (The a signal 
obtained by the plethysmograph measurement).

To aquire this signal, two Infrared LEDs are used to transmit light 
through the finger tip and an Infrared Photo-transistor detects the light on 
the other side. The pressure pulse of the cardiac cycle increases the blood 
volume in the finger tip and more light is absorbed. This light causes its collector 
voltage of the photo-transistor to increase. This voltage value is the 
photoplethysmogram signal used to derive the heart rate. The DC component of the signal 
obtained is attributable to the absorption of the skin tissue. The AC component is 
attributable to the variation in blood volume in the skin caused by the pressure pulse of 
the cardiac cycle.

The two most common measurements derived from a photoplethysmogram signal are heart rate 
and blood oxygen saturation (SpO2). This system includes an analog front end circuit that
models a photoplethysmo-graph and, using the MCU's input capture feature, measures the 
heart rate (*The schematic is shown in Figure-1 Below*)

The infrared LED's used a 100 ohm potentiometer to set the current. This needed to be replaced
to allow automatic adjustments in current through the LEDs for different individuals. First,
an ideal value for the photo-transistor must be found when someones finger tip is between the
infrared diodes and the photo-transistor. A voltage divider connected to the supply voltage is
adjusted to aquire the ideal voltage. This voltage is the set-point voltage. 

A digital potentiometer is used to automatically adjust the current through the infrared LEDs 
until the photo-transistor's collector voltage is equal to the set-point voltage; a voltage
between 0.3V and 1.0V.(*The schematic is shown in Figure-2 Below*)

The system measures heart rates from 40 to 200 bpm and displays them on an LCD with a resolution 
of 0.1 bpm. (*System demo shown below*)

## Schematics

###### Figure-1: Analog Front End Circuit
![alt text](http://i.imgur.com/9PJWSe1.png "Analog front end circuit")

###### Figure-2: Analog Front End Circuit With Digital Potentiometer
![alt text](http://i.imgur.com/OSxhrTG.png "Analog front end circuit")

## Demo Video

[![Project Demo](http://i.imgur.com/WRxF8gb.png)](https://www.youtube.com/watch?v=9BOhHENQnds "Project Demo - Click to Watch")