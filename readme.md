# Mav2Duplex
Version 2.0
(c) Radek Voltr, voltr@voltr.eu , @RVoltr
(c) contribution and fixes - mtbsteve, noradtux, helmarw , see GitHub Repo

### Mav2Ex Changes 

- Arduino 1.8.2 compatible
- Changed value ranges (see bellow)
- Mega32u4 processor supported (Arduino Leonardo and Pro Mini)

### Jetibox Library Changes
- improved timing
- additional hardware support (Mega32u4, Tiny85)
- memory and speed improvements
- resistor 4.7k is not needed and you can use direct wire connection to pin 9 (for example, must support interrupt)
- almost all data formats supported (int22 was ignored as currently non sense)


#### Values and ranges

**-31 to 31** - Armed, GPS Lock type, GPS Sat Count

**-8191 to 8191** - Bat %, Heading

**-819.1 to 819.1** - Climb rate, Speed

**-81.91 to 81.91** - HDOP

**-536870911 to 536870911**  Battery Capacity, Home Distance

**-53687091.1 to 53687091.1**  Altitude, Batt Voltage & Current





---
Version 1.1
(c) DevFor8.com - info@devfor8.com
(c) Contributors, please check history on GitHub for detailed list

Fixes :
- Arduino 1.6.2. compatible
- GPS in EX format fixed (Thanks for cooperation!!)

Hardware 
- Arduino Pro Mini (or similar) with 5V / 16Mhz and ATMega 328
- 4.7k
- GND/RX connection to Mavlink capable board (Arducopter/MegaPirateNG) with OSD telemetry output allowed (same config as MinimOSD)
- servo connector for connection between Arduino and Jeti receiver (insert into telemetry socket)
- Duplex capable transmitter (old JetiBox, new Jeti transmitter or Jetibox Profi)

Troubleshooting 
- Arduino reseting
* measure your voltage for RC. If it is bellow or on 5V, connect to VCC instead of RAW pin on Arduino (!!WARNING , Higher than 5V voltage can smoke your Arduino)

- Speed update on data is slow
* version 1.0 have major improvement in this area, it should be ok

- Speed of EX telemetry is slow than text one
* this is by design of protocol. You can speed it up with commenting lines with     
* JB.addData(ITEMNAME_1,ITEMTYPE_1); and     JB.setValue(1,ITEMVAL_1); (with ID of data which you don't want)
* speed and ammount of data in text protocol will keep same
* We are not recommend to remove Home Dist parameter and have activated automatic record. This can help you with copter find in case of crash


- Future development
* Define base selection of items for Ex protocol





