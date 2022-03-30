# MusselHeart
Mussel heart rate sensor code

`main_programs` includes the program `HeartBeat_interval_sample.ino`, which is the 30-second interval sampling program used for Gabriella's estuary animals in 2021-2022. This version requires SdFat v2.1.2 to compile correctly for Teensy3.5. 

`main_programs` also includes the older program `Gab_HeartBeat.ino` which attempted to sample continuously, but would occasionally drop data points and also used more power. 

`main_programs` file `LED_brightness_load.ino` can be used to write unique IR Led brightness settings to each of the 8 sensor channels used on the Mussel_heartrate_daughterboard hardware from 2020 when using sensors with the MAX30101/MAX30105 sensor (rev B heartrate module). 


Includes Python and Arduino code to attempt to get an Arduino to send data to a computer for real-time plotting of the signal.
This could be used for the gape sensor or the heart rate sensor, potentially. 
