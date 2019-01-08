# VS1053-based Music Player for Arduino

* Plays MP1-MP3, Ogg Vorbis, AAC, WAV, WMA, MIDI (potentially FLAC)
* Up to 48kHz 320kb/s MP3 playback (with restrictions, check the datasheet page 26 onwards)
* Records to Ogg Vorbis (multiple profiles, 44.1kHz 87kb/s used)
* Stereo Audio via a 3.5mm jack, up to a 30 ohm load
* External microphone (3.5mm jack) can be used, or internal electret mic
* Bass/Treble controls possible, but not implemented except via serial control
* Has a playlist for playback (limited by SRAM, 2 bytes per file) which is needed so that you can easily go back multiple tracks
* Controlled via a 4x4 matrix keypad or serial input
* 16x4 LCD screen connected via an I2C adaptor
* 8.3 filename restriction (although larger filenames can be played, with varying results)
And more

**Pin Connections**
```
From -> To
LCD SDA -> A4
LCD SCL -> A5
LCD GND -> GND
LCD VCC -> 5V
Keypad, pin 1 of the keypad is the far left column, pin 5 the top row. Ergo, character 1 brings pin 1 and 5 high, best to measure with a multimeter to determine what pin is what.
1 -> A3
2 -> A2
3 -> A1
4 -> A0
5 -> D3
6 -> D4
7 -> D5
8 -> D0 (also serial output, no spare pins, sometimes outputs garbage to the serial monitor, but doesn't affect the program operation)
```

This library is for use with a VS1053-based (http://www.vlsi.fi/en/products/vs1053.html) Arduino shield with an Arduino Uno in particular, but will work on similar boards.
This library was tested using a Geeetech Arduino MP3 shield board (http://www.geeetech.com/wiki/index.php/Arduino_MP3_shield_board_with_TF_card), which is a pseudo-clone board. The library will likely work with the original SparkFun version as well (https://www.sparkfun.com/products/12660) although I do not have one to test with. Originally published in Silicon Chip magazine, July 2017; wiring diagram and more details provided in that article.

It needs the third-party libraries:
1. SdFat (https://github.com/greiman/SdFat)
2. SFEMPShield (https://github.com/madsci1016/Sparkfun-MP3-Player-Shield-Arduino-Library)
3. LiquidCrystal_I2C (https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library)
4. Keypad https://github.com/Chris--A/Keypad

The LiquidCrystal library is especially annoying as there are multiple different versions around that are similar but use different function headers.
I've included most of the libraries as zips incase they differ greatly from the versions found online, or can't be found.

Patch files for the VS1053 can be found here: http://www.vlsi.fi/en/support/software/vs10xxpatches.html
I've included most of the patch files required and already converted. You will need patches.053 (required for ogg etc playback) and oggenc.053 (for recording to ogg) located in the root directory of the SD card.

By default it prints to a 16x4 LCD screen and the printing assumes as much. That will need to be manually altered if you want to use a smaller screen. The LCD screen I used also had a non-standard way to print per line (moving the cursor), which is why I defined the starting location of each row manually (LINE1-4).

The only difference between the regular and experimental version is the experimental version has a rotating keypay function built in, so you can input the full track name to record to or play off. No real reason not to use it apart from saving a little extra space, the majority of which is taken up by the libraries.
