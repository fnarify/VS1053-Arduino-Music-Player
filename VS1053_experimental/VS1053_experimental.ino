/**
 * Arduino project using the VS1503 MP3 player shield hooked up by default with an
 * 16x4 LCD screen and 4x4 keypad.
 * 
 * V1.1:
 *  -swapped pin 10 to pin 0 for the last row of the 4x4 keypad, this in turn fixed the MP3 player locking up
 *   whenever the fourth row was used when the MP3 player was running (pin 10 could not be used as it is used for SS)
 *   However, it does lead to some odd behaviour with the serial input reading, but it shouldn't
 *   cause any issues if you're using one exclusively or even together
 *  -optimised writing to the SD card so that there would be less dropouts due to data lost
 *   (this just involved writing in 2 256-byte chunks instead of 512 1-byte chunks)
 *  -added the option to choose between mp3 or ogg when selecting a track number
 *  -added a time (seconds) display when recording
 * 
 * V1.2:
 *  -Fixed the hangup that would occur when using the playlist after recording anything
 *  -added in the option to input a number to record to as opposed to just incrementing from 1 each time
 *  -recording now starts from 00 instead of 01
 *
 * V1.3:
 *  -Fixed the record function only writing half the required data
 *  
 * EXP ver:
 *  -added the option to use a rotating keypad for input when selecting the playback track. This
 *   lets you input the entire name for a file, the control scheme is detailed under the function createName()
 *  -changed recording so that the track names follow the track000.ogg format instead of the previous
 *   record00.ogg. One this means it goes up to 999, two it's easier to playback directly from the player
 *  -minor changes to some printed strings and data types
 *  
 *  There was a rename function added so that files could be renamed on the SD card, while that worked OK 
 *  on pretty much everything, it prevented recording from working. Even though you could play back the renamed file,
 *  the error was that it could not find the encoding plugin (oggenc.053).
 *  
 *  I'm not too sure the reason for this and thus haven't included it but it isn't hard for someone to implement the
 *  feature using the rename() function that is part of SdFat to see for themselves.
 * 
 * Will take Serial or keypad input to play songs loaded onto an MP3 player,
 * either through a naming scheme "TrackX...X.mp3", direct filename (8.3 format) or
 * via generating a playlist filled with file indices.
 * 
 * There are also other helpful options, like forwarding/reversing through tracks, 
 * changing volume, mono/stereo out, pausing, sinewave/memory tests, etc.
 * 
 * NOTE: if the patch file "patches.053" is not loaded onto the chip during begin()
 * then the Arduino will not be able to play ogg and some other file types.
 * You also need the file "oggenc.053" created from "venc44k2q05.plg" if you want to
 * record to ogg.
 * 
 * As an aside the differential output option does the following:
 * stereo playback -> create a 'virtual' sound
 * mono playback -> create a differential left/right 3V maximum output
 */
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include <SFEMP3Shield.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

// below is not needed if interrupt driven. Safe to remove if not using
#if defined(USE_MP3_REFILL_MEANS) && USE_MP3_REFILL_MEANS == USE_MP3_Timer1
  #include <TimerOne.h>
#elif defined(USE_MP3_REFILL_MEANS) && USE_MP3_REFILL_MEANS == USE_MP3_SimpleTimer
  #include <SimpleTimer.h>
#endif

#define MENU_SIZE  9      // number of menu items to display
#define DEF_OFFSET 2000   // 2s default offset for forwarding/reversing
#define MAX_BUFF   13     // max buffer size for serial input (8.3 filename limit)
#define MAX_INDEX  50     // max amount of file indices to store (subject to memory, 2 bytes ec)
                          // this is not needed if you don't want a way to go backwards but will need to be rewritten
// LCD dimensions, set as needed
#define LCD_ROWS   4
#define LCD_COLS   16
// keypad information
#define KEY_ROWS   4
#define KEY_COLS   4
#define ENTER      '5'
#define UP         '2'
#define DOWN       '8'
#define RIGHT      '6'
#define LEFT       '4'
#define BACK       '*'
// cursor start location for each LCD row
#define LINE1      0x0
#define LINE2      0x10
#define LINE3      0x8
#define LINE4      0x18
// 16 spaces set to needed screen width manually to save memory
#define spaces     F("                ")
// interrupt routine used when recording
#define VS1053_INT_ENABLE 0xC01A

uint8_t    result;                    // globally check return values from various functions
uint8_t    rec_state;                 // state of recording function.
uint8_t    counter = 0;               // small counter used for menu movement
int        playnum = 0;               // current song being played in the playlist
int        playlen = 0;               // length of f_index, incase we find less than MAX_INDEX files
uint16_t   playlist[MAX_INDEX] = {0}; // used to store mp3 file indexes, letting us go backwards and forwards
// These 3 constants cannot be stored in Flash due to the way they are handled by the library
const char keys[KEY_ROWS][KEY_COLS] = {
  {'1' , UP   , '3'  , 'A'},
  {LEFT, ENTER, RIGHT, 'B'},
  {'7' , DOWN , '9'  , 'C'},
  {BACK, '0'  , '#'  , 'D'}
};
// Digital pin 10 CANNOT be used as an input for the keypad, as it is already used for slave select (SS)
// and so must be used for output.
byte rowpins[KEY_ROWS] = {3, 4, 5, 0};
byte colpins[KEY_COLS] = {A3, A2, A1, A0};

// menu strings (restricted to LCD screen width)
char menubuff[LCD_COLS + 1];
PROGMEM const char menu[MENU_SIZE][LCD_COLS + 1] = {
  "0.Playlist",
  "1.Play track num",
  "2.Record",
  "3.Mono/stereo",
  "4.Reset VS1053",
  "5.Sinewave test",
  "6.Differ. output",
  "7.Turn off chip",
  "8.Turn on chip",
};

SdFat             sd;
SdFile            file;
SFEMP3Shield      MP3player;
LiquidCrystal_I2C lcd(0x3F, LCD_COLS, LCD_ROWS); // 0x27 for PCF8574T; 0x3F for PCF8574AT
Keypad keypad = Keypad(makeKeymap(keys), rowpins, colpins, KEY_ROWS, KEY_COLS);

// macros for printing to an LCD panel.
#define printline(n, s) do {lcd.setCursor((n), 0); lcd.print((s));} while(0)
#define clearline(n)    do {lcd.setCursor((n), 0); lcd.print(spaces);} while(0)

/**
 * \brief Setup the Arduino Chip's feature for our use.
 *
 * After Arduino's kernel has booted initialize basic features for this
 * application, such as Serial port and MP3player objects with .begin.
 * Along with displaying the Help Menu.
 *
 * \note returned Error codes are typically passed up from MP3player.
 * Whicn in turns creates and initializes the SdCard objects.
 *
 * \see
 * \ref Error_Codes
 */ 
void setup()
{
  Serial.begin(115200);
  lcd.begin();
  lcd.cursor();
  lcd.backlight();

  // initialise the SdCard.
  // SD_SEL == CS pin value.
  // SPI_HALF_SPEED == SPISettings(F_CPU/4, MSBFIRST, SPI_MODE0);
  if (!sd.begin(SD_SEL, SPI_HALF_SPEED)) {sd.initErrorHalt();}
  if (!sd.chdir("/"))                    {sd.errorHalt("sd.chdir");}

  createPlaylist(); // create the file index playlist used during playback
  
  // initialise the MP3 Player Shield; begin() will attempt to load patches.053  
  result = MP3player.begin();
  if (result != 0)
  {
    Serial.print(F("Error code "));
    Serial.print(result);
    Serial.println(F(" when trying to start player"));
    if (result == 6) {Serial.println(F("NO PATCH FILE, skipping"));}
  }
  // default volume is too loud (higher 8-bit values means lower volume).
  MP3player.setVolume(80, 80);

  displayMenu();
  help();
}

/**
 * Handles either user serial input or input via a matrix keypad. Serial input
 * is primarily kept for debugging.
 *
 * Additionally, if the means of refilling is not interrupt based then the
 * MP3player object is serviced with the availaible function.
 */
void loop()
{
  // 1 for initial command, 12 for extra command (8.3 name) plus '\0'.
  byte buff[MAX_BUFF + 1] = {0};
  byte cur = 0;
  uint8_t insize = 0;

  // parse keypad input, if any, else parse serial input
  cur = keypad.getKey();
  if (cur)
  {
    navMenu(cur);
  }
  else 
  {
    // collect serial input (extra_command) which is our first non-space letter
    // and any other values are appended to the rest of buff.
    while (Serial.available() && insize < MAX_BUFF)
    {
      cur = Serial.read();
      if (!isSpace(cur)) {buff[insize++] = cur;}
    }
    buff[insize] = '\0';
    // get command from serial input only when a valid command is received.
    if (insize) {parseSerial(buff[0], buff + 1, insize);}
  }

  delay(100);
}

/*** KEYPAD INPUT ***/

/**
 * Initial handling of keypad input for navigating the context menu is done here.
 * 
 * The context menu is the String array called menu, and three of the indices at a time
 * are displayed in displayMenu(). Movement is handled by the keys defined UP, DOWN, LEFT and RIGHT,
 * with ENTER acting as a selection key for the currently highlighted option.
 */
void navMenu(byte key)
{ 
  switch (key)
  {
    case ENTER:
      parseMenu(counter);
      break;
      
    // movement functionality
    case UP:
      counter = counter ? counter - 1 : MENU_SIZE - 1;
      break;
      
    case LEFT:
      if   (counter >= LCD_ROWS - 1) {counter -= LCD_ROWS - 1;}
      else                           {counter = MENU_SIZE - (LCD_ROWS - 1 - counter);}
      break;
      
    case RIGHT:
      counter = (counter + LCD_ROWS - 1) % MENU_SIZE;
      break;
      
    case DOWN:
      if (++counter == MENU_SIZE) {counter = 0;}
      break;
      
    default:
      counter = 0;
      break;
  }
  displayMenu();
}

/**
 * Handles input on the default context menu.
 * 
 * Counter is directly related to the index of each menu item in the String array menu.
 */
void parseMenu(uint8_t count)
{
  uint8_t cnt = 0;
  byte key;
  static uint16_t recfn = 0;
  char filename[MAX_BUFF]; // 8.3 filename (12 chars) + '\0'

  lcd.noCursor();
  lcd.clear();
  
  switch(count)
  {
    case 0: // playlist
      play();
      break;
      
    case 1: // play track num
      lcd.print(F("Track num 1-999"));
      printline(LINE2, F("A to confirm"));
      play_track();
      break;
      
    case 2: // record
      lcd.print(F("Enter: 000-999,"));
      printline(LINE2, F("A (record next)"));
      strcpy(filename, "track000.ogg");
      while (key = keypad.waitForKey())
      {
        if (isDigit(key))
        {
          filename[5 + cnt++] = key;
          printline(LINE3, filename);
        }
        else if (key == 'A') // record in order if 'A' is pressed
        {
          filename[5] = (recfn / 100) + '0';
          filename[6] = (recfn / 10) + '0';
          filename[7] = (recfn % 10) + '0';
          recfn++;
          if (recfn == 999) {recfn = 0;}
          break;
        }
        
        if (cnt == 3) {break;} // only add up to three letters
      }

      delay(500);

      lcd.clear();
      lcd.print(F("Recording..."));
      result = record(filename);
      if (!result)
      {
        printline(LINE3, F("Finished"));
        printline(LINE4, filename);
      }
      else
      {
        printline(LINE2, filename);
        printline(LINE3, F("Can't record:"));

        if      (result == 1) {printline(LINE4, F("no plugin"));}
        else if (result == 2) {printline(LINE4, F("can't make file"));}
        else                  {printline(LINE4, F("wrong filename"));}
      }

      delay(2000);

      createPlaylist();            // recreate playlist incase data is written to the SD card
      MP3player.vs_init();         // restart MP3 player after recording to prevent lockups during playback
      MP3player.setVolume(80, 80); // volume does need to be reset.

      break;
      
    case 3: // change to mono/stereo
      lcd.print(F("Mono Mode:"));
      if (MP3player.getMonoMode())
      {
        MP3player.setMonoMode(0);
        printline(LINE2, F("Disabled"));
      }
      else
      {
        MP3player.setMonoMode(1);
        printline(LINE2, F("Enabled"));
      }
      delay(2000);
      break;
      
    case 4: // reset VS1053
      lcd.print(F("Reset VS1503"));
      delay(2000);
      MP3player.stopTrack();
      MP3player.vs_init();
      break;
      
    case 5: // sinewave test
      lcd.print(F("Sine test:"));
      test('t'); // enable
      keypad.waitForKey(); // blocking input
      test('t'); // disable
      delay(2000);
      break;
  
    case 6: // differential output
      lcd.print(F("Differ. Output:"));
      lcd.setCursor(LINE2, 0);
      if (MP3player.getDifferentialOutput())
      {
        MP3player.setDifferentialOutput(0);
        lcd.print(F("Disabled"));
      }
      else
      {
        MP3player.setDifferentialOutput(1);
        lcd.print(F("Enabled"));
      }
      delay(2000);
      break;
      
    case 7: // turn chip off
      lcd.print(F("VS1053b off"));
      delay(2000);
      MP3player.end();
      break;
      
    case 8: // turn chip on
      lcd.print(F("VS1053b on"));
      delay(2000);
      MP3player.begin();
      break;

    default: // don't do anything for the rest
      break;
  }

  lcd.cursor(); // cursor back on
}

/*** PLAY FUNCTIONS ***/

/**
 * Handles playing of file indices (16-bits) in the array playlist, and also
 * handles user input related to playlist functionality.
 * Including going forwards/backwards, volume, etc.
 * 
 * Generic function name may cause issues with other libraries.
 */
void play()
{
  char filename[MAX_BUFF];
  bool skipped = true;   // if a song has been skipped.

  MP3player.setVUmeter(1);
  
  // reset playlist index before starting
  playnum = 0;
  while (playnum < playlen)
  {
    lcd.clear();
    
    // increment to the next file index if and only if
    // the song finished without skipping (LEFT/RIGHT pressed)
    if (!skipped) {playnum++;}
    skipped = false;
    
    if (file.open(sd.vwd(), playlist[playnum], O_READ))
    {
      if (!file.getFilename(filename))
      {
        file.close();
      }
      else
      {
        file.close();
        result = MP3player.playMP3(filename, 0);
        if (result)
        {
          printline(LINE1, F("Cannot play song"));
          printline(LINE2, F("trying next song"));
          delay(2000);
          Serial.print(result);
          playnum++;
        }
        else // display file data
        {
          lcd.print(F("Playing track:"));
          printline(LINE2, filename);
          skipped = play_commands();
        }
      }
    }
  }
  
  MP3player.setVUmeter(0);
}

/** 
 *  MP3 is playing and command received.
 *  
 *  Sometimes corrupted SD cards, poor wiring on the keypad and possibly other problems,
 *  can cause the switch statement to behave oddly. Such as a break causing the inner while() loop
 *  to be exited.
 *  
 *  Commands:
 *  LEFT -> skip to previous song
 *  RIGHT -> skip to next song
 *  ENTER -> pause/unpause song
 *  'A' -> restart song 1s from start
 *  '1', '3' -> decrease/increase playspeed
 *  '7', '9' -> decrease/increase volume
 *  BACK -> exit playback
 */
bool play_commands()
{
  char command;
  uint8_t playspeed;
  union twobyte mp3_vol; // helps deal with endian issues and individual byte access  
  bool retval = false;
  uint32_t tstart = millis(), toff = 0;
  
  while (MP3player.isPlaying())
  {
    print_info(tstart, toff);
    
    // handle input
    command = keypad.getKey();
    if (command)
    {
      switch (command)
      {
        case 'A': // restart song
          printline(LINE3, F("Restarting"));
          MP3player.stopTrack();
          retval = true;
          break;
          
        case LEFT: // skip to previous song
          playnum ? playnum-- : playnum = playlen - 1;
          retval = true;
          MP3player.stopTrack();
          break;
          
        case RIGHT: // skip to next song
          playnum++;
          retval = true;
          MP3player.stopTrack();
          break;
          
        case ENTER: // pause
          clearline(LINE4);
          if (MP3player.getState() == playback)
          {
            toff += millis();
            printline(LINE4, F("Paused"));
            MP3player.pauseMusic();
          }
          else if (MP3player.getState() == paused_playback)
          {
            printline(LINE4, F("Resuming"));
            toff = millis() - toff;
            MP3player.resumeMusic();
          }
          break;
          
        case '1': case '3': // decrease/increase playspeed
          playspeed = MP3player.getPlaySpeed();
          if (command == '3') // increase
          {
            if (playspeed >= 254) {playspeed = 5;}
            else                  {playspeed++;}
          }
          else if (playspeed) // decrease
          {
            playspeed--;
          }
          MP3player.setPlaySpeed(playspeed);
          break;
  
        case '7': case '9': // decrease/increase volume
          mp3_vol.word = MP3player.getVolume(); // returns a double uint8_t of Left and Right packed into int16_t
  
          if (command == '7') // note dB is negative (higher byte values == lower volume)
          {
            // assume equal balance and use byte[1] for math
            if (mp3_vol.byte[1] >= 254) {mp3_vol.byte[1] = 254;}
            else                        {mp3_vol.byte[1] += 2;} // keep it to whole dB's
          }
          else
          {
            if (mp3_vol.byte[1] <= 2) {mp3_vol.byte[1] = 2;} // range check
            else                      {mp3_vol.byte[1] -= 2;}
          }
          // push byte[1] into both left and right assuming equal balance.
          MP3player.setVolume(mp3_vol.byte[1], mp3_vol.byte[1]); // commit new volume
          break;
          
        case BACK: default: // exit playback
          MP3player.stopTrack();
          playnum = playlen; // end outer loop
          counter = 0;
          break;
      }
    }
  }

  return retval;
}

/**
 * Prints time and VU info during playback.
 * VU meter should be initialised and disabled within the parent function.
 */
void print_info(uint32_t start, uint32_t offset)
{
  char linebuff[LCD_COLS + 1];
  union twobyte vu; // for VUmeter functionality.

  // don't display data if paused
  if (MP3player.getState() == playback)
  {
    // display time playing
    if (snprintf(linebuff, LCD_COLS + 1, "Time: %lus",(millis() - start - offset) / 1000))
    {
      printline(LINE3, linebuff);
    }
    
    // display VU meter
    vu.word = MP3player.getVUlevel();
    if (snprintf(linebuff, LCD_COLS + 1, "VU: L=%u R=%u", vu.byte[1], vu.byte[0]) && (millis() % 1000UL > 900UL))
    {
      printline(LINE4, linebuff);
    }
  }
}

/**
 * Creates a formatted string ("trackXXX.ext", where ext is a given file extension and
 * XXX is an integer from 000 - 999.
 * 
 * A track will keep playing until eiter it stops itself or the user prompts it to stop
 * by pressing any key.
 * 
 * Note that it can take time for the buffer to be flushed once a file has stopped playing.
 * This means that opening the same file again may not work if you attempt to open it again straight away.
 * All you can do is wait a few seconds before opening it again.
 */
void play_track()
{
  char trackname[MAX_BUFF];
  byte key;
  uint8_t cnt = 0;
  uint16_t tracknum = 0;

  printline(LINE3, F("name(L)/num(R)?"));
  key = keypad.waitForKey();
  clearline(LINE3);
  if (key == LEFT)
  {
    createName(trackname);
  }
  else
  {
    // collect input (blocking).
    while (key = keypad.waitForKey())
    {
      // only collect up to 3 characters.
      if (isDigit(key))
      {
        tracknum = tracknum * 10 + key - '0';
        cnt++;
        printline(LINE3, tracknum);
      }
      
      // track number confirmed
      if (key == 'A' || cnt == 3) {break;}
    }

    if (!tracknum) {tracknum = 1;}
    snprintf(trackname, MAX_BUFF, "track%03u.mp3", tracknum);
    
    // play .ogg if RIGHT key pressed, otherwise default to play as .mp3
    printline(LINE4, F("MP3(L)/OGG(R)?"));
    key = keypad.waitForKey();
    if (key == RIGHT) {strcpy(trackname + 9, "ogg");}
  }

  result = MP3player.playMP3(trackname, 0);
  if (result)
  {
    // debug info
    printline(LINE3, trackname);
    if (result == 2)      {printline(LINE4, F("track not found "));}
    else if (result == 3) {printline(LINE4, F("player in reset "));}
    Serial.print(F("Error "));
    Serial.println(result);
  }
  else
  {
    lcd.clear();
    printline(LINE1, F("Playing"));
    printline(LINE2, trackname);

    // play until any user input, or the song finishes
    while (MP3player.isPlaying())
    {
      if (keypad.getKey()) {MP3player.stopTrack();}
    }
    printline(LINE3, F("Finished playing"));
  }
    
  delay(2000);
}

/**
 * Uses a rotating keyboard layout to build the at most 12 character long
 * filename needed for playing back a track. The keypad layout is identical
 * to what was typical for mobile phones.
 * 
 * At most the trackname can only be 12 characters long (MAX_BUFF - 1)
 * due to a limitation in the SdFat library to 8.3 filenames.
 *
 * CONTROLS:
 * 1 -> moves to the next character to write
 * 2 -> abc
 * 3 -> def
 * 4 -> ghi
 * 5 -> jkl
 * 6 -> mno
 * 7 -> pqrs
 * 8 -> tuv
 * 9 -> wxyz
 * # -> enters 0 initially and then increments that number by 1
 * * -> deletes the rightmost character
 * 0 -> appends a '.' and finishes writing the filename
 */
void createName(char *trackname)
{
  byte key, prevkey = '\0';
  uint8_t cnt = 0, limit = 2;
  int index = -1;

  memset(trackname, 0, MAX_BUFF);

  while (key = keypad.waitForKey())
  {
    if (key == 'A') // upper/lowercase
    {
      if (trackname[index] >= 'A' && trackname[index] <= 'Z')      {trackname[index] += 32;}
      else if (trackname[index] >= 'a' && trackname[index] <= 'z') {trackname[index] -= 32;}
    }
    else if (key == '1') // enter character
    {
      if (index != 7) {prevkey = '\0';}
      cnt = 0;
    }
    else if (key > '1' && key <= '9') // rotating keypad
    {
      if (key != prevkey)
      {
        if (index < 7) {index++;}
        trackname[index] = 'a' + 3 * (key - '0' - 2);
        // off by one for 8 and 9
        if (key > '7') {trackname[index]++;}
        // pqrs and wxyz for digits 7 and 9
        limit = (key == '7' || key == '9') ? 3 : 2;        
        prevkey = key;
        cnt = 1;
      }
      else
      {
        if (cnt > limit) {trackname[index] -= limit; cnt = 1;}
        else             {trackname[index]++; cnt++;}
      }
    }
    else if (key == '#') // start from 0 and increment each time
    {
      if (cnt >= 0 && index < 7)
      {
        if (index < 7) {index++;}
        trackname[index] = '0';
        cnt = -1;
      }
      else
      {
        trackname[index]++;
        if (trackname[index] > '9') {trackname[index] = '0';}
      }
    }
    else if (key == '*' && index > -1) // delete character
    {
      trackname[index--] = ' ';
      prevkey = '\0';
    }
    else if (key == '0' && index > -1) // add '.' for file ext
    {
      index++;
      break;
    }

    printline(LINE3, trackname);
  }
  
  printline(LINE4, F("MP3(L)/OGG(R)?"));
  key = keypad.waitForKey();
  if (key == RIGHT) {strcpy(trackname + index, ".ogg");}
  else              {strcpy(trackname + index, ".mp3");}
}

/**
 * Builds the playlist used in the play() function, and should be called whenever
 * the data on the SD card changes or during program start.
 */
void createPlaylist()
{
  // store the unique index for every mp3 file in the SD card.
  char filename[MAX_BUFF];
  // make sure length and array are zeroed before starting
  playlen = 0;
  memset(playlist, 0, sizeof(playlist));
  sd.vwd()->rewind();
  while (playlen < MAX_INDEX && file.openNext(sd.vwd(), O_READ))
  {
    if (file.getFilename(filename)) // check for mp3/aac/wma/wav/fla/mid/ogg substring.
    {
      if (isFnMusic(filename)) // store 16-bit unsigned index.
      {
        playlist[playlen++] = (sd.vwd()->curPosition() / 32) - 1;
      }
    }
    file.close();
  }
}

/*** SERIAL INPUT ***/

/**
 * Parses through the characters of the users input, executing corresponding
 * MP3player library functions and features then displaying a brief menu and
 * prompting for next input command.
 *
 * The extra_command is a char array up to MAX_BUFF - 2 chars in size that 
 * can be used to define filenames or values.
 */
void parseSerial(byte key_command, byte *extra_command, int len)
{
  uint8_t playspeed;
  uint16_t trackno;      // track number to open, format trackX...X.mp3
  int32_t offset_ms;     // specifying an offset value for the track
  union twobyte vu;      // for VUmeter functionality.
  union twobyte mp3_vol; // helps deal with endian issues and individual byte access
  
  Serial.print(F("Received command: "));
  if (extra_command) {Serial.write(extra_command - 1, len);}
  else               {Serial.write(key_command);}
  Serial.println();

  switch(key_command)
  {
      case 's': // stop curent track.
        Serial.println(F("Stopping"));
        MP3player.stopTrack();
        break;
        
      case '^': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9': // play corresponding track.
        /** Stick to 8.3 filenames due to SdFat limitation of 13 character long filenames. */
        // convert extra_command or key command to a number (explicit conversion to char *).
        trackno = key_command == '^' ? atoi((char*) extra_command) : key_command - 48;

#if USE_MULTIPLE_CARDS
        sd.chvol(); // assign desired sdcard's volume.
#endif

        // tell the MP3 Shield to play a track
        result = MP3player.playTrack(trackno);

        // check result, see readme for error codes.
        if (result)
        {
          Serial.print(F("Error "));
          Serial.println(result);
        }
        else // print mp3 info if possible.
        {
          printTrackInfo();
        }
        break;

      case '*': // "* file.ogg" records to the given filename
        Serial.println(record((char*) extra_command));
        MP3player.vs_init(); // restart hardware
        MP3player.setVolume(80, 80);
        break;
        
      case '-': case '+': // change volume.
        mp3_vol.word = MP3player.getVolume(); // returns a double uint8_t of Left and Right packed into int16_t

        if (key_command == '-') // note dB is negative
        {
          // assume equal balance and use byte[1] for math
          if (mp3_vol.byte[1] >= 254){mp3_vol.byte[1] = 254;} // range check
          else                       {mp3_vol.byte[1] += 2;}  // keep it simpler with whole dB's
        }
        else
        {
          if (mp3_vol.byte[1] <= 2) {mp3_vol.byte[1] = 2;} // range check
          else                      {mp3_vol.byte[1] -= 2;}
        }
        // push byte[1] into both left and right assuming equal balance.
        MP3player.setVolume(mp3_vol.byte[1], mp3_vol.byte[1]); // commit new volume
        Serial.print(F("Volume changed to -"));
        Serial.print(mp3_vol.byte[1] >> 1, 1);
        Serial.println(F("[dB]"));
        break;
        
      case '<': case '>': // change play speed.
        playspeed = MP3player.getPlaySpeed(); // create key_command existing variable
        // note playspeed of Zero is equal to ONE, normal speed.
        if (key_command == '>')
        {
          if (playspeed >= 254) {playspeed = 5;} // range check
          else                  {playspeed += 1;}
        }
        else if (playspeed)
        {
          playspeed--;
        }
        MP3player.setPlaySpeed(playspeed);
        Serial.print(F("playspeed to "));
        Serial.println(playspeed, DEC);
        break;
        
      case 'f': case 'F': // play track with f filename.mp3 (space is optional).
        // Internal limit for filename size is MAX_BUFF - 2, but can be increased.
        offset_ms = key_command == 'F' ? DEF_OFFSET : 0;

        // tell the MP3 Shield to play that file
        // check result, see readme for error codes.
        result = MP3player.playMP3((char*) extra_command, offset_ms);
        if (result)
        {
          Serial.print(F("Error "));
          Serial.println(result);
        }
        else
        {
          printTrackInfo();
        }
        break;
        
      case 'd': // displays files in current directory on sd card.
        if (!MP3player.isPlaying())
        {
          // prevent root.ls when playing, something locks the dump. but keeps playing.
          Serial.println(F("Files (name date time size):"));
          sd.ls(LS_R | LS_DATE | LS_SIZE);
        }
        else
        {
          Serial.println(F("Playing files, try again later"));
        }
        break;
        
      case 'i': // get and display audio info.
        MP3player.getAudioInfo();
        break;
        
      case 'p': // pauses/resumes current playback.
        if (MP3player.getState() == playback)
        {
          MP3player.pauseMusic();
          Serial.println(F("Pausing"));
        }
        else if (MP3player.getState() == paused_playback)
        {
          MP3player.resumeMusic();
          Serial.println(F("Resuming"));
        }
        else
        {
          Serial.println(F("Not Playing!"));
        }
        break;
        
      case 'r': // restarts current song.
        // restarts 2000ms after the beginning and only works when paused.
        MP3player.pauseMusic();
        MP3player.resumeMusic(2000);
        break;
        
      case 'R': // restarts VS10xx chip.
        MP3player.stopTrack();
        MP3player.vs_init();
        Serial.println(F("Reset VS10xx chip"));
        break;
        
      case 't': case 'm': // tests sinewave (t) or memory (m).
        test(key_command);
        break;
        
      case 'e': // spatial ear speaker settings.
        result = MP3player.getEarSpeaker();
        result = result >= 3 ? 0 : result + 1;
        MP3player.setEarSpeaker(result); // commit new earspeaker
        Serial.print(F("earspeaker to "));
        Serial.println(result, DEC);
        break;
        
      case 'M': // enables/disables mono output
        Serial.print(F("Mono Mode "));
        if (MP3player.getMonoMode())
        {
          MP3player.setMonoMode(0);
          Serial.println(F("disabled"));
        }
        else
        {
          MP3player.setMonoMode(1);
          Serial.println(F("enabled"));
        }
        break;
        
      case 'g': case 'k': // jump to a given time (g) or rewind/forward (k)
        offset_ms = atoi((char*) extra_command); // explicit conversion to char *
        offsetTrack(offset_ms, key_command);
        break;
        
      case 'o': // enables the VS10xx chip.
        MP3player.begin();
        Serial.println(F("VS10xx restored from low power reset mode"));
        break;
        
      case 'O': // disables the VS10xx chip.
        MP3player.end();
        Serial.println(F("VS10xx placed into low power reset mode"));
        break;
        
      case 'D': // enables/disables differential output.
        Serial.print(F("Differential Mode "));
        if (MP3player.getDifferentialOutput())
        {
          MP3player.setDifferentialOutput(0);
          Serial.println(F("disabled"));
        }
        else
        {
          MP3player.setDifferentialOutput(1);
          Serial.println(F("enabled"));
        }
        break;
        
      case 'S': // get current state of the VS10xx chip.
        displayState();
        break;
        
      case 'V': // enables a VUmeter (if supported).
        MP3player.setVUmeter(1);
        Serial.print(F("VU meter = "));
        Serial.println(MP3player.getVUmeter());
        Serial.println(F("Hit any key to stop"));

        while (!Serial.available())
        {
          vu.word = MP3player.getVUlevel();
          Serial.print(F("VU: L = "));
          Serial.print(vu.byte[1]);
          Serial.print(F(" / R = "));
          Serial.print(vu.byte[0]);
          Serial.println(" dB");
          delay(1000);
        }
        Serial.read();

        MP3player.setVUmeter(0);
        break;
        
      case 'h': // display help.
        help();
        break;
        
      default:
        Serial.println(F("not recognised"));
        break;
  }

  // print prompt after key stroke has been processed.
  Serial.println(F("Enter *,^,1-9,f,F,s,d,+,-,i,>,<,p,r,R,t,m,M,g,k,h,O,o,D,S,V :"));
}

/*** RECORDING ***/

/**
 * Implements recording for the VS1053 according to the VorbisEncoder170c.pdf manual
 * The recorded files are saved in OGG format on an SD card.
 * Note that it primarily records using the on-board microphone, but line-input
 * can be enabled by writing SM_LINE1 to SCI_MODE as in the manual, or defining
 * a macro called USE_LINEIN
 */
uint8_t record(char *filename)
{
  uint8_t wordsToWrite;
  uint16_t data, wordsToRead, wordsWaiting;
  uint32_t timer;
  byte wbuff[256]; // we write up to 256 bytes at a time twice

  // make sure state begins at 0
  rec_state = 0;

  /* 1. set VS1053 clock to 4.5x = 55.3 MHz; our board uses a 12.288MHz crystal */
  MP3player.Mp3WriteRegister(SCI_CLOCKF, 0xC000);
  timer = millis();
  while (!digitalRead(MP3_DREQ) || millis() - timer > 100UL) {;}
  
  /* 2. clear SCI_BASS */
  MP3player.Mp3WriteRegister(SCI_BASS, 0);

  /* 3. reset VS1053 */
  MP3player.Mp3WriteRegister(SCI_MODE, (MP3player.Mp3ReadRegister(SCI_MODE) | SM_RESET));
  // Wait until DREQ is high or 100ms
  timer = millis();
  while (!digitalRead(MP3_DREQ) || millis() - timer > 100UL) {;}

  /* 4. disable all interrupts except SCI */
  MP3player.Mp3WriteRegister(SCI_WRAMADDR, VS1053_INT_ENABLE);
  MP3player.Mp3WriteRegister(SCI_WRAM, 0x2);

  /* 5. need to load plugin for recording (exported as oggenc.053, 
        original filename is venc44k2q05.plg but it's too large for an SD card) */
  if (MP3player.VSLoadUserCode((char*) "oggenc.053"))
  {
    Serial.println(F("Can't load plugin"));
    return 1;
  }

  // load file for recording
  if (filename)
  {
    if (!file.open(filename, O_RDWR | O_CREAT))
    {
      Serial.println(F("Can't open file"));
      return 2;
    }
  }
  else
  {
    Serial.println(F("No filename given"));
    return 3;
  }

  /* 6. Set VS1053 mode bits as needed. If USE_LINEIN is set
        then the line input will be used over the on-board microphone */
  #ifdef USE_LINEIN
    MP3player.Mp3WriteRegister(SCI_MODE, SM_LINE1 | SM_ADPCM | SM_SDINEW);
  #else
    MP3player.Mp3WriteRegister(SCI_MODE, SM_ADPCM | SM_SDINEW);
  #endif

  /* 7. Set recording levels on control registers SCI_AICTRL1/2 */
  // Rec level: 1024 = 1. If 0, use AGC.
  MP3player.Mp3WriteRegister(SCI_AICTRL1, 1024);
  // Maximum AGC level: 1024 = 1. Only used if SCI_AICTRL1 is set to 0.
  MP3player.Mp3WriteRegister(SCI_AICTRL2, 0);
  
  /* 8. no VU meter to set */
   
  /* 9. set a value to SCI_AICTRL3, in this case 0 */
  MP3player.Mp3WriteRegister(SCI_AICTRL3, 0);

  /* 10. no profile to set for VOX */

  /* 11. Active encoder by writing 0x34 to SCI_AIADDR for Ogg Vorbis */
  MP3player.Mp3WriteRegister(SCI_AIADDR, 0x34);

  /* 12. wait until DREQ pin is high before reading data */
  timer = millis();
  while (!digitalRead(MP3_DREQ) || millis() - timer > 100UL) {;}

  /**
   * Handles recording data:
   * state == 0 -> normal recording
   * state == 1 -> user and micro requested end of recording
   * state == 2 -> stopped recording, but data still being collected
   * state == 3 -> recoding finished
   */
  Serial.println(F("Recording started..."));
  timer = millis(); // start timing.
  while (rec_state < 3)
  {
    printline(LINE2, (millis() - timer) / 1000);
    
    // detect if key is pressed and stop recording
    if (keypad.getKey() && !rec_state)
    {
      rec_state = 1;
      MP3player.Mp3WriteRegister(SCI_AICTRL3, 1);
    }
    
    // check for serial request to stop recording
    if (Serial.available())
    {
      if (Serial.read() == 's' && !rec_state)
      {
        rec_state = 1;
        MP3player.Mp3WriteRegister(SCI_AICTRL3, 1);
      }
    }

    // see how many 16-bit words there are waiting in the VS1053 buffer
    wordsWaiting = MP3player.Mp3ReadRegister(SCI_HDAT1);

    // if user has requested and VS1053 has stopped recording increment state to 2
    if (rec_state == 1 && MP3player.Mp3ReadRegister(SCI_AICTRL3) & (1 << 1))
    {
      rec_state = 2;
      // reread the HDAT1 register to make sure there are no extra words left.
      wordsWaiting = MP3player.Mp3ReadRegister(SCI_HDAT1);
    }

    // read and write 512-byte blocks. Except for when recording ends, then write a smaller block.
    while (wordsWaiting >= ((rec_state < 2) ? 256 : 1))
    {
      wordsToRead = min(wordsWaiting, 256);
      wordsWaiting -= wordsToRead;

      // if this is the last block, read one 16-bit value less as it is handled separately
      if (rec_state == 2 && !wordsWaiting) {wordsToRead--;}

      wordsToWrite = wordsToRead / 2;

      // transfer the 512-byte block in two groups of 256-bytes due to memory limitations,
      // except if it's the last block to transfer, then transfer all data except for the last 16-bits
      for (uint8_t i = 0; i < 2; i++)
      {
        for (uint8_t j = 0; j < wordsToWrite; j++)
        {
          data = MP3player.Mp3ReadRegister(SCI_HDAT0);
          wbuff[2 * j] = data >> 8;
          wbuff[2 * j + 1] = data & 0xFF;
        }
        
        if (wordsToWrite)
        {
          file.write(wbuff, 2 * wordsToWrite);
        }
      }
            
      // if last data block
      if (wordsToRead < 256)
      {
        rec_state = 3;

        // read the very last word of the file
        data = MP3player.Mp3ReadRegister(SCI_HDAT0);

        // always write first half of the last word
        file.write(data >> 8);

        // read SCI_AICTRL3 twice, then check bit 2 of the latter read
        MP3player.Mp3ReadRegister(SCI_AICTRL3);
        if (!(MP3player.Mp3ReadRegister(SCI_AICTRL3) & (1 << 2)))
        {
          // write last half of the last word only if bit 2 is clear
          file.write(data & 0xFF);
        }
      }
    }
  }

  // done, now close file
  file.close();

  MP3player.Mp3WriteRegister(SCI_MODE, (MP3player.Mp3ReadRegister(SCI_MODE) | SM_RESET));

  Serial.println(F("Done"));
  return 0;
}

/*** MISC ***/

/**
 * helper function for offset jumping and rewinding/forwarding.
 * offset_ms is a String converted to a 32-bit long.
 * mode == 'g' -> skip to a point
 * mode == 'k' -> forward/reverse
 */
void offsetTrack(int32_t offset_ms, byte mode)
{
  if (!offset_ms)
  {
    offset_ms = mode == 'k' ? -DEF_OFFSET : DEF_OFFSET;
    Serial.print(F("default "));
  }
  
  Serial.print(F("jump to "));
  Serial.print(offset_ms, DEC);
  Serial.println(F("[ms]"));
  result = mode == 'g' ? MP3player.skipTo(offset_ms) :
           mode == 'k' ? MP3player.skip(offset_ms)   :
           0                                         ;

  if (result)
  {
    Serial.print(F("Error "));
    Serial.print(result);
    Serial.println(F(" trying to skip track"));
  }
}

/**
 * Display mp3 track info only.
 * mp3 ID3 info is up to 30 bytes long, and getTrackInfo adds a null-byte
 * to the end of the string so use a 31 byte long array.
 */
void printTrackInfo()
{
  char metadata[31] = {0};
  Serial.print(F("P:"));

  // extract and print up to LCD_COLS bytes of info each.
  MP3player.getTrackInfo(TRACK_TITLE, metadata, 30);
  Serial.println(metadata);
  MP3player.getTrackInfo(TRACK_ARTIST, metadata, 30);
  Serial.println(metadata);
  MP3player.getTrackInfo(TRACK_ALBUM, metadata, 30);
  Serial.println(metadata);
}

/**
 * Displays current state of the chip.
 */
void displayState()
{
  Serial.println(F("Current State of VS10xx is:"));
  Serial.print(F("isPlaying() = "));
  Serial.println(MP3player.isPlaying());

  Serial.print(F("getState() = "));
  switch (MP3player.getState()) {
  case uninitialized:
    Serial.print(F("uninitialised"));
    break;
  case initialized:
    Serial.print(F("initialised"));
    break;
  case deactivated:
    Serial.print(F("deactivated"));
    break;
  case loading:
    Serial.print(F("loading"));
    break;
  case ready:
    Serial.print(F("ready"));
    break;
  case playback:
    Serial.print(F("playback"));
    break;
  case paused_playback:
    Serial.print(F("paused_playback"));
    break;
  case testing_memory:
    Serial.print(F("memory_test"));
    break;
  case testing_sinewave:
    Serial.print(F("sine_test"));
    break;
  }
  Serial.println();
}

/**
 * Tests both a sinewave and the chip's memory depending on the mode passed.
 * mode == 't' -> sinewave test
 * mode == 'm' -> memory test
 */
void test(byte mode)
{
  int8_t state;
  state = mode == 't' ? MP3player.enableTestSineWave(126) :
          mode == 'm' ? MP3player.memoryTest()            :
          0                                               ;

  if (state == -1)
  {
    Serial.println(F("Unavailable while playing music or chip in reset"));
  }
  else if (state == 1 && mode == 't')
  {
    Serial.println(F("Enabling sinewave"));
    printline(LINE2, F("Enabled")); // Remove if not using an LCD screen.
  }
  else if (state == 2)
  {
    MP3player.disableTestSineWave();
    Serial.println(F("Disabling sinewave"));
    printline(LINE2, F("Disabled")); // Remove if not using an LCD screen.
  }
  else if (mode == 'm')
  {
    Serial.print(F("Memory test results = "));
    Serial.println(state, HEX);
    Serial.println(F("Should be 0x83FF"));
    Serial.println(F("A reset is needed to recover to normal"));
  }
}

/**
 * Prints a full menu of the commands available along with descriptions.
 */
void help()
{
  Serial.println(F("COMMANDS:"));
  Serial.println(F(" [^ number] play track number"));
  Serial.println(F(" [1-9] play trackX.mp3"));
  Serial.println(F(" [* file.ogg] ogg vorbis recording to file"));
  Serial.println(F(" [f file.mp3] play given filename"));
  Serial.println(F(" [F file.mp3] same as [f] but with initial skip of 2s"));
  Serial.println(F(" [s] stop playing/recording"));
  Serial.println(F(" [d] display directory of SD card"));
  Serial.println(F(" [+ or -] change volume"));
  Serial.println(F(" [> or <] increment or decrement play speed by 1 factor"));
  Serial.println(F(" [i] retrieve current audio information"));
  Serial.println(F(" [e] increment Spatial EarSpeaker, default is 0, wraps after 4"));
  Serial.println(F(" [p] pause"));
  Serial.println(F(" [r] resumes 2s from start of file"));
  Serial.println(F(" [R] Resets and initialise VS10xx chip"));
  Serial.println(F(" [t] toggle sinewave test"));
  Serial.println(F(" [m] memory test, a reset is needed after to recover"));
  Serial.println(F(" [M] toggle between Mono and Stereo Output"));
  Serial.println(F(" [g offset] skip to a given offset (ms) in current track"));
  Serial.println(F(" [k offset] skip a given number of ms in current track"));
  Serial.println(F(" [O} puts the VS10xx into low power reset"));
  Serial.println(F(" [o} takes the VS10xx out of low power reset"));
  Serial.println(F(" [D] toggle SM_DIFF between inphase and differential output"));
  Serial.println(F(" [S] show device state"));
  Serial.println(F(" [V] enable VU meter"));
  Serial.println(F(" [h] show help"));
}

/**
 * Displays the char array menu on the LCD screen and handles scrolling.
 */
void displayMenu()
{
  lcd.clear();
  lcd.print(F("MENU"));
  // using strcyp_P to let us the menu items in flash
  strcpy_P(menubuff, menu[counter]);
  printline(LINE2, menubuff);
  strcpy_P(menubuff, menu[(counter + 1) % MENU_SIZE]);
  printline(LINE3, menubuff);
  strcpy_P(menubuff, menu[(counter + 2) % MENU_SIZE]);
  printline(LINE4, menubuff);

  lcd.setCursor(LINE2, 0);
}

