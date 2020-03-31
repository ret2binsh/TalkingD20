/* D20 Die - Program to handle the Talking D20 Die project
 *  -Program controls an ADXL345 for accelerometer readings in order to
 *    calculate which face is up after a given roll after ensuring the die
 *    has stabilized. Communication is with I2C and leverages a hardware
 *    interrupt on Pin 3 (Interrupt Pin 0)
 *    
 *  -Program controls a DFPlayer Mini via Software Serial in order to 
 *    play the applicable sound bytes aka, Roll Announcement, Face Value,
 *    and Banter.
 *    
 *  - Credit to Phillip Burgess' Adafruit post for project idea.
 *    https://learn.adafruit.com/talking-d20-20-sided-gaming-die/overview
 *    
 *  - Joshua B Gonzales March 2020
 */
 
#include <avr/power.h>
#include <avr/sleep.h>
#include <limits.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#include <DFMiniMp3.h>
#include <SparkFun_ADXL345.h>

/* ====================================== DFPlayer Mini Notification Class ========================= */

bool trackComplete = false;

// implement a notification class for the dfplayer mini
class Mp3Notify
{
public:
  static void OnError(uint16_t errorCode)
  {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    Serial.println(errorCode);
  }
  static void OnPlayFinished(uint16_t track)
  {
    Serial.print("Play finished for #");
    Serial.println(track);
    trackComplete = true;
  }
  static void OnCardOnline(uint16_t code)
  {
    Serial.println("Card online ");
  }
  static void OnUsbOnline(uint16_t code)
  {
    Serial.println("USB Disk online ");
  }
  static void OnCardInserted(uint16_t code)
  {
    Serial.println("Card inserted ");
  }
  static void OnUsbInserted(uint16_t code)
  {
    Serial.println("USB Disk inserted ");
  }
  static void OnCardRemoved(uint16_t code)
  {
    Serial.println("Card removed ");
  }
  static void OnUsbRemoved(uint16_t code)
  {
    Serial.println("USB Disk removed ");
  }
};

/* ====================================== Program Setup =============================================== */

ADXL345 adxl = ADXL345();
SoftwareSerial secondarySerial(10, 11); // RX, TX
DFMiniMp3<SoftwareSerial, Mp3Notify> mp3(secondarySerial);

void wakeUp();
void playWait();
void takeReading();

const int interruptPin0 = 2;
const int isTrackComplete = 5;
volatile int triggered = 0;


// XYZ face coordinate mapping
int coords[20][3] = {
  {-79, 134, 211},   // #1
  {158, -206, -32},  // #2
  {-226, 86, -50},   // #3
  {255, 79, -43},    // #4
  {162, -38, 216},   // #5
  {10, 151, -200},   // #6
  {-133, -31, 218},  // #7
  {-75, -133, -190}, // #8
  {17, 257, -46},    // #9
  {-136, -202, -40}, // #10
  {166, 208, 56},    // #11
  {9, -251, 65},     // #12
  {111, 135, 210},   // #13
  {159, 44, -196},   // #14
  {8, -142, 221},    // #15
  {-135, 41, -197},  // #16
  {-229, -73, 57},   // #17
  {253, -83, 55},    // #18
  {-128, 217, 52},   // #19
  {99, -130, -193}   // #20
};

// mappings for soundbyte to track index num
int announce[3] = {21,22,23};
int good[3]     = {29,31,32};
int bad[3]      = {24,25,26};
int three[2]    = {3,30};
int crit[3]     = {34,35,36};
int fail[1]     = {37};

// ISR Function that detects freefall 
void wakeUp(){
  triggered = true;
};

void setup() 
{
  randomSeed(analogRead(0));
  Serial.begin(9600);
  Serial.println("initializing...");
  mp3.begin();
  mp3.setVolume(20);
  mp3.playGlobalTrack(33);
  playWait();

  // setup the adxl global settings and thresholds
  adxl.powerOn();
  adxl.setRangeSetting(2);
  adxl.setInterruptLevelBit(0);
  adxl.setFullResBit(1);
  adxl.setFreeFallThreshold(5);
  adxl.setFreeFallDuration(5);
  adxl.setImportantInterruptMapping(0, 0, 2, 0, 0);
  adxl.FreeFallINT(1);

  pinMode(interruptPin0,INPUT);
  pinMode(isTrackComplete,INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin0), wakeUp, RISING);

  // Recommended to clear the ADXL345's interrupt source register prior to main loop
  // this ensures it does not falsely trigger or fails to trigger at all
  adxl.getInterruptSource();
  
  Serial.println("starting...");
}

/* ====================================== Track Wait Function =============================================== */

// Run the mp3.loop until the track is finished playing.
// This is used to ensure subsequent tracks will play
// during the face reading function.
void playWait()
{
  int finished = 0;
  while(1) {
    mp3.loop();
    finished = digitalRead(isTrackComplete);
    if (trackComplete && finished)
      break;
  }
  trackComplete = false;

}

/* ====================================== MCU Sleep Handler =============================================== */

// Sleep setup function that sets up the adxl interrupt and clears registers
// prior to sleep
void mcuSleep()
{

  Serial.println("Going to sleep.");
  delay(500);
  
  // Prepare the MCU for deep sleep mode
  power_all_disable();
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  

  // Unset all interrupt mappings and read interrupt source to clear the register
  // prior to resetting the interrupt register. Ensures it is clear prior to sleep
  //adxl.setImportantInterruptMapping(0, 0, 0, 0, 0);
  //adxl.FreeFallINT(0);
  
  //adxl.setImportantInterruptMapping(0, 0, 1, 0, 0);
  //adxl.FreeFallINT(1);

  // Enable ATMEGA324p interrupts 1 clock cycle prior to sleep to guarantee it goes
  // to sleep and is not interrupted prior

  interrupts();
  sleep_cpu();

  //Resumes here
  sleep_disable();
  power_twi_enable();
  power_usart0_enable();
  power_timer0_enable();
  
}

/* ====================================== MAIN PROGRAM LOOP =============================================== */

void loop() 
{
  //mcuSleep();
  triggered = false;
  adxl.getInterruptSource();

  // While loop that is currently taking the place of MCU Sleep. 
  // Should be replaced once the sleep triggering is fixed.
  while (1) {
    int check = digitalRead(interruptPin0);

    // break out of the infinite while loop once a trigger is detected
    if (triggered){
      adxl.getInterruptSource();
      delay(1000);
      triggered = false;
      break;
    }
  }
  
  // if the d20 stabilizes within a 3sec max period then calculate
  // the given face and play the applicable sound bytes
  if (stabilizeD20()){
    calculateFace();
  }
}

/* ====================================== Face Calculation Function ========================================== */

// Calculate which face is up using the Euclidean distance calculation
// (minus sqrt) and then play the applicable sound effects:
// - Roll Announcement ("ITS","YOU ROLL")
// -- FACE NUMBER ("1","2","3"...)
// --- VALUE BANTER ("HAHA","GOOD SHOW", 20(CRIT SOUNDBYTE))
void calculateFace()
{
  int32_t dX, dY, dZ, d, dMin = LONG_MAX;
  int16_t fX, fY, fZ;
  uint8_t i, iMin = 0;
  int x,y,z;

  // get now stabile xyz coords
  adxl.readAccel(&x, &y, &z);

  // iterate through the face XYZ coord map to find closest match
  for (int i=0; i < 20; i++)
  {
    // get the current index's xyz
    fX = coords[i][0];
    fY = coords[i][1];
    fZ = coords[i][2];

    // calculate the delta between the reading's and the index's xyz coords
    dX = x - fX;
    dY = y - fY;
    dZ = z - fZ;

    // calculate the relative Euclidean distance (avoids an expensive sqrt calc)
    d = (dX * dX) + (dY * dY) + (dZ * dZ);

    // determine if this is the closest calculation yet
    if (d < dMin)
    {
      dMin = d;   // Save closest Euclidean distance
      iMin = i;   // Save index for closest match
    }
  }

  // face value is 1 plus the index num
  int face = iMin + 1;
  
  long randNum;
  randNum = random(3);
  mp3.playGlobalTrack(announce[randNum]);
  playWait();

  // The 3 face has a "hidden" joke sound byte
  // only play 10 percent of the time this hits
  if (face == 3)
  {
    randNum = random(10);
    if (randNum == 0){
      mp3.playGlobalTrack(three[1]);
      playWait();
    }else{
      mp3.playGlobalTrack(three[0]);
      playWait();
    }
  }
  else{
    mp3.playGlobalTrack(face);
    playWait();
  }

  // Play banter soundbyte
  // 3 or less taunts the player
  // 17 - 19 praises the player
  // 20 plays a critical chance sound byte
  randNum = random(3);
  if (face == 1) {
    mp3.playGlobalTrack(fail[0]);
    playWait();
  }
  else if (face <= 3){
    mp3.playGlobalTrack(bad[randNum]);
    playWait();
  }
  else if (face >= 17 && face < 20){
    mp3.playGlobalTrack(good[randNum]);
    playWait();
  }
  else if (face == 20){
    int randChance = random(10);
    if (randChance < 1){
      mp3.playGlobalTrack(crit[0]);
    }
    else{
      mp3.playGlobalTrack(crit[random(1,3)]);
    }
  }
}

/* ====================================== D20 Stabilization Function =========================================== */

// take continues readings over 3s to determine if the d20
// has stabilized or if it is being moved around
bool stabilizeD20()
{
  uint32_t startTime, prevTime, currentTime;
  int prevX, prevY, prevZ;
  int x,y,z;
  int dX, dY, dZ;
  uint32_t stableTime = 250;
  uint32_t timeout = 3000;

  // get initial orientation and time
  adxl.readAccel(&prevX, &prevY, &prevZ);
  prevTime = startTime = millis();

  // repeatedly check to see if the d20 stabilizes
  // if it lasts longer than 3 seconds then we assume
  // the d20 is being carried
  while(((currentTime = millis()) - startTime) < timeout) {
    if ((currentTime - prevTime) >= stableTime) {
      Serial.println("Now Stable.");
      Serial.print("X: ");
      Serial.println(x);
      Serial.print("Y: ");
      Serial.println(y);
      Serial.print("Z: ");
      Serial.println(z);
      return true; // Stable
    }
    adxl.readAccel(&x, &y, &z);
    dX = x - prevX;
    dY = y - prevY;
    dZ = z - prevZ;

    if ((dX *dX + dY * dY + dZ *dZ) >= 300) {
      prevX = x;
      prevY = y;
      prevZ = z;
      prevTime = millis();
    }
  }
  return false;
}
