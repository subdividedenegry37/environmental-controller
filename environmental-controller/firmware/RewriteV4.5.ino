/*
 * Environmental Controller
 *
 * Dual-relay humidity / vapor pressure deficit (VPD) controller for an
 * enclosed environment. Reads temperature and relative humidity from a
 * DHT22, reads ambient light from an analog photoresistor, and drives
 * two AC relays (humidifier and dehumidifier) through a hysteretic
 * control loop with independent day and night modes.
 *
 * Hardware:
 *   - Arduino Uno / Nano (ATmega328P)
 *   - DHT22 temperature and humidity sensor
 *   - SSH1106 128x64 I2C OLED display
 *   - Analog photoresistor / LDR for day-night detection
 *   - Two mechanical relays driving 120VAC loads
 *   - Four momentary pushbuttons for menu control
 *   - Settings persisted to on-board EEPROM
 *
 * Control strategy:
 *   - 10-sample moving averages on temperature, humidity, and light
 *     readings to reject DHT22 noise and smooth analog variation
 *   - Hysteresis bands around target values to accommodate the dead
 *     time between relay switching and room-air equilibration. Without
 *     this, fast sensor response versus slow physical response produces
 *     continuous oscillation around the setpoint.
 *   - Day mode targets VPD (function of temperature and humidity);
 *     night mode targets relative humidity directly
 *   - EEPROM writes gated on change detection to prevent flash wear
 *   - Non-blocking main loop: sensors, display, EEPROM, and light
 *     sensor each run on independent timers using millis() deltas
 *
 * Status:
 *   Early self-taught embedded project (2020-2022). Kept in its
 *   original working form; heavy use of globals and repeated
 *   moving-average implementations reflect that origin.
 */

#include <Wire.h>
#include <EEPROM.h>
#include <DHT.h>

// Screen Set Up
#include <GyverOLED.h>
GyverOLED<SSH1106_128x64> oled;

// Sensor Set Up
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Humidifier and Dehumidifier Settings
bool LastOnDehumidifier = false;
bool LastOnHumidifier = false;

// VPD
float StoredVPDMovingAverage[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte LastVPDArrayPos = 0;
float AveragedVPD = 0;
float CurrentVPD = 0;

// Humidity Sensor
float AveragedHumidity = 0;
byte LastHumArrayPos = 0;
float StoredHumidityMovingAverage[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Light Sensor
byte LastLightArrayPos = 0;
float LightTotalValues = 0;
int LightArraySize = 10;
float AvLight = 0;
float LightAnalog_Reading = 0; // light array readings are different, narrowning problem
float LightMovingAverage[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Settings Vars
float TargetVPD = 1;
const float VPDTolerance = .1;

float NightModeTargetHumidity = 60;
const float NightModeHumidityTolerance = 2;

int LeafOffset = 0;
int HumidityOffset = 0;
float TempOffset = 0;

bool DisplayF = true;
byte SettingsScreenState = 0;
bool SleepMode = false;
bool NightMode = false;
bool NightModeEnabled = false;
byte RelaySetting = 0;

int LightSensorCutoff = 950;


// Relay Pins
const byte HumidifierPin = 10;
const byte DehumidifierPin = 11;





// Timer Vars
unsigned long debounceDelay = 100;    //debounce delay
const int UpdateDelayEEPROM = 30000; // How long between EEPROM checks
const int UpdateDelaySensor = 2000; // How long between Temp & Humidity Sensor checks
const int UpdateDelayScreen = 2000; // How long between Screen Refreshes
const int UpdateDelayLightSensor = 1000; // How long between Light Sensor Refreshes
unsigned long SleepDelay = 300000; // How long until screen goes to Sleep

unsigned long lastDebounceTime = 0;  // Time since the button pin was toggled
unsigned long StartTimeEEPROM = 0; // Start Time for EEPROM cycle
unsigned long StartTimeSensor = 0; // Start Time for Sensor cycle
unsigned long StartTimeScreen = 0; // Start Time for Screen cycle
unsigned long StartTimeSleep = 0; // Start Time for Sleep cycle
unsigned long StartTimeLightSensor = 0; // Start Time for Light Sensor cycle

unsigned long PresentTime = 0; // variable for storing the present time


// Menu Vars
const byte Button1Pin = 6;
const byte Button2Pin = 7;
const byte Button3Pin = 8;
const byte Button4Pin = 9;
int Button1Status = HIGH;
int Button2Status = HIGH;
int Button3Status = HIGH;
int Button4Status = HIGH;
int Button1LastState = HIGH;
int Button2LastState = HIGH;
int Button3LastState = HIGH;
int Button4LastState = HIGH;
bool ButtonReady = false;


// EEPROM
bool FirstTimeRun = true;
const byte StoredFirstTimeAddress = 6;
const byte StoredVPDSettingAddress = 11;
const byte StoredLeafOffsetAddress = 16;
const byte StoredTempOffsetAddress = 21;
const byte StoredHumidityOffsetAddress = 26;
const byte StoredTempScaleSettingAddress = 31;
const byte StoredNightModeEnabledAddress = 37;
const byte StoredNightModeHumidityAddress = 41;




void setup() {
  bool FirstTimeSetting;
  Serial.begin(9600);
  EEPROM.begin(512);
  delay(1000);

  // Buttons
  pinMode(Button1Pin, INPUT_PULLUP); //Button 1 Pin
  pinMode(Button2Pin, INPUT_PULLUP); //Button 2 Pin
  pinMode(Button3Pin, INPUT_PULLUP); //Button 3 Pin
  pinMode(Button4Pin, INPUT_PULLUP); //Button 4 Pin

  // Screen Pins
  pinMode (4, SDA);
  pinMode (5, SCL);

  // Humidity Sensor
  dht.begin();
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);

  // Relay Pins
  pinMode(HumidifierPin, OUTPUT);
  pinMode(DehumidifierPin, OUTPUT);


  // Light Sensor
pinMode(A1, INPUT_PULLDOWN);



  // Check for Stored Settings
  EEPROM.get(StoredFirstTimeAddress, FirstTimeSetting);
  if (FirstTimeSetting != false) {
    FirstTimeRun = false;
    EEPROM.put(StoredFirstTimeAddress, FirstTimeRun);
    EEPROM.commit();
  }
  // Update settings if not first time run

  else {
    EEPROM.get(StoredVPDSettingAddress, TargetVPD);
    EEPROM.get(StoredLeafOffsetAddress, LeafOffset);
    EEPROM.get(StoredTempOffsetAddress, TempOffset);
    EEPROM.get(StoredHumidityOffsetAddress, HumidityOffset);
    EEPROM.get(StoredTempScaleSettingAddress, DisplayF);
    EEPROM.get(StoredNightModeEnabledAddress, NightModeEnabled);
    EEPROM.get(StoredNightModeHumidityAddress, NightModeTargetHumidity);
  }

  //Set up Display
  oled.init();

  delay(1000);
  UpdateRelaySettings();
  DisplayHomeScreen();
}




void loop() {
  const float VPDSettingMax = 1.6;
  const float VPDSettingMin = .4;

  const int LeafSettingMax = 0;
  const int LeafSettingMin = -5;
  const int HumiditySettingMax = 50;
  const int HumiditySettingMin = -50;
  const int TempSettingMax = 50;
  const int TempSettingMin = -50;
  const float NightModeTargetHumidityMax = 100;
  const float NightModeTargetHumidityMin = 1;
  const int NumOfSettings = 6;


  PresentTime = millis(); // Updating the value of PresentTime

  // Read Buttons
  int Button1Read = digitalRead(Button1Pin);
  int Button2Read = digitalRead(Button2Pin);
  int Button3Read = digitalRead(Button3Pin);
  int Button4Read = digitalRead(Button4Pin);


  /* ==============Check if any buttons are pressed============== */
  if (Button1Read != Button1LastState || Button2Read != Button2LastState || Button3Read != Button3LastState || Button4Read != Button4LastState) {
    lastDebounceTime = millis();
  }


  // if debounce timer has elapsed since button press and values stayed the same
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (Button1Read != Button1Status) {
      Button1Status = Button1Read;
    }
    else if (Button2Read != Button2Status) {
      Button2Status = Button2Read;
    }
    else if (Button3Read != Button3Status) {
      Button3Status = Button3Read;
    }
    else if (Button4Read != Button4Status) {
      Button4Status = Button4Read;
    }



    if (Button1Status == LOW || Button2Status == LOW || Button3Status == LOW || Button4Status == LOW) {
      StartTimeSleep = PresentTime; // Reset Sleep timer's passed time value
      if (SleepMode == true) { // If in sleep, wake up and do not execute button press
        SleepMode = false;
        DisplayHomeScreen();
      }

      else if (SleepMode == false) {
        // If not in sleep mode, execute button press

        // If on Home Screen
        if (SettingsScreenState == 0) {

          if (Button1Status == LOW) {
            if (TargetVPD < VPDSettingMax) {
              TargetVPD = TargetVPD + .1;
            }

          }
          if (Button2Status == LOW) {

            if (TargetVPD > VPDSettingMin) {
              TargetVPD = TargetVPD - .1;
            }
          }
          if (Button3Status == LOW) {
            SleepMode = true;
            oled.clear();
            oled.update();
          }
          if (Button4Status == LOW) {
            SettingsScreenState = 1;

          }
        }

        // If on Setting Screen 1 - Humidity Offset
        else if (SettingsScreenState == 1) {
          if (Button1Status == LOW) {
            // Increase Setting if below Max

            if (HumidityOffset < HumiditySettingMax) {
              HumidityOffset = HumidityOffset + 1;
            }
          }
          if (Button2Status == LOW) {
            // Decrease Setting if above Min

            if (HumidityOffset > HumiditySettingMin) {
              HumidityOffset = HumidityOffset - 1;
            }

          }
          if (Button3Status == LOW) {
            // Go to Home Screen
            SettingsScreenState = 0;
          }

          if (Button4Status == LOW) { // Next Setting

            if (SettingsScreenState < NumOfSettings) {
              SettingsScreenState++;

            }
            else {
              SettingsScreenState = 1;
            }

          }

        }

        // If on Setting Screen 2 - Leaf Offset
        else if (SettingsScreenState == 2) {
          if (Button1Status == LOW) {
            // Increase Setting if below Max
            if (LeafOffset < LeafSettingMax) {
              LeafOffset = LeafOffset + 1;
            }

          }
          if (Button2Status == LOW) { // Decrease Setting if above Min

            if (LeafOffset > LeafSettingMin) {
              LeafOffset = LeafOffset - 1;
            }
          }
          if (Button3Status == LOW) { // Go to Home Screen
            SettingsScreenState = 0;
          }

          if (Button4Status == LOW) { // Next Setting

            if (SettingsScreenState < NumOfSettings) {
              SettingsScreenState++;

            }
            else {
              SettingsScreenState = 1;
            }

          }
        }

        // If on Setting Screen 3 - Temperature Offset
        else if (SettingsScreenState == 3) {
          if (Button1Status == LOW) {
            // Increase Setting if below Max
            if (TempOffset < TempSettingMax) {
              TempOffset = TempOffset + 1;
            }

          }
          if (Button2Status == LOW) {
            // Decrease Setting if above Min

            if (TempOffset > TempSettingMin) {
              TempOffset = TempOffset - 1;
            }
          }
          if (Button3Status == LOW) {
            // Go to Home Screen
            SettingsScreenState = 0;
          }

          if (Button4Status == LOW) { // Next Setting

            if (SettingsScreenState < NumOfSettings) {
              SettingsScreenState++;

            }
            else {
              SettingsScreenState = 1;
            }

          }
        }

        // If on Setting Screen 4 - Toggle Fahrenheit or Celsius
        else if (SettingsScreenState == 4) {
          if (Button1Status == LOW || Button2Status == LOW) {
            // Toggle C or F
            if (DisplayF == true) {
              DisplayF = false;
            }
            else {
              DisplayF = true;
            }
          }

          if (Button3Status == LOW) {
            // Go to Home Screen
            SettingsScreenState = 0;
          }

          if (Button4Status == LOW) {
            // Next Setting

            if (SettingsScreenState < NumOfSettings) {
              SettingsScreenState++;

            }
            else {
              SettingsScreenState = 1;
            }

          }
        }

        // If on Setting Screen 5
        else if (SettingsScreenState == 5) {
          if (Button1Status == LOW || Button2Status == LOW) {
            // Toggle NightMode
            if (NightModeEnabled == true) {
              NightModeEnabled = false;
            }
            else {
              NightModeEnabled = true;
            }
          }

          if (Button3Status == LOW) { // Go to Home Screen
            SettingsScreenState = 0;
          }

          if (Button4Status == LOW) { // Next Setting

            if (SettingsScreenState < NumOfSettings) {
              SettingsScreenState++;

            }
            else {
              SettingsScreenState = 1;
            }

          }
        }


        // If on Setting Screen 6 - Humidity Setting for NightMode
        else if (SettingsScreenState == 6) {
          // Increase Setting if below Max
          if (Button1Status == LOW) {

            if (NightModeTargetHumidity < NightModeTargetHumidityMax) {
              NightModeTargetHumidity = NightModeTargetHumidity + 1;
            }

          }
          // Decrease Setting if above Min
          if (Button2Status == LOW) {

            if (NightModeTargetHumidity > NightModeTargetHumidityMin) {
              NightModeTargetHumidity = NightModeTargetHumidity - 1;
            }
          }
          if (Button3Status == LOW) { // Go to Home Screen
            SettingsScreenState = 0;
          }

          if (Button4Status == LOW) { // Next Setting

            if (SettingsScreenState < NumOfSettings) {
              SettingsScreenState++;

            }
            else {
              SettingsScreenState = 1;
            }

          }
        }


        //Displaylatest after any button is pressed
        if (SleepMode == false) {
          if (SettingsScreenState == 0) {
            DisplayHomeScreen();
          }
          else {
            DisplaySettingScreen(SettingsScreenState);
          }
        }
      }

    }
    else {
      if ((PresentTime - StartTimeSleep >= SleepDelay) && (SleepMode == false)) { // If time passed since last button press is greater or equal to SleepDelay AND Sleep Mode is off, put screen to sleep
        SleepMode = true;
        oled.clear();
        oled.update();
      }
    }
    lastDebounceTime = millis();
  }

  Button1LastState = Button1Read;
  Button2LastState = Button2Read;
  Button3LastState = Button3Read;
  Button4LastState = Button4Read;

  /* ==============Temp & Humidity Sensor============== */
  if ((PresentTime - StartTimeSensor) >= UpdateDelaySensor) { // Check if time passed is equal to or more than elapsed time since this cycle started

    UpdateRelaySettings();

    StartTimeSensor = PresentTime; // updating the passed time value
  }

  /* ==============Screen============== */
  if (((PresentTime - StartTimeScreen) >= UpdateDelayScreen) && (SleepMode == false)) { // Check if time passed is equal to or more than the delay time since this cycle started and Sleep Mode is off

    // If on Home screen, Displaycurrent stats
    if (SettingsScreenState == 0) {
      DisplayHomeScreen();
    }
    // If on Setting Screen, Displaycurrent Setting Screen
    //  else {
    //    DisplaySettingScreen(SettingsScreenState);
    //  }

    StartTimeScreen = PresentTime; // updating the passed time value
  }

  /* ==============EEPROM============== */


  if ((PresentTime - StartTimeEEPROM) >= UpdateDelayEEPROM) { // Check if time passed is equal to or more than elapsed time since this cycle started
    SaveToFlash();
    StartTimeEEPROM = PresentTime; // updating the passed time value
  }


  /* ==============Light Sensor============== */
  if ((PresentTime - StartTimeLightSensor) >= UpdateDelayLightSensor) { // Check if time passed is equal to or more than elapsed time since this cycle started
    
    LightTotalValues = 0;
    LightArraySize = 10;
    LightAnalog_Reading = analogRead(A1); // getting value for a print command
    LightMovingAverage[LastLightArrayPos] = LightAnalog_Reading;

    if (LastLightArrayPos < 9) {
      LastLightArrayPos++;
    }
    else
    {
      LastLightArrayPos = 0;
    }

    // Find average of all values in the array
    for (int x = 0; x < 10; x++) {
      if (LightMovingAverage[x] != 0) {
        LightTotalValues = LightTotalValues + LightMovingAverage[x];
      }
      else {
        LightArraySize--;
      }
    }
    AvLight = LightTotalValues / LightArraySize;



    if (NightModeEnabled == true) {
      if (AvLight <= LightSensorCutoff) {
        NightMode = true;
      }
      if (AvLight > LightSensorCutoff) {
        NightMode = false;
      }
    }
    StartTimeLightSensor = PresentTime; // updating the passed time value
  }

}
void SaveToFlash() {

  bool HasValueChanged = false;
  float LastSaveTargetVPD;
  int LastSaveLeafOffset;
  int LastSaveHumidityOffset;
  float LastSaveTempOffset;
  bool LastSaveDisplayF;
  bool LastSaveNightModeEnabled;
  float LastSaveNightModeTargetHumidity;


  EEPROM.get(StoredVPDSettingAddress, LastSaveTargetVPD);
  EEPROM.get(StoredLeafOffsetAddress, LastSaveLeafOffset);
  EEPROM.get(StoredTempOffsetAddress, LastSaveTempOffset);
  EEPROM.get(StoredHumidityOffsetAddress, LastSaveHumidityOffset);
  EEPROM.get(StoredTempScaleSettingAddress, LastSaveDisplayF);
  EEPROM.get(StoredNightModeEnabledAddress, LastSaveNightModeEnabled);
  EEPROM.get(StoredNightModeHumidityAddress, LastSaveNightModeTargetHumidity);

  if (LastSaveTargetVPD != TargetVPD) {
    HasValueChanged = true;
  }
  if (LastSaveLeafOffset != LeafOffset) {
    HasValueChanged = true;
  }
  if (LastSaveTempOffset != TempOffset) {
    HasValueChanged = true;
  }
  if (LastSaveHumidityOffset != HumidityOffset) {
    HasValueChanged = true;
  }
  if (LastSaveDisplayF != DisplayF) {
    HasValueChanged = true;
  }
  if (LastSaveNightModeEnabled != NightModeEnabled) {
    HasValueChanged = true;
  }
  if (LastSaveNightModeTargetHumidity != NightModeTargetHumidity) {
    HasValueChanged = true;
  }

  if (HasValueChanged == true) {
    EEPROM.put(StoredVPDSettingAddress, TargetVPD);
    EEPROM.put(StoredLeafOffsetAddress, LeafOffset);
    EEPROM.put(StoredTempOffsetAddress, TempOffset);
    EEPROM.put(StoredHumidityOffsetAddress, HumidityOffset);
    EEPROM.put(StoredTempScaleSettingAddress, DisplayF);
    EEPROM.put(StoredNightModeEnabledAddress, NightModeEnabled);
    EEPROM.put(StoredNightModeHumidityAddress, NightModeTargetHumidity);
    EEPROM.commit();

  }
}



// Find current VPD
float FindVPD() {

  // Check current humidity
  float CurrentHumidity = dht.readHumidity();
  // Read temperature as Celsius
  float CurrentTemp = dht.readTemperature();

  // Adjusted values based on Settings
  float AdjustedHumidity = CurrentHumidity + HumidityOffset;
  float AdjustedTemp = CurrentTemp + ((TempOffset * 5) / 9);



  //leaf offset is using the Saturation pressure of the leaf compared to the room
  float leaf = 0.611 * exp((17.502 * (AdjustedTemp + LeafOffset)) / ((AdjustedTemp + LeafOffset) + 240.97));
  float e_sat = 0.611 * exp((17.502 * AdjustedTemp) / (AdjustedTemp + 240.97));
  float e_act = e_sat * AdjustedHumidity / 100;
  float VPD = leaf - e_act;

  return (VPD);
}


// Return moving average for VPD
float VPDMovingAverage() {

  float AvVPD;
  float TotalValues = 0;
  int ArraySize = 10;

  // Add newest value to array
  StoredVPDMovingAverage[LastVPDArrayPos] = FindVPD();

  if (LastVPDArrayPos < 9) {
    LastVPDArrayPos++;
  }
  else {
    LastVPDArrayPos = 0;
  }



  // Find average of all values in the array
  for (int x = 0; x < 10; x++) {
    if (StoredVPDMovingAverage[x] != 0) {
      TotalValues = TotalValues + StoredVPDMovingAverage[x];
    }
    else {
      ArraySize--;
    }
  }
  AvVPD = TotalValues / ArraySize;

  /*
    Serial.println("xxxxxxxxxxx");
    Serial.print("Total Value: ");
    Serial.println(TotalValues);
    Serial.print("ArraySize: ");
    Serial.println(ArraySize);
    Serial.print("AvVPD: ");
    Serial.println(AvVPD);
    Serial.println("xxxxxxxxxxx");
  */
  return (AvVPD);
}

// Return moving average for Humidity
float HumidityMovingAverage() {

  float AvHumidity;
  float TotalValues = 0;
  int ArraySize = 10;


  // Add newest value to array
  StoredHumidityMovingAverage[LastHumArrayPos] = dht.readHumidity();
  if (LastHumArrayPos < 9) {
    LastHumArrayPos++;
  }
  else {
    LastHumArrayPos = 0;
    
  }


  // Find average of all values in the array
  for (int x = 0; x < 10; x++) {
    if (StoredHumidityMovingAverage[x] != 0) {
      TotalValues = TotalValues + StoredHumidityMovingAverage[x];
    }
    else {
      ArraySize--;
    }
  }
  AvHumidity = TotalValues / ArraySize;
  return (AvHumidity);
}


void DisplayHomeScreen() {
  oled.clear();
  oled.home();
  oled.setScale(1);

  float AdjustedTemp = 0;
  // DisplayTemp in F or C
  if (DisplayF == false) {
    AdjustedTemp = dht.readTemperature() + ((TempOffset * 5) / 9);
    oled.print(F("Temp: "));
    oled.print(AdjustedTemp);
    oled.println(F(" C"));

  }
  else {
    AdjustedTemp = dht.readTemperature() + ((TempOffset * 5) / 9);
    AdjustedTemp = (AdjustedTemp * 1.8) + 32; // convert to F
    oled.print(F("Temp: "));
    oled.print(AdjustedTemp);
    oled.println(F(" F"));
  }

  // DisplayHumidity
  float CurrentHumidity = dht.readHumidity();
  oled.print(F("Humidity: "));
  oled.println(CurrentHumidity + HumidityOffset);


  // DisplayRoom VPD
  oled.print(F("Current VPD: "));
  oled.println(VPDMovingAverage());

  // DisplayTarget VPD
  oled.print(F("Target VPD: "));
  oled.println(TargetVPD);

  //TEMP - Avg Light
  oled.print(F("Avg Light: "));
  oled.println(AvLight);
  oled.println(LightAnalog_Reading);
  oled.update();
}

void DisplaySettingScreen(int SettingScreen) {
  float ModTemp;
  float TempOffsetC;
  oled.clear();
  oled.home();


  if (SettingScreen == 1) {
    // DisplayHumidity Offset
    oled.print(F("Humidity Offset: "));
    oled.println(HumidityOffset);
    oled.print(F("Current Hum: "));
    oled.println(dht.readHumidity() + HumidityOffset);
  }

  if (SettingScreen == 2) {
    // DisplayLeaf Offset
    oled.print(F("Leaf Offset: "));
    oled.println(LeafOffset);
  }

  if (SettingScreen == 3) {
    // DisplayTemp Offset
    TempOffsetC = ((TempOffset * 5) / 9);

    oled.println(F("Temp Offset: "));
    oled.print(TempOffset);
    oled.print(F("F/"));
    oled.print(TempOffsetC);
    oled.println(F("C"));
    oled.print(F("Current Temp: "));
    if (DisplayF == false) {
      oled.print(dht.readTemperature() + TempOffsetC);
      oled.println(F(" C"));
    }
    else {
      ModTemp = dht.readTemperature() + TempOffsetC;
      ModTemp = (ModTemp * 1.8) + 32; // convert to F
      oled.print(ModTemp);
      oled.println(F(" F"));
    }
  }
  if (SettingScreen == 4) {
    // DisplayTemp Scale Toggle
    oled.println(F("Temp Scale: "));
    if (DisplayF == true) {
      oled.println(F("Fahrenheit"));
    }
    else {
      oled.println(F("Celsius"));
    }

  }

  if (SettingScreen == 5) {
    // DisplayNigthMode Toggle
    oled.print(F("NightMode: "));
    if (NightModeEnabled == true) {
      oled.println(F("Enabled"));
    }
    else {
      oled.println(F("Disabled"));
    }
  }

  if (SettingScreen == 6) {
    // DisplayNightMode Humidity Target
    oled.println(F("NightMode Humidity"));
    oled.print(F("Target:"));
    oled.println(NightModeTargetHumidity);
  }


  oled.update();
}

void UpdateRelaySettings() {
  float CurrentVPD = VPDMovingAverage();
  float CurrentHumidity = HumidityMovingAverage() + HumidityOffset;
  /*
      Serial.print("CurrentVPD: ");
      Serial.println(CurrentVPD);
      Serial.print("TargetVPD: ");
      Serial.println(TargetVPD);
      Serial.print("Relay Setting: ");
      Serial.println(RelaySetting);
      Serial.print("Last On Humid: ");
      Serial.println(LastOnHumidifier);
      Serial.print("Last On Dehumid: ");
      Serial.println(LastOnDehumidifier);
      Serial.println("-----------");
  */

  /* ==================Check status of relays================== */


  if (NightMode == false) {

    // If both Humidifier and Dehumid are off
    if (RelaySetting == 0) {

      // If neither Humidifier or Dehumidifier were on, ie first time started, turn on immediately
      if (LastOnHumidifier == false && LastOnDehumidifier == false) {

        if (CurrentVPD >= TargetVPD) { // If current VPD is greater than TargetVPD turn on Humidifier
          RelaySetting = 1;
          LastOnHumidifier = true;
          LastOnDehumidifier = false;
        }
        else if (CurrentVPD < TargetVPD) { // If current VPD is less than TargetVPD turn on Dehumidifier
          RelaySetting = 2;
          LastOnHumidifier = false;
          LastOnDehumidifier = true;
        }
      }
      else if (LastOnHumidifier == true) {
        if (CurrentVPD >= TargetVPD) {
          RelaySetting = 1; //Turn on Humidifier
        }
        else if (CurrentVPD < (TargetVPD - VPDTolerance)) {
          RelaySetting = 2; //Turn on Dehumidifier
          LastOnHumidifier = false;
          LastOnDehumidifier = true;
        }
      }

      else if (LastOnDehumidifier == true) {
        if (CurrentVPD <= TargetVPD) {
          RelaySetting = 2; //Turn on Dehumidifier
        }
        else if (CurrentVPD > (TargetVPD + VPDTolerance)) {
          RelaySetting = 1; //Turn on Humidifier
          LastOnHumidifier = true;
          LastOnDehumidifier = false;
        }
      }
    }

    else if (RelaySetting == 1) { // If Humidifier is On, turn off when less than TargetVPD
      if (CurrentVPD <= (TargetVPD)) {
        RelaySetting = 0;
        LastOnHumidifier = true;
        LastOnDehumidifier = false;
      }
    }

    else if (RelaySetting == 2) { // If Dehumidifier is On, turn off when greater than TargetVPD
      if (CurrentVPD >= (TargetVPD)) {
        RelaySetting = 0;
        LastOnHumidifier = false;
        LastOnDehumidifier = true;
      }
    }

  }

  else if (NightMode == true) {

    // If both Humidifier and Dehumid are off
    if (RelaySetting == 0) {
      // If neither Humidifier or Dehumidifier were on, ie first time started, turn on immediately
      if (LastOnHumidifier == false && LastOnDehumidifier == false) {

        if (CurrentHumidity >= NightModeTargetHumidity) { // If current Humidity is greater than TargetVPD turn on Humidifier
          RelaySetting = 2;
          LastOnHumidifier = false;
          LastOnDehumidifier = true;
        }
        else if (CurrentHumidity < NightModeTargetHumidity) { // If current VPD is less than TargetVPD turn on Dehumidifier
          RelaySetting = 1;
          LastOnHumidifier = true;
          LastOnDehumidifier = false;
        }
      }
      else if (LastOnHumidifier == true) {
        if (CurrentHumidity < NightModeTargetHumidity) {
          RelaySetting = 1; //Turn on Humidifier
        }
        else if (CurrentHumidity >= (NightModeTargetHumidity + NightModeHumidityTolerance)) {
          RelaySetting = 2; //Turn on DeHumidifier
        }
      }

      else if (LastOnDehumidifier == true) {
        if (CurrentHumidity < NightModeTargetHumidity) {
          RelaySetting = 1; //Turn on Humidifier
        }
        else if (CurrentHumidity >= (NightModeTargetHumidity - NightModeHumidityTolerance)) {
          RelaySetting = 2; //Turn on Dehumidifier
        }
      }
    }


    else if (RelaySetting == 1) { // If Humidifier is On, turn off when less than TargetVPD
      if (CurrentHumidity >= (NightModeTargetHumidity)) {
        RelaySetting = 0;
        LastOnHumidifier = true;
        LastOnDehumidifier = false;
      }
    }


    else if (RelaySetting == 2) { // If Dehumidifier is On, turn off when greater than TargetVPD
      if (CurrentHumidity < (NightModeTargetHumidity)) {
        RelaySetting = 0;
        LastOnHumidifier = false;
        LastOnDehumidifier = true;
      }
    }

  }


  /* ============Turn Relays On or Off================== */

  // Turn Both Off
  if (RelaySetting == 0) {
    digitalWrite(DehumidifierPin, HIGH);
    digitalWrite(HumidifierPin, HIGH);
  }

  // Turn Humidifier On
  else if (RelaySetting == 1) {
    digitalWrite(DehumidifierPin, HIGH);
    digitalWrite(HumidifierPin, LOW);
  }

  // Turn Dehumidifier is On
  else if (RelaySetting == 2) {
    digitalWrite(DehumidifierPin, LOW);
    digitalWrite(HumidifierPin, HIGH);
  }




}
