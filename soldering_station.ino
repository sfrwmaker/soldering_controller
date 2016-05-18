// The 4 7-segment indicator and led bar indicator with the max7219 controller @ arduino nano
#include "LedControl.h"
#include <EEPROM.h>

// max7219 interface
const byte M_DIN = 12;
const byte M_CLK = 10;
const byte M_CS  = 11;
const byte M_INTENSITY = 3;                   // display intensity (overwitten by brhghtness configuration parameter)

const byte R_MAIN_PIN = 2;                    // Rotary Encoder main pin (right)
const byte R_SECD_PIN = 4;                    // Rotary Encoder second pin (left)
const byte R_BUTN_PIN = 3;                    // Rotary Encoder push button pin

const byte probePIN  = A0;                    // Thermometer pin from soldering iron
const byte heaterPIN = 5;                     // soldering iron heater pin
const byte buzzerPIN = 6;                     // simple buzzer to make a noise

const uint16_t temp_minC = 180;               // Minimum temperature in degrees of celsius
const uint16_t temp_maxC = 400;               // Maximum temperature in degrees of celsius
const uint16_t temp_minF = (temp_minC *9 + 32*5 + 2)/5;
const uint16_t temp_maxF = (temp_maxC *9 + 32*5 + 2)/5; 

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
  uint32_t ID                           each time increment by 1
  struct cfg                            config data, 8 bytes
  byte CRC                              the checksum
*/
struct cfg {
  uint16_t temp_min;                        // The minimum temperature (180 centegrees)
  uint16_t temp_max;                        // The temperature for 400 centegrees
  uint16_t temp;                            // The temperature of the iron to be start
  byte     brightness;                      // The display brightness [0-15]
  bool     celsius;                         // Temperature units: true - celsius, false - farenheit
};

class CONFIG {
  public:
    CONFIG() {
      can_write = is_valid = false;
      buffRecords = 0;
      rAddr = wAddr = 0;
      eLength = 0;
      nextRecID = 0;
      Config.temp = 470;                      // Default start temperature
      Config.temp_min = 417;                  // Default value for minimum temperature (180 Centegrees)
      Config.temp_max = 700;                  // Default value for maximum temperature (400 Centegrees)
      save_calibration = false;
    }
    void init();
    bool load(void);
    bool isValid(void)    { return is_valid; }
    uint16_t temp(void)   { return Config.temp; }
    byte getBrightness(void) { return Config.brightness; }
    bool getTempUnits(void)  { return Config.celsius; }
    bool saveTemp(uint16_t t);
    void saveConfig(byte bright, bool cels);
    void saveCalibrationData(uint16_t t_max, uint16_t t_min);
    void getCalibrationData(uint16_t& t_max, uint16_t& t_min);
  private:
    struct cfg Config;
    bool readRecord(uint16_t addr, uint32_t &recID);
    bool save(void);
    bool can_write;                           // Tha flag indicates that data can be saved
    bool is_valid;                            // Weither tha data was loaded
    bool save_calibration;                    // Weither the calibration data should be saved
    byte buffRecords;                         // Number of the records in the outpt buffer
    uint16_t rAddr;                           // Address of thecorrect record in EEPROM to be read
    uint16_t wAddr;                           // Address in the EEPROM to start write new record
    uint16_t eLength;                         // Length of the EEPROM, depends on arduino model
    uint32_t nextRecID;                       // next record ID
    const byte record_size = 16;              // The size of one record in bytes

};

 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
  eLength = EEPROM.length();
  byte t, p ,h;
  uint32_t recID;
  uint32_t minRecID = 0xffffffff;
  uint16_t minRecAddr = 0;
  uint32_t maxRecID = 0;
  uint16_t maxRecAddr = 0;
  byte records = 0;

  nextRecID = 0;

  // read all the records in the EEPROM find min and max record ID
  for (uint16_t addr = 0; addr < eLength; addr += record_size) {
    if (readRecord(addr, recID)) {
      ++records;
      if (minRecID > recID) {
        minRecID = recID;
        minRecAddr = addr;
      }
      if (maxRecID < recID) {
        maxRecID = recID;
        maxRecAddr = addr;
      }
    } else {
      break;
    }
  }

  if (records == 0) {
    wAddr = rAddr = 0;
    can_write = true;
    return;
  }

  rAddr = maxRecAddr;
  if (records < (eLength / record_size)) {    // The EEPROM is not full
    wAddr = rAddr + record_size;
    if (wAddr > eLength) wAddr = 0;
  } else {
    wAddr = minRecAddr;
  }
  can_write = true;
}

bool CONFIG::saveTemp(uint16_t t) {
  if (!save_calibration && (t == Config.temp)) return true;
  Config.temp = t;
  save_calibration = false;
  return save();  
}

void CONFIG::saveConfig(byte bright, bool cels) {
  if ((bright >= 0) && (bright <= 15))
    Config.brightness = bright;
  Config.celsius = cels;
  save();                                       // Save new data into the EEPROM
}

void CONFIG::saveCalibrationData(uint16_t t_max, uint16_t t_min) {
  Config.temp_max  = t_max;
  Config.temp_min  = t_min;
  save_calibration = true;
}

void CONFIG::getCalibrationData(uint16_t& t_max, uint16_t& t_min) {
  t_max = Config.temp_max;
  t_min = Config.temp_min;
}

bool CONFIG::save(void) {
  if (!can_write) return can_write;
  if (nextRecID == 0) nextRecID = 1;

  uint16_t startWrite = wAddr;
  uint32_t nxt = nextRecID;
  byte summ = 0;
  for (byte i = 0; i < 4; ++i) {
    EEPROM.write(startWrite++, nxt & 0xff);
    summ <<=2; summ += nxt;
    nxt >>= 8;
  }
  byte* p = (byte *)&Config;
  for (byte i = 0; i < sizeof(struct cfg); ++i) {
    summ <<= 2; summ += p[i];
    EEPROM.write(startWrite++, p[i]);
  }
  summ ++;                                  // To avoid empty records
  EEPROM.write(wAddr+record_size-1, summ);

  rAddr = wAddr;
  wAddr += record_size;
  if (wAddr > EEPROM.length()) wAddr = 0;
  return true;
}

bool CONFIG::load(void) {

  is_valid = readRecord(rAddr, nextRecID);
  nextRecID ++;
  if (is_valid) {
    if (Config.temp_min >= Config.temp_max) {
      Config.temp_min = 417; Config.temp_max = 700;
    }
    if ((Config.temp > Config.temp_max) || (Config.temp < Config.temp_min)) Config.temp = 470;
    if ((Config.brightness < 0) || (Config.brightness > 15)) Config.brightness = M_INTENSITY;
  }
  return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
  byte Buff[16];

  for (byte i = 0; i < 16; ++i) 
    Buff[i] = EEPROM.read(addr+i);
  
  byte summ = 0;
  for (byte i = 0; i < sizeof(struct cfg) + 4; ++i) {

    summ <<= 2; summ += Buff[i];
  }
  summ ++;                                    // To avoid empty fields
  if (summ == Buff[15]) {                     // Checksumm is correct
    uint32_t ts = 0;
    for (char i = 3; i >= 0; --i) {
      ts <<= 8;
      ts |= Buff[i];
    }
    recID = ts;
    byte i = 4;
    memcpy(&Config, &Buff[4], sizeof(struct cfg));
    return true;
  }
  return false;
}

//------------------------------------------ class BUZZER ------------------------------------------------------
class BUZZER {
  public:
    BUZZER(byte BuzzerPIN) { buzzerPIN = BuzzerPIN; }
    void shortBeep(void)  { tone(buzzerPIN, 3520, 80); }
  private:
    byte buzzerPIN;
};

//------------------------------------------ class BUTTON ------------------------------------------------------
class BUTTON {
  public:
    BUTTON(byte ButtonPIN, unsigned int timeout_ms = 3000) {
      pt = tickTime = 0;
      buttonPIN = ButtonPIN;
      overPress = timeout_ms;
    }
    void init(void) { pinMode(buttonPIN, INPUT_PULLUP); }
    void setTimeout(uint16_t timeout_ms = 3000) { overPress = timeout_ms; }
    byte intButtonStatus(void) { byte m = mode; mode = 0; return m; }
    void cnangeINTR(void);
    byte buttonCheck(void);
    bool buttonTick(void);
  private:
    volatile byte mode;                        // The button mode: 0 - not pressed, 1 - pressed, 2 - long pressed
    const uint16_t tickTimeout = 200;          // Period of button tick, while tha button is pressed 
    const uint16_t shortPress = 900;           // If the button was pressed less that this timeout, we assume the short button press
    uint16_t overPress;                        // Maxumum time in ms the button can be pressed
    volatile uint32_t pt;                      // Time in ms when the button was pressed (press time)
    uint32_t tickTime;                         // The time in ms when the button Tick was set
    byte buttonPIN;                            // The pin number connected to the button
};

void BUTTON::cnangeINTR(void) {          // Interrupt function, called when the button status changed
  
  bool keyUp = digitalRead(buttonPIN);
  unsigned long now_t = millis();
  if (!keyUp) {                                // The button has been pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t; 
  } else {
    if (pt > 0) {
      if ((now_t - pt) < shortPress) mode = 1; // short press
        else mode = 2;                         // long press
      pt = 0;
    }
  }
}

byte BUTTON::buttonCheck(void) {               // Check the button state, called each time in the main loop

  mode = 0;
  bool keyUp = digitalRead(buttonPIN);         // Read the current state of the button
  uint32_t now_t = millis();
  if (!keyUp) {                                // The button is pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t;
  } else {
    if (pt == 0) return 0;
    if ((now_t - pt) > shortPress)             // Long press
      mode = 2;
    else
      mode = 1;
    pt = 0;
  } 
  return mode;
}

bool BUTTON::buttonTick(void) {                // When the button pressed for a while, generate periodical ticks

  bool keyUp = digitalRead(buttonPIN);         // Read the current state of the button
  uint32_t now_t = millis();
  if (!keyUp && (now_t - pt > shortPress)) {   // The button have been pressed for a while
    if (now_t - tickTime > tickTimeout) {
       tickTime = now_t;
       return (pt != 0);
    }
  } else {
    if (pt == 0) return false;
    tickTime = 0;
  } 
  return false;
}

//------------------------------------------ class ENCODER ------------------------------------------------------
class ENCODER {
  public:
    ENCODER(byte aPIN, byte bPIN, int16_t initPos = 0) {
      pt = 0; mPIN = aPIN; sPIN = bPIN; pos = initPos;
      min_pos = -32767; max_pos = 32766; channelB = false; increment = 1;
      changed = 0;
      is_looped = false;
    }
    void init(int16_t initPos, int16_t low, int16_t upp, byte inc = 1, byte fast_inc = 0, bool looped = false) {
      min_pos = low; max_pos = upp;
      if (!write(initPos)) initPos = min_pos;
      increment = fast_increment = inc;
      if (fast_inc > increment) fast_increment = fast_inc;
      is_looped = looped;
      pinMode(mPIN, INPUT_PULLUP);
      pinMode(sPIN, INPUT_PULLUP);
    }
    void reset(int16_t initPos, int16_t low, int16_t upp, byte inc) {
      min_pos = low; max_pos = upp; 
      if (!write(initPos)) initPos = min_pos;
      increment = inc;
    }
    void set_increment(byte inc) { increment = inc; }
    byte get_increment(void) { return increment; }
    bool write(int16_t initPos) {
      if ((initPos >= min_pos) && (initPos <= max_pos)) {
        pos = initPos;
        return true;
      }
      return false;
    }
    int16_t read(void) { return pos; }
    void cnangeINTR(void);
  private:
    const uint16_t overPress = 1000;
    int32_t min_pos, max_pos;
    volatile uint32_t pt;                     // Time in ms when the encoder was rotaded
    volatile uint32_t changed;                // Time in ms when the value was changed
    volatile bool channelB;
    volatile int32_t pos;                     // Encoder current position
    byte mPIN, sPIN;                          // The pin numbers connected to the main channel and to the socondary channel
    bool is_looped;                           // Weither the encoder is looped
    byte increment;                           // The value to add or substract for each encoder tick
    byte fast_increment;                      // The value to change encoder when in runs quickly
    const uint16_t fast_timeout = 300;        // Time in ms to change encodeq quickly
};

void ENCODER::cnangeINTR(void) {              // Interrupt function, called when the channel A of encoder changed
  
  bool rUp = digitalRead(mPIN);
  unsigned long now_t = millis();
  if (!rUp) {                                 // The channel A has been "pressed"
    if ((pt == 0) || (now_t - pt > overPress)) {
      pt = now_t;
      channelB = digitalRead(sPIN);
    }
  } else {
    if (pt > 0) {
      byte inc = increment;
      if ((now_t - pt) < overPress) {
        if ((now_t - changed) < fast_timeout) inc = fast_increment;
        changed = now_t;
        if (channelB) pos -= inc; else pos += inc;
        if (pos > max_pos) { 
          if (is_looped)
            pos = min_pos;
          else 
            pos = max_pos;
        }
        if (pos < min_pos) {
          if (is_looped)
            pos = max_pos;
          else
            pos = min_pos;
        }
      }
      pt = 0; 
    }
  }
}

//------------------------------------------ class 4 digits display -------------------------------------------
class DISPLAY7 : protected LedControl {
  public:
    DISPLAY7(byte DIN, byte CLK, byte CS, byte BRIGHT) : LedControl((int)DIN, (int)CLK, (int)CS) {
      intensity = BRIGHT;
    }
    void init(void);
    void clear(void) { this->clearDisplay(0); }
    void brightness(byte BRIGHT);             // Set the display brightness
    void number(int data, byte dot_mask);     // Set the digital number to be displayed
    void message(char msg[4]);                // Set the 4-shar word to be displayed
    void heating(void);                       // The highest digit animate heating process
    void cooling(void);                       // The highest digit animate cooling process
	  void P(void);                             // The highest digit show 'setting power' process
    void tSet(void);                          // THe highest digit show 'temperature set' information (idle state)
    void upper(void);                         // The highest digit show 'setting upper temperature' process
    void lower(void);                         // The highest digit show 'setting lower temperature' process
    void setupMode(byte mode);                // Show the onfigureation mode [0 - 2]
    void noAnimation(void);                   // Switch off the animation in the highest digit
    void show(void);                          // Show the data on the 4-digit indicator
    void percent(byte Power);                 // Show the percentage on the led bar (for example power supplied)
  private:
    byte intensity;                           // The display brightness
    byte dot_mask;                            // the decimal dot mask
    bool big_number;                          // If the number is big, we cannot use highest digit for animation
    byte animate_type;                        // animation type: 0 - off, 1 - heating, 2 - cooling
    byte animate_count;                       // The number of the bytes in the animation
    byte animate_index;                       // Current byte in the animation
    uint32_t animate_ms;                      // The time in ms when animation should change to the symbol
    const uint16_t animate_speed = 100;       // milliseconds to switch next byte in the animation string (100)
    const byte a_heating[5] = {
      0b00001000, 0b00010100, 0b00000001, 0b00100010, 0b01000000
    };
    const byte a_cooling[4] = {
      0b01000000, 0b00000001, 0b00001000, 0
    };
    const byte setup_mode[3][4] = {
      {0b00011111, 0b00000101, 0b01111011, 0b00001111},   // 'brgt'
      {0b00000000, 0b01001110, 0b00000001, 0b01000111},   // 'C-F'
      {0b00001111, 0b00011100, 0b00010101, 0b01001111}    // 'tune'
    };
};

void DISPLAY7::init(void) {
  this->shutdown(0, false);
  this->setIntensity(0, intensity);
  this->clear();
  delay(500);
  dot_mask = 0;
  big_number = false;
  noAnimation();
}

void DISPLAY7::brightness(byte BRIGHT) {
  if (BRIGHT > 15) BRIGHT = 15;
  intensity = BRIGHT;
  this->setIntensity(0, intensity);
}

void DISPLAY7::noAnimation(void) {
  animate_type = 0;
  animate_count = animate_index = 0;
  animate_ms = 0;
}

void DISPLAY7::number(int data, byte dot_mask) {
  big_number = (data >= 1000);
  byte i;
  byte m = 1;
  for (i = 0; i < 4; ++i) {
    if (data == 0 && i != 0) break;
    byte s = data % 10;
    data /= 10;
    this->setDigit(0, 3-i, s, (m & dot_mask));
    m <<= 1;
  }
  for (; i < 4; ++i) {
    this->setChar(0, 3-i, ' ', (m & dot_mask));
    m <<= 1;
  }
}

void DISPLAY7::message(char msg[4]) {
  for (byte i = 0; i < 4; ++i)
    this->setChar(0, i, msg[i], false);
  animate_type = 0;
}

void DISPLAY7::setupMode(byte mode) {
  if ((mode >= 0) && (mode <= 2)) {
    for (byte i = 0; i < 4; ++i)
      this->setRow(0, i, setup_mode[mode][i]);
  }
}

void DISPLAY7::heating(void) {
  if (big_number) return;
  animate_type = 1;
  animate_count = 5;
  animate_index = 0;
  animate_ms = 0;
  animate_ms = millis() + animate_speed;
  this->setRow(0, 0, a_heating[0]);
}

void DISPLAY7::cooling(void) {
  if (big_number) return;
  animate_type = 2;
  animate_count = 4;
  animate_index = 0;
  animate_ms = 0;
  animate_ms = millis() + animate_speed * 5;
  this->setRow(0, 0, a_cooling[0]);
}

void DISPLAY7::P(void) {
  if (big_number) return;
  animate_type = 0;
  this->setChar(0, 0, 'P', true);
}

void DISPLAY7::upper(void) {
  if (big_number) return;
  animate_type = 0;
  this->setRow(0, 0, 0b01100011);
}

void DISPLAY7::lower(void) {
  if (big_number) return;
  animate_type = 0;
  this->setRow(0, 0, 0b00011101);
}

void DISPLAY7::tSet(void) {
  if (big_number) return;
  animate_type = 0;
  this->setRow(0, 0, 0b10001111);
}

void DISPLAY7::show(void) {
  if (!big_number && animate_type) {
    if ((animate_count > 1) && (animate_ms < millis())) {
      if(++animate_index >= animate_count) animate_index = 0;
      switch (animate_type) {
        case 1:
          animate_ms = millis() + animate_speed;
          this->setRow(0, 0, a_heating[animate_index]);
          break;
        case 2:
          animate_ms = millis() + animate_speed * 5;
          this->setRow(0, 0, a_cooling[animate_index]);
          break;
        default:
          break;
      }
    }    
  }
}

void DISPLAY7::percent(byte Power) {
  if (Power > 10) Power = 10;
  uint16_t mask = 0;
  for (byte i = 0; i < Power; ++i) {
    mask >>= 1;
    mask |= 0x8000;                           // 15-th bit
  }
  byte m1 = mask >> 8;
  byte m2 = mask & 0xff;
  this->setRow(0, 4, m1);
  this->setRow(0, 5, m2);
}

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 8
class HISTORY {
  public:
    HISTORY(void) { len = 0; }
    void init(void) { len = 0; }
    void put(int item) {
      if (len < H_LENGTH) {
        queue[len++] = item;
      } else {
        for (byte i = 0; i < len-1; ++i) queue[i] = queue[i+1];
        queue[H_LENGTH-1] = item;
      }
    }
    bool isFull(void) { return len == H_LENGTH; }
    int top(void) { return queue[0]; }
    int average(void);
    float dispersion(void);
    float gradient(void);
  private:
    int queue[H_LENGTH];
    byte len;
};

int HISTORY::average(void) {
  long sum = 0;
  if (len == 0) return 0;
  if (len == 1) return queue[0];
  for (byte i = 0; i < len; ++i) sum += queue[i];
  sum += len >> 1;                      // round the average
  sum /= len;
  return (int)sum;
}

float HISTORY::dispersion(void) {
  if (len < 3) return 1000;
  long sum = 0;
  long avg = average();
  for (byte i = 0; i < len; ++i) {
    long q = queue[i];
    q -= avg;
    q *= q;
    sum += q;
  }
  sum += len << 1;
  float d = (float)sum / (float)len;
  return d;
}

// approfimating the history with the line (y = ax+b) using method of minimum square. Gradient is parameter a
float HISTORY::gradient(void) {
  if (len < 2) return 0;
  long sx, sx_sq, sxy, sy;
  sx = sx_sq = sxy = sy = 0;
  for (byte i = 1; i <= len; ++i) {
    sx    += i;
  sx_sq += i*i;
  sxy   += i*queue[i-1];
  sy    += queue[i-1];
  }
  long numerator   = len * sxy - sx * sy;
  long denominator = len * sx_sq - sx * sx;
  float a = (float)numerator / (float)denominator;
  return a;
}

//------------------------------------------ class soldering iron ---------------------------------------------
/*  Use PID algorythm 
 *  Un = Kp*(Xs - Xn) + Ki*summ{j=0; j<=n}(Xs - Xj) + Kd(Xn-1 - Xn)
 *  We use the ineractive formulae:
 *    Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(2*Xn-1 - Xn - Xn-2)
 *  With the first step:
 *  U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;
 */
class IRON {
  public:
    IRON(byte heater_pin, byte sensor_pin) {
      hPIN = heater_pin;
      sPIN = sensor_pin;
      on = false;
      unit_celsius = true;
	    fix_power = false;
      unit_celsius = true;
    }
    void init(uint16_t t_max, uint16_t t_min);
    void switchPower(bool On);
    bool isOn(void) { return on; }
    bool isCold(void) { return (temp() < temp_cold); }
    void setTempUnits(bool celsius) { unit_celsius = celsius; }
    bool getTempUnits(void) { return unit_celsius; }
    uint16_t getTemp(void) { return temp_set; }
    void setTemp(int t);                  // Set the temperature to be keeped
    // Set the temperature to be keeped in human readable units (celsius or farenheit)
    void setTempHumanUnits(int t);
    // Translate internal temperature to the celsius or farenheit
    uint16_t temp2humanUnits(uint16_t temp);
    uint16_t temp(void);                  // The actual temperature of the soldering iron
	  uint16_t lastTemp(void) { return temp_hist[1]; }
    byte getPower(void);                  // power that is applied to the soldering iron [0 - 255]
    byte getAvgPower(void);               // average applied power
    byte appliedPower(void);              // Power applied to the solder [0-10]
    byte hotPercent(void);                // How hot is the iron (used in the idle state)
    void keepTemp(void);                  // Main solder iron loop
	  bool fixPower(byte Power);            // Set the specified power to the the soldering iron
  private:
    void applyPower(void);                // Check the the power limits and apply power to the heater
    void resetPID(void);                  // Reset PID algorythm parameters
    uint32_t checkMS;                     // Milliseconds to measure the temperature next time
    byte hPIN, sPIN;                      // the heater PIN and the sensor PIN
    int  power;                           // The soldering station power
    bool on;                              // Weither the soldering iron is on
	  bool fix_power;                       // Weither the soldering iron is set the fix power
    bool unit_celsius;                    // Human readable units for the temparature (celsius or farenheit)
    int  temp_set;                        // The temperature that should be established
    int  temp_hist[2];                    // previously measured temperature
    bool iron_checked;                    // Weither the iron works
    int  temp_start;                      // The temperature when the solder was switched on
    int  elapsed_time;                    // The time elipsed from the start (in seconds)
    uint16_t temp_min;                    // The minimum temperature (180 centegrees)
    uint16_t temp_max;                    // The maximum temperature (400 centegrees)
    bool pid_iterate;                     // Weither the inerative PID formulae can be used
    HISTORY h_power;
    const uint16_t temp_cold  = 255;      // The cold temperature to touch the iron safely
    const byte max_power = 220;           // maximum power to the iron (180)
    const byte min_power = 25;            // minimum power to the iron
    const byte delta_t = 2;               // measurement error of the temperature
    const uint16_t period = 500;          // The period to check the soldering iron temperature, ms
    const int check_time = 10;            // Time in seconds to check weither the solder is heating
    const int heat_expected = 10;         // The iron should change the temperature at check_time
    const byte denominator_p = 10;        // The common coefficeient denominator power of 2 (10 means 1024)
    const int Kp = 5120;                  // Kp multiplied by denominator
    const int Ki = 512;                   // Ki multiplied by denominator
    const int Kd = 768;                   // Kd multiplied by denominator
};

void IRON::setTemp(int t) {
  if (on) resetPID();
  temp_set = t;
}

void IRON::setTempHumanUnits(int t) {
  int temp;
  if (unit_celsius) {
    if (t < temp_minC) t = temp_minC;
    if (t > temp_maxC) t = temp_maxC;
    temp = map(t+1, temp_minC, temp_maxC, temp_min, temp_max);
  } else {
    if (t < temp_minF) t = temp_minF;
    if (t > temp_maxF) t = temp_maxF;
    temp = map(t+2, temp_minF, temp_maxF, temp_min, temp_max);
  }
  for (byte i = 0; i < 10; ++i) {
    int tH = temp2humanUnits(temp);
    if (tH <= t) break;
    --temp;
  }
  setTemp(temp);
}

uint16_t IRON::temp2humanUnits(uint16_t temp) {
  if (!unit_celsius)  return map(temp, temp_min, temp_max, temp_minF, temp_maxF);
  return map(temp, temp_min, temp_max, temp_minC, temp_maxC);  
}

void IRON::resetPID(void) {
  pid_iterate = false;
  temp_hist[0] = temp_hist[1] = 0;
}

byte IRON::appliedPower(void) {
  byte p = getPower(); 
  return map(p, 0, max_power, 0, 10);  
}

byte IRON::getPower(void) {
  int p = power;
  if (p < 0) p = 0;
  if (p > max_power) p = max_power;
  return p & 0xff;  
}

byte IRON::getAvgPower(void) {
  int p = h_power.average();
  return p & 0xff;  
}

byte IRON::hotPercent(void) {
  uint16_t t = this->temp();
  char r = map(t, temp_cold, temp_set, 0, 10);
  if (r < 0) r = 0;
  return r;
}

void IRON::init(uint16_t t_max, uint16_t t_min) {
  pinMode(sPIN, INPUT);
  pinMode(hPIN, OUTPUT);
  digitalWrite(hPIN, LOW);
  on = false;
  fix_power = false;
  power = 0;
  checkMS = 0;

  elapsed_time = 0;
  temp_start = analogRead(sPIN);
  iron_checked = false;
  temp_max = t_max; temp_min = t_min;

  resetPID();
  h_power.init();
}

void IRON::switchPower(bool On) {
  on = On;
  if (!on) {
    digitalWrite(hPIN, LOW);
	  fix_power = false;
    return;
  }

  resetPID();
  temp_hist[1] = analogRead(sPIN);
  h_power.init();
  checkMS = millis();
}

uint16_t IRON::temp(void) {
  uint16_t t1 = analogRead(sPIN);
  delay(50);
  uint16_t t2 = analogRead(sPIN);
  if (abs(t1 - t2) < 50) {
    t1 += t2 + 1;
    t1 >>= 1;                            // average of two measurements
    return t1;
  }
  if (abs(t1 - temp_hist[1]) < abs(t2 - temp_hist[1]))
    return t1;
  else
    return t2;
}

void IRON::keepTemp(void) {
  if (checkMS > millis()) return;
  checkMS = millis() + period;

  if (!on) {                                    // If the soldering iron is set to be switched off
    if (!fix_power)
      digitalWrite(hPIN, LOW);                  // Surely power off the iron
    return;
  }
 
  int temp_curr = temp();
  
  // Check weither the iron can be heated
  if (!iron_checked) {
    elapsed_time += period / 1000;
    if (elapsed_time >= check_time) {
      if ((abs(temp_set - temp_curr) < 100) || ((temp_curr - temp_start) > heat_expected)) {
        iron_checked = true;
      } else {
        switchPower(false);                    // Prevent the iron damage
        elapsed_time = 0;
        temp_start = analogRead(sPIN);
        iron_checked = false;
      }
    }
  }

  if (temp_hist[0] == 0) {                    // first, use the direct formulae, not the iterate process
    long p = (long)Kp*(temp_set - temp_curr) + (long)Ki*(temp_set - temp_curr);
    p += (1 << (denominator_p-1));
    p >>= denominator_p;
    temp_hist[1] = temp_curr;
    if ((temp_set - temp_curr) < 30) {        // If the temperature is near, prepare the PID iteration process
      if (pid_iterate) {                      // The second loop
        temp_hist[0] = temp_hist[1];          // Now we are redy to use iterate algorythm
      } else {
        pid_iterate = true;                   // The first loop
      }
    }
    power = p;
  } else {
    long delta_p = (long)Kp * (temp_hist[1] - temp_curr);
    delta_p += (long)Ki * (temp_set - temp_curr);
    delta_p += (long)Kd * (2*temp_hist[1] - temp_hist[0] - temp_curr);
    delta_p += (1 << (denominator_p-1));
    delta_p >>= denominator_p;
    power += delta_p;
    temp_hist[0] = temp_hist[1];
    temp_hist[1] = temp_curr;
  }
  applyPower();
}

void IRON::applyPower(void) {
  byte p = getPower();
  if (temp_hist[1] > (temp_set + 1)) p = 0;
  if (p == 0) digitalWrite(hPIN, LOW);
  if (on) analogWrite(hPIN, p & 0xff);
  h_power.put(p);
}

bool IRON::fixPower(byte Power) {

  if (Power == 0) {                     // To switch off the iron, set the power to 0
    fix_power = false;
	  digitalWrite(hPIN, LOW);
	  return true;
  }

  if (Power > 80) return false;

  if (!fix_power) {
	  fix_power = true;
	  power = Power;
	  analogWrite(hPIN, power & 0xff);
  } else {
    if (power != Power) {
	    power = Power;
	    analogWrite(hPIN, power & 0xff);
	  }
  }
  return true;
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
  public:
    SCREEN* next;                       // Pointer to the next screen
    SCREEN* nextL;                      // Pointer to the next Level screen, usually, setup
    SCREEN* main;                       // Pointer to the main screen

    SCREEN() {
      next = nextL = main = 0;
      force_redraw = true;
      scr_timeout = 0;
      time_to_return = 0;
    }
    virtual void init(void) { }
    virtual void show(void) { }
    virtual SCREEN* menu(void) {if (this->next != 0) return this->next; else return this; }
    virtual SCREEN* menu_long(void) { if (this->nextL != 0) return this->nextL; else return this; }
    virtual void rotaryValue(int16_t value) { }
    bool isSetup(void){ return (scr_timeout != 0); }
    void forceRedraw(void) { force_redraw = true; }
    SCREEN* returnToMain(void) {
      if (main && (scr_timeout != 0) && (millis() >= time_to_return)) {
        scr_timeout = 0;
        return main;
      }
      return this;
    }
    void resetTimeout(void) {
      if (scr_timeout > 0)
        time_to_return = millis() + scr_timeout*1000;
    }
    void setSCRtimeout(uint16_t t) {
      scr_timeout = t;
      resetTimeout(); 
    }
  protected:
    bool force_redraw;
    uint16_t scr_timeout;               // Timeout is sec. to return to the main screen, canceling all changes
    uint32_t time_to_return;            // Time in ms to return to main screen
};

//---------------------------------------- class mainSCREEN [the soldering iron is OFF] ------------------------
class mainSCREEN : public SCREEN {
  public:
    mainSCREEN(IRON* Iron, DISPLAY7* Display, ENCODER* ENC, BUZZER* Buzz, CONFIG* Cfg) {
      update_screen = 0;
	    pIron = Iron;
	    pD = Display;
      pEnc = ENC;
      pBz = Buzz;
      pCfg = Cfg;
      is_celsius = true;
    }
	  virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    
  private:
	  IRON* pIron;                        // Pointer to the iron instance
    DISPLAY7* pD;                       // Pointer to the display instance
    ENCODER* pEnc;                      // Pointer to the rotary encoder instance
    BUZZER*  pBz;                       // Pointer to the simple buzzer instance
    CONFIG*  pCfg;                      // Pointer to the configuration instance
	  bool show_set_temp;                 // Weither show the temperature was set or 'OFF' string
	  uint32_t update_screen;             // Time in ms to switch information on the display
    bool used;                          // Weither the iron was used (was hot)
    bool cool_notified;                 // Weither there was cold notification played
    bool is_celsius;                    // The temperature units (Celsius or farenheit)
    char msg_off[4]  = {' ', '0', 'F', 'F'};
    char msg_cool[4] = {'c', '0', 'L', 'd'};
    char msg_idle[4] = {'1', 'd', 'L', 'E'};
};

void mainSCREEN::init(void) {
  pIron->switchPower(false);
  uint16_t temp_set = pIron->getTemp();
  is_celsius = pCfg->getTempUnits();
  pIron->setTempUnits(is_celsius);
  uint16_t tempH = pIron->temp2humanUnits(temp_set);
  if (is_celsius)
    pEnc->init(tempH, temp_minC, temp_maxC, 1, 5);
  else
    pEnc->init(tempH, temp_minF, temp_maxF, 1, 5);
  show_set_temp = false;
  update_screen = millis();
  pD->clear();
  forceRedraw();
  uint16_t temp = pIron->temp();
  used = ((temp > 400) && (temp < 740));
  cool_notified = !used;
  if (used) {                         // the iron was used, we should save new data in EEPROM
    pCfg->saveTemp(temp_set);
  }
}

void mainSCREEN::rotaryValue(int16_t value) {
  show_set_temp = true;
  update_screen = millis() + 1000;
  pIron->setTempHumanUnits(value);
  pD->number(value, 0);
  pD->tSet();
}

void mainSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis();

  uint16_t temp = pIron->temp();
  if (temp > 720) {                     // No iron connected
    pD->message(msg_idle);
    pD->noAnimation();
    return;
  }

  if (show_set_temp) {
    update_screen += 7000;
    uint16_t temp = pIron->getTemp();
    temp = pIron->temp2humanUnits(temp);
    pD->number(temp, 0);
    pD->tSet();
  } else {
    update_screen += 10000;
    if (used && pIron->isCold()) {
      pD->message(msg_cool);
      if (!cool_notified) {
        pBz->shortBeep();
        cool_notified = true;
      }
    } else {
	    pD->message(msg_off);
      if (used && !cool_notified) pD->cooling();
    }
  }
  byte hot = pIron->hotPercent();
  pD->percent(hot);
  show_set_temp = !show_set_temp;
}

//---------------------------------------- class workSCREEN [the soldering iron is ON] -------------------------
class workSCREEN : public SCREEN {
  public:
    workSCREEN(IRON* Iron, DISPLAY7* Display, ENCODER* Enc, BUZZER* Buzz) {
      update_screen = 0;
	    pIron = Iron;
	    pD    = Display;
      pBz   = Buzz;
      pEnc   = Enc;
	    heating_animation = ready = false;
    }
	  virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    
  private:
    bool heating_animation;               // Weither the heating animation is ON
    uint32_t update_screen;               // Time in ms to update the screen
	  IRON* pIron;                          // Pointer to the iron instance
    DISPLAY7* pD;                         // Pointer to the display instance
    BUZZER* pBz;                          // Pointer to the simple Buzzer instance
    ENCODER* pEnc;                        // Pointer to the rotary encoder instance
    bool ready;                           // Weither the iron is ready
    HISTORY hTemp;
};

void workSCREEN::init(void) {
  uint16_t temp_set = pIron->getTemp();
  bool is_celsius = pIron->getTempUnits();
  uint16_t tempH = pIron->temp2humanUnits(temp_set);
  if (is_celsius)
    pEnc->init(tempH, temp_minC, temp_maxC, 1, 5);
  else
    pEnc->init(tempH, temp_minF, temp_maxF, 1, 5);
  pIron->switchPower(true);
  heating_animation = false;
  ready = false;
  hTemp.init();
  pD->clear();
  forceRedraw();
}

void workSCREEN::rotaryValue(int16_t value) {
  heating_animation = false;
  ready = false;
  update_screen = millis() + 2000;
  pIron->setTempHumanUnits(value);
  pD->number(value, 0);
  pD->tSet();
}

void workSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + 1000;

  uint16_t temp = pIron->lastTemp();
  uint16_t temp_set = pIron->getTemp();
  hTemp.put(temp);
  int avg = hTemp.average();
  temp = pIron->temp2humanUnits(avg);
  pD->number(temp, 0);
  byte p = pIron->appliedPower();
  pD->percent(p);
  if ((abs(temp_set - avg) < 4) && (hTemp.dispersion() < 15))  {
    pD->noAnimation();
	  heating_animation = false;
    if (!ready) {
      pBz->shortBeep();
      ready = true;
    }
	  return;
  }
  if (!ready && temp < temp_set) {
    if (!heating_animation) {
      heating_animation = true;
      pD->heating();
	  }
  }
}

//---------------------------------------- class errorSCREEN [the soldering iron error detected] ---------------
class errorSCREEN : public SCREEN {
  public:
    errorSCREEN(DISPLAY7* Display) {
	    pD = Display;
    }
	  virtual void init(void) { pD->clear(); pD->message(msg_fail); }
    
  private:
    DISPLAY7* pD;                         // Pointer to the display instance
    char msg_fail[4] = {'F', 'A', '1', 'L'};
};

//---------------------------------------- class powerSCREEN [fixed power to the iron] -------------------------
class powerSCREEN : public SCREEN {
  public:
    powerSCREEN(IRON* Iron, DISPLAY7* Display, ENCODER* Enc) {
	    pIron = Iron;
	    pD = Display;
	    pEnc = Enc;
      on = false;
    }
	  virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    
  private:
	  IRON* pIron;                          // Pointer to the iron instance
    DISPLAY7* pD;                         // Pointer to the display instance
	  ENCODER* pEnc;                        // Pointer to the rotary encoder instance
    uint32_t update_screen;               // Time in ms to update the screen
    bool on;                              // Weither the power of soldering iron is on
    HISTORY hTemp;
	  const byte max_power = 100;           // Maximum possible power to be applied
};

void powerSCREEN::init(void) {
  byte p = pIron->getAvgPower();
  pEnc->init(p, 0, max_power, 1);
  on = true;                             // Do start heating immediately
  pIron->switchPower(false);
  pIron->fixPower(p);
  hTemp.init();
  pD->clear();
}

void powerSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;

  uint16_t temp = pIron->temp();
  hTemp.put(temp);
  int avg = hTemp.average();
  temp = pIron->temp2humanUnits(avg);
  pD->number(temp, 0);
  pD->noAnimation();
  byte p = pIron->appliedPower();
  pD->percent(p);
  update_screen = millis() + 500;

}

void powerSCREEN::rotaryValue(int16_t value) {
  pD->number(value, 0);
  pD->P();
  byte p = pIron->appliedPower();
  pD->percent(p);
  if (on)
    pIron->fixPower(value);
  update_screen = millis() + 1000;
}

SCREEN* powerSCREEN::menu(void) {
  on = !on;
  if (on) {
    uint16_t pos = pEnc->read();
    on = pIron->fixPower(pos);
  } else {
    pIron->fixPower(0);
  }
  return this;
}

SCREEN* powerSCREEN::menu_long(void) {
  pIron->fixPower(0);
  if (nextL) {
    pIron->switchPower(true);
    return nextL;
  }
  return this;
}

//---------------------------------------- class tuneSCREEN [tune the register and calibrating the iron] -------
class tuneSCREEN : public SCREEN {
  public:
    tuneSCREEN(IRON* Iron, DISPLAY7* Display, ENCODER* ENC, BUZZER* Buzz, CONFIG* Cfg) {
      update_screen = 0;
      pIron = Iron;
      pD = Display;
      pEnc = ENC;
      pBz  = Buzz;
      pCfg = Cfg;
      upper = true;
    }
    virtual void init(void);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    
  private:
    void saveTempParam(void);             // Save the parameters for upper or lower temperature
    IRON* pIron;                          // Pointer to the iron instance
    DISPLAY7* pD;                         // Pointer to the display instance
    ENCODER* pEnc;                        // Pointer to the rotary encoder instance
    BUZZER* pBz;                          // Pointer to the simple Buzzer instance
    CONFIG* pCfg;                         // Pointer to the configuration class
    bool upper;                           // Weither tune upper or lower temperature
    bool arm_beep;                        // Weither beep is armed
    uint32_t update_screen;               // Time in ms to switch information on the display
    uint16_t tul[2];                      // upper & lower temp
    HISTORY hTemp;
    const byte max_power = 100;           // Maximum possible power to be applied
};

void tuneSCREEN::init(void) {
  byte p = 75;
  pEnc->init(p, 0, max_power, 1);
  pIron->fixPower(p);
  update_screen = millis();
  upper = true;
  arm_beep = true;
  tul[0] = tul[1] = 0;
  hTemp.init();
  forceRedraw();
}

void tuneSCREEN::saveTempParam(void) {
  uint16_t pos = pEnc->read();
  uint16_t temp = pIron->temp();
  hTemp.put(temp);
  temp = hTemp.average();
  byte indx = 1;
  if (upper) indx = 0;
  tul[indx] = temp;
}
  
SCREEN* tuneSCREEN::menu(void) {
  saveTempParam();
  uint16_t temp = hTemp.average();
  pD->number(temp, 0);
  if (upper)
    pD->upper();
  else
    pD->lower();
  update_screen = millis() + 1000;
  upper = !upper;
  hTemp.init();
  arm_beep = true;
  force_redraw = true;
  return this;
}

SCREEN* tuneSCREEN::menu_long(void) {
  saveTempParam();
  pIron->fixPower(0);
  bool all_data = true;
  for (byte i = 0; i < 2; ++i) {
    if (!tul[i]) all_data = false;
  }
  if (all_data) {                         // save calibration data
    pCfg->saveCalibrationData(tul[0], tul[1]); 
  }
  if (nextL) return nextL;
  return this;
}

void tuneSCREEN::rotaryValue(int16_t value) {
  pD->number(value, 0);
  if (upper)
    pD->upper();
  else
    pD->lower();
  pIron->fixPower(value);
  update_screen = millis() + 1000;
}

void tuneSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + 1000;

  int16_t temp = pIron->temp();
  hTemp.put(temp);
  temp = hTemp.average();
  pD->number(temp, 0);
    if (upper)
    pD->upper();
  else
    pD->lower();
  byte p = pIron->appliedPower();
  pD->percent(p);
  if (arm_beep && (hTemp.dispersion() < 15)) {
    pBz->shortBeep();
    arm_beep = false;
  }
}

//---------------------------------------- class configSCREEN [configuration menu] -----------------------------
class configSCREEN : public SCREEN {
  public:
    configSCREEN(IRON* Iron, DISPLAY7* Display, ENCODER* Enc, CONFIG* Cfg) {
      pIron = Iron;
      pD = Display;
      pEnc = Enc;
      pCfg = Cfg;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    
  private:
    IRON* pIron;                          // Pointer to the iron instance
    DISPLAY7* pD;                         // Pointer to the display instance
    ENCODER* pEnc;                        // Pointer to the rotary encoder instance
    CONFIG*  pCfg;                        // Pointer to the config instance
    uint32_t update_screen;               // Time in ms to update the screen
    byte mode;                            // Which parameter to change: 0 - brightness, 1 - C/F, 2 - tuneSCREEN
    bool tune;                            // Weither the parameter is modified
    bool changed;                         // Weither some configuration parameter has been changed
    byte brigh;                           // Current screen brightness
    bool cels;                            // Current celsius/farenheit;
    char msg_celsius[4]   = {' ', ' ', ' ', 'c' };
    char msg_farenheit[4] = {' ', ' ', ' ', 'F' };
};

void configSCREEN::init(void) {
  mode = 0;
  pEnc->init(mode, 0, 2, 1, 0, true);
  tune    = false;
  changed = false;
  brigh   = pCfg->getBrightness();
  cels    = pCfg->getTempUnits();
  pD->clear();
  this->setSCRtimeout(30);
}

void configSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;
  force_redraw = false;
  update_screen = millis() + 10000;
  if ((mode == 1) && tune) {
    if (cels)
      pD->message(msg_celsius);
    else
      pD->message(msg_farenheit);
    return;
  }
  pD->setupMode(mode);
}

void configSCREEN::rotaryValue(int16_t value) {
  update_screen = millis() + 10000;
  if (tune) {
    changed = true;
    if (mode == 0) {
      brigh = value;
      pD->brightness(brigh);
    } else {                              // mode == 1, C/F
      cels = !value;
    }
  } else {
    mode = value;
  }
  force_redraw = true;
}

SCREEN* configSCREEN::menu(void) {
  if (tune) {
    tune = false;
    pEnc->init(mode, 0, 2, 1, 0, true);
  } else {
    switch (mode) {
      case 0:                             // brightness
        pEnc->init(brigh, 0, 15, 1);
        break;
      case 1:                             // C/F
        pEnc->init(cels, 0, 1, 1, 0, true);
        break;
      case 2:                             // Calibration
        if (next) return next;
        break;
    }
    tune = true;
  }
  force_redraw = true;
  return this;
}

SCREEN* configSCREEN::menu_long(void) {

  if (nextL) {
    if (changed) {
      pCfg->saveConfig(brigh, cels);
      pIron->setTempUnits(cels);
    }
    return nextL;
  }
  return this;
}

//=================================== End of class declarations ================================================
DISPLAY7 disp(M_DIN, M_CLK, M_CS, M_INTENSITY);
ENCODER rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON rotButton(R_BUTN_PIN);
IRON iron(heaterPIN, probePIN);

CONFIG ironCfg;
BUZZER simpleBuzzer(buzzerPIN);

mainSCREEN   offScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);
workSCREEN   wrkScr(&iron, &disp, &rotEncoder, &simpleBuzzer);
errorSCREEN  errScr(&disp);
powerSCREEN  powerScr(&iron, &disp, &rotEncoder);
configSCREEN cfgScr(&iron, &disp, &rotEncoder, &ironCfg);
tuneSCREEN  tuneScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);

SCREEN *pCurrentScreen = &offScr;

// the setup routine runs once when you press reset:
void setup() {
  disp.init();
  disp.clear();

  // Load configuration parameters
  ironCfg.init();
  bool is_cfg_valid = ironCfg.load();
  uint16_t temp_min, temp_max;
  ironCfg.getCalibrationData(temp_max, temp_min);
  if (is_cfg_valid) {
    byte dBright = ironCfg.getBrightness();
    disp.brightness(dBright);
  }

  iron.init(temp_max, temp_min);
  uint16_t temp = ironCfg.temp();
  iron.setTemp(temp);

  // Initialize rotary encoder
  rotButton.init();
  delay(500);
  attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_BUTN_PIN), rotPushChange,  CHANGE);

  // Initialize SCREEN hierarchy
  offScr.next    = &wrkScr;
  offScr.nextL   = &cfgScr;
  wrkScr.next    = &offScr;
  wrkScr.nextL   = &powerScr;
  errScr.next    = &offScr;
  errScr.nextL   = &offScr;
  powerScr.nextL = &wrkScr;
  cfgScr.next    = &tuneScr;
  cfgScr.nextL   = &offScr;
  cfgScr.main    = &offScr;
  tuneScr.nextL  = &cfgScr;
  pCurrentScreen->init();
}

void rotEncChange(void) {
  rotEncoder.cnangeINTR();
}

void rotPushChange(void) {
  rotButton.cnangeINTR();
}

// The main loop
void loop() {
  static int16_t old_pos = rotEncoder.read();

  bool iron_on = iron.isOn();
  if ((pCurrentScreen == &wrkScr) && !iron_on) {  // the soldering iron failed
    pCurrentScreen = &errScr;
	pCurrentScreen->init();
  }

  SCREEN* nxt = pCurrentScreen->returnToMain();
  if (nxt != pCurrentScreen) {                // return to the main screen by timeout
    pCurrentScreen = nxt;
    pCurrentScreen->init();
  }

  byte bStatus = rotButton.intButtonStatus();
  switch (bStatus) {
    case 2:                                   // long press;
	    nxt = pCurrentScreen->menu_long();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 1:                                   // short press
      nxt = pCurrentScreen->menu();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 0:                                   // Not pressed
    default:
      break;
  }

  int16_t pos = rotEncoder.read();
  if (old_pos != pos) {
    pCurrentScreen->rotaryValue(pos);
	  old_pos = pos;
    if (pCurrentScreen->isSetup())
     pCurrentScreen->resetTimeout();
  }

  pCurrentScreen->show();
  iron.keepTemp();
    
  disp.show();
  delay(10);
}


