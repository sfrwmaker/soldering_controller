// The 4 7-segment indicator and led bar indicator with the max7219 controller @ arduino nano
#include "LedControl.h"
#include <EEPROM.h>

// max7219 interface
const byte M_DIN = 12;
const byte M_CLK = 10;
const byte M_CS  = 11;

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
  uint16_t temp_min;                            // The minimum temperature (180 centegrees)
  uint16_t temp_max;                            // The temperature for 400 centegrees
  uint16_t temp;                                // The temperature of the iron to be start
  byte     off_timeout;                         // The Automatic switch-off timeout in minutes [0 - 30]
  bool     celsius;                             // Temperature units: true - celsius, false - farenheit
  byte     brightness;                          // The display brightness [0-15]
};

class CONFIG {
  public:
    CONFIG() {
      can_write = is_valid = false;
      buffRecords = 0;
      rAddr = wAddr = 0;
      eLength = 0;
      nextRecID = 0;
      save_calibration = false;
    }
    void init();
    bool load(void);
    bool isValid(void)       { return is_valid; }
    uint16_t temp(void)      { return Config.temp; }
    byte getOffTimeout(void) { return Config.off_timeout; }
    bool getTempUnits(void)  { return Config.celsius; }
    byte getBrightness(void) { return Config.brightness; }
    bool saveTemp(uint16_t t);
    void saveConfig(byte off, bool cels, byte bright);
    void saveCalibrationData(uint16_t t_max, uint16_t t_min);
    void getCalibrationData(uint16_t& t_max, uint16_t& t_min);
    void setDefaults(bool Write = false);
  private:
    struct cfg Config;
    bool readRecord(uint16_t addr, uint32_t &recID);
    bool save(void);
    bool can_write;                             // Tha flag indicates that data can be saved
    bool is_valid;                              // Whether tha data was loaded
    bool save_calibration;                      // Whether the calibration data should be saved
    byte buffRecords;                           // Number of the records in the outpt buffer
    uint16_t rAddr;                             // Address of thecorrect record in EEPROM to be read
    uint16_t wAddr;                             // Address in the EEPROM to start write new record
    uint16_t eLength;                           // Length of the EEPROM, depends on arduino model
    uint32_t nextRecID;                         // next record ID
    const byte record_size = 16;                // The size of one record in bytes
    const uint16_t def_min = 410;               // Default minimum temperature
    const uint16_t def_max = 700;               // Default maximum temperature
    const uint16_t def_set = 450;               // Default setup temperature
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

  setDefaults();
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
  if (records < (eLength / record_size)) {      // The EEPROM is not full
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

void CONFIG::saveConfig(byte off, bool cels, byte bright) {
  if (off > 30) off = 0;
  if (bright > 15) bright = 15;
  Config.off_timeout = off;
  Config.celsius = cels;
  Config.brightness = bright;
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
  summ ++;                                      // To avoid empty records
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
      setDefaults();
    }
    if ((Config.temp > Config.temp_max) || (Config.temp < Config.temp_min)) Config.temp = def_set;
    if ((Config.off_timeout > 30) ) Config.off_timeout = 0;
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
  summ ++;                                      // To avoid empty fields
  if (summ == Buff[15]) {                       // Checksumm is correct
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

void CONFIG::setDefaults(bool Write) {          // Restore default values
  Config.temp = def_set;
  Config.temp_min = def_min;
  Config.temp_max = def_max;
  Config.off_timeout = 0;                       // Default autometic switch-off timeout (disabled)
  Config.celsius = true;                        // Default use celsius
  Config.brightness = 3;
  if (Write) {
    save();
    save_calibration = false;
  }
}

//------------------------------------------ class BUZZER ------------------------------------------------------
class BUZZER {
  public:
    BUZZER(byte BuzzerPIN) { buzzerPIN = BuzzerPIN; }
    void shortBeep(void)  { tone(buzzerPIN, 3520, 160); }
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
    void init(void) {
      pinMode(mPIN, INPUT_PULLUP);
      pinMode(sPIN, INPUT_PULLUP);
    }
    void reset(int16_t initPos, int16_t low, int16_t upp, byte inc = 1, byte fast_inc = 0, bool looped = false) {
      min_pos = low; max_pos = upp;
      if (!write(initPos)) initPos = min_pos;
      increment = fast_increment = inc;
      if (fast_inc > increment) fast_increment = fast_inc;
      is_looped = looped;
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
    volatile uint32_t pt;                       // Time in ms when the encoder was rotaded
    volatile uint32_t changed;                  // Time in ms when the value was changed
    volatile bool channelB;
    volatile int16_t pos;                       // Encoder current position
    byte mPIN, sPIN;                            // The pin numbers connected to the main channel and to the socondary channel
    bool is_looped;                             // Whether the encoder is looped
    byte increment;                             // The value to add or substract for each encoder tick
    byte fast_increment;                        // The value to change encoder when in runs quickly
    const uint16_t fast_timeout = 300;          // Time in ms to change encodeq quickly
};

void ENCODER::cnangeINTR(void) {                // Interrupt function, called when the channel A of encoder changed
  
  bool rUp = digitalRead(mPIN);
  unsigned long now_t = millis();
  if (!rUp) {                                   // The channel A has been "pressed"
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
class DSPL : protected LedControl {
  public:
    DSPL(byte DIN, byte CLK, byte CS, byte BRIGHT) : LedControl((int)DIN, (int)CLK, (int)CS) {
      intensity = BRIGHT;
    }
    void init(void);
    void clear(void)                            { LedControl::clearDisplay(0); }
    void brightness(byte BRIGHT);               // Set the display brightness
    void tSet(uint16_t t, bool celsuis);        // Show the temperature set
    void tCurr(uint16_t t);                     // Show The current temperature
    void pSet(byte p);                          // Show the power set
    void tempLim(byte indx, uint16_t temp);     // Show the upper or lower temperature limit
    void msgNoIron(void);                       // Show 'No iron' message
    void msgReady(void);                        // Show 'Ready' message
    void msgOn(void) { }                        // Do not Show 'On' message
    void msgOff(void);                          // Show 'Off' message
    void msgCold(void);                         // Show 'Cold' message
    void msgFail(void);                         // Show 'Fail' message
    void msgTune(void);                         // Show 'Tune' message
    void msgUpper(void);                        // Show 'setting upper temperature' process
    void msgLower(void);                        // Show 'setting lower temperature' process
    void msgDefault();                          // Show 'default' message (load default configuratuin)
    void msgCancel(void);                       // Show 'cancel' message
    void msgApply(void);                        // Show 'Apply' message
    void heating(void);                         // Animate the heating process
    void cooling(void);                         // Animate the cooling process
    void setupMode(byte mode, byte p = 0);      // Show the configureation mode [0 - 2]
    void noAnimation(void);                      // Switch off the animation
    void show(void);                            // Show display animation (heating / cooling)
    void percent(byte Power);                   // Show the percentage [0-100]
  private:
    void number(int data);                      // Set the digital number to be displayed
    void message(byte msg[4]);                  // Set the 4-char word to be displayed
    byte intensity;                             // The display brightness
    bool is_cold;                               // Wheither the 'cold' message have been diplayed
    byte animate_type;                          // animation type: 0 - off, 1 - heating, 2 - cooling
    byte animate_count;                         // The number of the bytes in the animation
    byte animate_index;                         // Current byte in the animation
    uint32_t animate_ms;                        // The time in ms when animation should change to the symbol
    const uint16_t animate_speed = 100;         // milliseconds to switch next byte in the animation string (100)
    const byte a_heating[5] = {
      0b00001000, 0b00010100, 0b00000001, 0b00100010, 0b01000000
    };
    const byte a_cooling[4] = {
      0b01000000, 0b00000001, 0b00001000, 0
    };
    enum sym { s_t_dot = 0b10001111, s_up = 0b11100011, s_low = 0b10011101, s_U = 0b00111110, s_n_dot = 0b10010101};
    byte msg_off[4]      = {0b00000000, 0b01111110, 0b01000111, 0b01000111};    // " 0FF"
    byte msg_cold[4]     = {0b01001110, 0b01111110, 0b00001110, 0b00111101};    // "C0Ld"
    byte msg_idle[4]     = {0b00110000, 0b00111101, 0b00001110, 0b01001111};    // "1dLE"
    byte msg_fail[4]     = {0b01000111, 0b01110111, 0b00110000, 0b00001110};    // "FA1L"
    byte msg_cancel[4]   = {0b01001110, 0b01110111, 0b01001110, 0b00001110};    // "CACL"
    byte msg_tune[4]     = {0b00001111, 0b00011100, 0b00010101, 0b01001111};    // "tunE"
    byte msg_default[4]  = {0b00111101, 0b01001111, 0b01000111, 0b00001111};    // "dEFt"
    byte msg_apply[4]    = {0b01110111, 0b01100111, 0b00001110, 0b00111011};    // "APLY"
};

void DSPL::init(void) {
  LedControl::shutdown(0, false);
  LedControl::setIntensity(0, intensity);
  clear();
  delay(500);
  is_cold = false;
  noAnimation();
}

void DSPL::brightness(byte BRIGHT) {
  if (BRIGHT > 15) BRIGHT = 15;
  intensity = BRIGHT;
  LedControl::setIntensity(0, intensity);
}

void DSPL::tSet(uint16_t t, bool celsius) {
  number(t);
  animate_type = 0;
  LedControl::setRow(0, 0, s_t_dot);
}

void DSPL::tCurr(uint16_t t) {
  number(t);
}

void DSPL::pSet(byte p) {
  number(p);
  animate_type = 0;
  LedControl::setChar(0, 0, 'P', true);
}

void DSPL::tempLim(byte indx, uint16_t temp) {
  number(temp);
  animate_type = 0;
  byte sym = s_up;
  if (indx > 0) sym = s_low;
  LedControl::setRow(0, 0, sym);
}

void DSPL::msgNoIron(void) {
  message(msg_idle);
}

void DSPL::msgReady(void) {
  animate_type = 0;
}

void DSPL::msgOff(void) {
  if (is_cold) return;
  message(msg_off);
}

void DSPL::msgCold(void) {
  message(msg_cold);
  is_cold = true;
}

void DSPL::msgFail(void) {
  message(msg_fail);
}

void DSPL::msgTune(void) {
  message(msg_tune);
}

void DSPL::msgUpper(void) {
  animate_type = 0;
  LedControl::setRow(0, 0, s_up);
}

void DSPL::msgLower(void) {
  animate_type = 0;
  LedControl::setRow(0, 0, s_low);
}

void DSPL::msgDefault() {
  message(msg_default);
}

void DSPL::msgCancel(void) {
  message(msg_cancel);
}

void DSPL::msgApply(void) {
  message(msg_apply);
}

void DSPL::heating(void) {
  if (animate_type == 1) return;
  animate_type = 1;
  animate_count = 5;
  animate_index = 0;
  animate_ms = 0;
  animate_ms = millis() + animate_speed;
  LedControl::setRow(0, 0, a_heating[0]);
}

void DSPL::cooling(void) {
  if (animate_type != 2) {
    is_cold = false;
    animate_type = 2;
    animate_count = 4;
    animate_index = 0;
    animate_ms = 0;
    animate_ms = millis() + animate_speed * 5;
    LedControl::setRow(0, 0, a_cooling[0]);
  }
}

void DSPL::setupMode(byte mode, byte p) {
  switch (mode) {
    case 0:
      number(p);
      LedControl::setChar(0, 0, 'A', false);
      LedControl::setRow(0, 1, s_low);
      break;
    case 1:
      clear();
      LedControl::setRow(0, 0, s_U);
      LedControl::setRow(0, 1, s_n_dot);
      if (p)
        LedControl::setRow(0, 3, 0b01001110); // 'C'
      else
        LedControl::setChar(0, 3, 'F', false);
      break;
    case 2:
      number(p);
      LedControl::setRow(0, 0, 0b00011111);   // 'b'
      LedControl::setRow(0, 1, 0b10000101);   // 'r.'
      break;
    default:
      break;
  }
}

void DSPL::show(void) {
  if (animate_type) {
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

void DSPL::noAnimation(void) {
  animate_type = 0;
  animate_count = animate_index = 0;
  animate_ms = 0;
}

void DSPL::percent(byte Power) {
  if (Power > 100) Power = 100;
  Power += 5;
  Power /= 10;
  uint16_t mask = 0;
  for (byte i = 0; i < Power; ++i) {
    mask >>= 1;
    mask |= 0x8000;                           // 15-th bit
  }
  byte m1 = mask >> 8;
  byte m2 = mask & 0xff;
  LedControl::setRow(0, 4, m1);
  LedControl::setRow(0, 5, m2);
}

void DSPL::number(int data) {
  if (data >= 1000) {
    for (byte i = 1; i <= 3; ++i)
      LedControl::setChar(0, i, '-', false);
    return; 
  }
  byte i;
  for (i = 0; i < 4; ++i) {
    if (data == 0 && i != 0) break;
    byte s = data % 10;
    data /= 10;
    LedControl::setDigit(0, 3-i, s, false);
  }
  for (; i < 4; ++i) {
    LedControl::setChar(0, 3-i, ' ', false);
  }
}

void DSPL::message(byte msg[4]) {
  for (byte i = 0; i < 4; ++i)
    LedControl::setRow(0, i, msg[i]);
  animate_type = 0;
}

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 16
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
    bool  isFull(void)                          { return len == H_LENGTH; }
    int   last(void)                            { return queue[len-1]; }
    int   top(void)                             { return queue[0]; }
    int   average(void);
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
  sum += len >> 1;                              // round the average
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

//------------------------------------------ class PID algoritm to keep the temperature -----------------------
/*  The PID algoritm 
 *  Un = Kp*(Xs - Xn) + Ki*summ{j=0; j<=n}(Xs - Xj) + Kd(Xn - Xn-1),
 *  Where Xs - is the setup temperature, Xn - the temperature on n-iteration step
 *  In this program the interactive formulae is used:
 *    Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(Xn-2 + Xn - 2*Xn-1)
 *  With the first step:
 *  U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;
 */
//------------------------------------------ class PID algoritm to keep the temperature -----------------------
/*  The PID algoritm 
 *  Un = Kp*(Xs - Xn) + Ki*summ{j=0; j<=n}(Xs - Xj) + Kd(Xn - Xn-1),
 *  Where Xs - is the setup temperature, Xn - the temperature on n-iteration step
 *  In this program the interactive formulae is used:
 *    Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(Xn-2 + Xn - 2*Xn-1)
 *  With the first step:
 *  U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;
 */
class PID {
  public:
    PID(void) {
      Kp = 256;
      Ki =  54;
      Kd = 150;
    }
    void resetPID(int temp = -1);               // reset PID algoritm history parameters
    int changePID(byte p, int k);
    // Calculate the power to be applied
    int reqPower(int temp_set, int temp_curr, int power);
  private:
    void  debugPID(int t_set, int t_curr, long kp, long ki, long kd, long delta_p);
    int   temp_hist[2];                         // previously measured temperature
    bool  pid_iterate;                          // Whether the inerative PID formulae can be used
    long  Kp, Ki, Kd;                           // The PID algorithm coefficients
    const byte denominator_p = 8;               // The common coefficeient denominator power of 2 (8 means divide by 256)
};

void PID::resetPID(int temp) {
  pid_iterate = false;
  temp_hist[0] = 0;
  if (temp > 0)
    temp_hist[1] = temp;
  else
    temp_hist[1] = 0;
}

int PID::changePID(byte p, int k) {
  switch(p) {
    case 1:
      if (k >= 0) Kp = k;
      return Kp;
    case 2:
      if (k >= 0) Ki = k;
      return Ki;
    case 3:
      if (k >= 0) Kd = k;
      return Kd;
    default:
      break;
  }
  return 0;
}

int PID::reqPower(int temp_set, int temp_curr, int power) {
  if (temp_hist[0] == 0) {                      // first, use the direct formulae, not the iterate process
    long p = (long)Kp*(temp_set - temp_curr) + (long)Ki*(temp_set - temp_curr);
    p += (1 << (denominator_p-1));
    p >>= denominator_p;
    temp_hist[1] = temp_curr;
    if ((temp_set - temp_curr) < 30) {          // If the temperature is near, prepare the PID iteration process
      if (!pid_iterate) {                       // The first loop
        pid_iterate = true;
      } else {                                  // The second loop
        temp_hist[0] = temp_hist[1];            // Now we are redy to use iterate algorythm
      }
    }
    power = p;
  } else {
    long kp = Kp * (temp_hist[1] - temp_curr);
    long ki = Ki * (temp_set - temp_curr);
    long kd = Kd * (temp_hist[0] + temp_curr - 2*temp_hist[1]);
    long delta_p = kp + ki + kd;
    delta_p += (1 << (denominator_p-1));
    delta_p >>= denominator_p;
//    debugPID(temp_set, temp_curr, kp, ki, kd, delta_p);
    power += delta_p;
    temp_hist[0] = temp_hist[1];
    temp_hist[1] = temp_curr;
  }
  return power;
}

void PID::debugPID(int t_set, int t_curr, long kp, long ki, long kd, long delta_p) {
  Serial.print(t_set-t_curr); Serial.print(": ");
  Serial.print("[ "); Serial.print(temp_hist[0]);
  Serial.print(", "); Serial.print(temp_hist[1]);
  Serial.print(", "); Serial.print(t_curr);
  Serial.print(" ] kp = "); Serial.print(kp);
  Serial.print(", ki = "); Serial.print(ki);
  Serial.print(", kd = "); Serial.print(kd);
  Serial.print("; DP = "); Serial.println(delta_p);
}

//------------------------------------------ class soldering iron ---------------------------------------------
class IRON : protected PID {
  public:
    IRON(byte heater_pin, byte sensor_pin) {
      hPIN = heater_pin;
      sPIN = sensor_pin;
      on = false;
      unit_celsius = true;
      fix_power = false;
      unit_celsius = true;
      no_iron = true;
    }
    void     init(uint16_t t_max, uint16_t t_min);
    void     switchPower(bool On);
    bool     isOn(void)                         { return on; }
    bool     isCold(void)                       { return (h_temp.last() < temp_cold); }
    bool     noIron(void)                       { return no_iron; }
    void     setTempUnits(bool celsius)         { unit_celsius = celsius; }
    bool     getTempUnits(void)                 { return unit_celsius; }
    uint16_t getTemp(void)                      { return temp_set; }
    uint16_t tempAverage(void)                  { return h_temp.average(); }
    uint16_t tempDispersion(void)               { return h_temp.dispersion(); }
    uint16_t powerDispersion(void)              { return h_power.dispersion(); }
    byte     getMaxFixedPower(void)             { return max_fixed_power; }
    void     setTemp(int t);                    // Set the temperature to be keeped
    // Set the temperature to be keeped in human readable units (celsius or farenheit)
    void     setTempHumanUnits(int t);
    // Translate internal temperature to the celsius or farenheit
    uint16_t temp2humanUnits(uint16_t temp);
    byte     getAvgPower(void);                 // Average applied power
    byte     appliedPower(void);                // Power applied to the solder [0-100%]
    byte     hotPercent(void);                  // How hot is the iron (used in the idle state)
    void     keepTemp(void);                    // Main solder iron loop
    bool     fixPower(byte Power);              // Set the specified power to the the soldering iron
  private:
    uint16_t   temp(void);                      // Read the actual temperature of the soldering iron
    void       applyPower(void);                // Check the the power limits and apply power to the heater
    uint32_t   checkMS;                         // Milliseconds to measure the temperature next time
    byte       hPIN, sPIN;                      // The heater PIN and the sensor PIN
    int        power;                           // The soldering station power
    byte       actual_power;                    // The power supplied to the iron
    bool       on;                              // Whether the soldering iron is on
    bool       fix_power;                       // Whether the soldering iron is set the fix power
    bool       no_iron;                         // Whether the iron is connected
    bool       unit_celsius;                    // Human readable units for the temparature (celsius or farenheit)
    int        temp_set;                        // The temperature that should be keeped
    bool       iron_checked;                    // Whether the iron works
    int        temp_start;                      // The temperature when the solder was switched on
    uint32_t   elapsed_time;                    // The time elipsed from the start (ms)
    uint16_t   temp_min;                        // The minimum temperature (180 centegrees)
    uint16_t   temp_max;                        // The maximum temperature (400 centegrees)
    HISTORY    h_power;
    HISTORY    h_temp;
    const uint16_t temp_cold    = 255;          // The cold temperature to touch the iron safely
    const uint16_t temp_no_iron = 720;          // Sensor reading when the iron disconnected
    const byte max_power        = 180;          // maximum power to the iron (220)
    const byte max_fixed_power  = 120;          // Maximum power in fiexed power mode
    const byte delta_t          = 2;            // The measurement error of the temperature
    const uint16_t period       = 500;          // The period to check the soldering iron temperature, ms
    const int check_time        = 10000;        // Time in ms to check Whether the solder is heating
    const int heat_expected     = 10;           // The iron should change the temperature at check_time
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

byte IRON::getAvgPower(void) {
  int p = h_power.average();
  return p & 0xff;  
}

byte IRON::appliedPower(void) {
  byte p = getAvgPower(); 
  return map(p, 0, max_power, 0, 100);  
}

byte IRON::hotPercent(void) {
  uint16_t t = h_temp.average();
  char r = map(t, temp_cold, temp_set, 0, 100);
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
  actual_power = 0;
  checkMS = 0;

  elapsed_time = 0;
  temp_start = analogRead(sPIN);
  iron_checked = false;
  temp_max = t_max; temp_min = t_min;

  resetPID();
  h_power.init();
  h_temp.init();
}

void IRON::switchPower(bool On) {
  on = On;
  if (!on) {
    digitalWrite(hPIN, LOW);
    fix_power = false;
    return;
  }

  resetPID(analogRead(sPIN));
  h_power.init();
  checkMS = millis();
}

uint16_t IRON::temp(void) {
  int16_t temp = 0;
  int16_t t1 = analogRead(sPIN);
  delayMicroseconds(50);
  int16_t t2 = analogRead(sPIN);

  if (abs(t1 - t2) < 10) {                                  
    temp = t1 + t2 + 1;                         // average of two measurements
    temp >>= 1;
  } else {
    int tprev = h_temp.last();
    if (abs(t1 - tprev) < abs(t2 - tprev)) {
      temp = t1 + 3*tprev + 2;
      temp >>= 2;
    } else {
      temp = t2 + 3*tprev + 2;
      temp >>= 2;
    }
  }

  // If the power is off and no iron detected, do not put the temperature into the history 
  if (!on && !fix_power && (temp > temp_no_iron)) {
    no_iron = true;
  } else {
    no_iron = false;
    h_temp.put(temp);
  }

  return temp;
}

void IRON::keepTemp(void) {
  if (checkMS > millis()) return;
  checkMS = millis() + period;

  int temp_curr = temp();                       // Read the temperature and save it to the history buffer periodically

  if (!on) {                                    // If the soldering iron is set to be switched off
    if (!fix_power)
      digitalWrite(hPIN, LOW);                  // Surely power off the iron
    return;
  }
   
  // Check Whether the iron can be heated
  if (!iron_checked) {
    elapsed_time += period;
    if (elapsed_time >= check_time) {
      if ((abs(temp_set - temp_curr) < 100) || ((temp_curr - temp_start) > heat_expected)) {
        iron_checked = true;
      } else {
        switchPower(false);                     // Prevent the iron damage
        elapsed_time = 0;
        temp_start = analogRead(sPIN);
        iron_checked = false;
      }
    }
  }

  // Use PID algoritm to calculate power to be applied
  power = reqPower(temp_set, temp_curr, power);
  applyPower();
}

void IRON::applyPower(void) {
  int p = power;
  if (p < 0) p = 0;
  if (p > max_power) p = max_power;

  if (h_temp.last() > (temp_set + 8)) p = 0;
  if (p == 0) actual_power = 0;
  if (on) actual_power = p & 0xff;
  h_power.put(p);
  analogWrite(hPIN, actual_power);
}

bool IRON::fixPower(byte Power) {
  if (Power == 0) {                             // To switch off the iron, set the power to 0
    fix_power = false;
    actual_power = 0;
    digitalWrite(hPIN, LOW);
    return true;
  }

  if (Power > max_fixed_power) {
    actual_power = 0;
    return false;
  }

  if (!fix_power) {
    fix_power = true;
    power = Power;
    actual_power = power & 0xff;
  } else {
    if (power != Power) {
      power = Power;
      actual_power = power & 0xff;
    }
  }
  analogWrite(hPIN, actual_power);
  return true;
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
  public:
    SCREEN* next;                               // Pointer to the next screen
    SCREEN* nextL;                              // Pointer to the next Level screen, usually, setup
    SCREEN* main;                               // Pointer to the main screen
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
        time_to_return = millis() + (uint32_t)scr_timeout*1000;
    }
    void setSCRtimeout(uint16_t t) {
      scr_timeout = t;
      resetTimeout(); 
    }
  protected:
    bool force_redraw;
    uint16_t scr_timeout;                       // Timeout is sec. to return to the main screen, canceling all changes
    uint32_t time_to_return;                    // Time in ms to return to main screen
};

//---------------------------------------- class mainSCREEN [the soldering iron is OFF] ------------------------
class mainSCREEN : public SCREEN {
  public:
    mainSCREEN(IRON* Iron, DSPL* DSP, ENCODER* ENC, BUZZER* Buzz, CONFIG* Cfg) {
      update_screen = 0;
      pIron = Iron;
      pD = DSP;
      pEnc = ENC;
      pBz = Buzz;
      pCfg = Cfg;
      is_celsius = true;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
  private:
    IRON*    pIron;                             // Pointer to the iron instance
    DSPL*    pD;                                // Pointer to the DSPLay instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    BUZZER*  pBz;                               // Pointer to the simple buzzer instance
    CONFIG*  pCfg;                              // Pointer to the configuration instance
    uint32_t update_screen;                     // Time in ms to switch information on the display
    bool     used;                              // Whether the iron was used (was hot)
    bool     cool_notified;                     // Whether there was cold notification played
    bool     is_celsius;                        // The temperature units (Celsius or farenheit)
    bool     show_set_temp;                     // Wheither show the temperature was set or 'OFF' string
    const uint16_t period = 3000;               // The period to update the screen
};

void mainSCREEN::init(void) {
  show_set_temp = false;
  pIron->switchPower(false);
  uint16_t temp_set = pIron->getTemp();
  is_celsius = pCfg->getTempUnits();
  pIron->setTempUnits(is_celsius);
  uint16_t tempH = pIron->temp2humanUnits(temp_set);
  if (is_celsius)
    pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
  else
    pEnc->reset(tempH, temp_minF, temp_maxF, 1, 5);
  update_screen = millis();
  pD->clear();
  pD->tSet(tempH, is_celsius);
  pD->msgOff();
  forceRedraw();
  uint16_t temp = pIron->tempAverage();
  used = ((temp > 300) && (temp < 740));
  cool_notified = !used;
  if (used) {                                   // the iron was used, we should save new data in EEPROM
    pCfg->saveTemp(temp_set);
  }
}

void mainSCREEN::rotaryValue(int16_t value) {
  update_screen = millis() + period;
  pIron->setTempHumanUnits(value);
  pD->tSet(value, is_celsius);
}

void mainSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + period;

  byte hot = pIron->hotPercent();
  pD->percent(hot);
  show_set_temp = !show_set_temp;

  if (pIron->noIron()) {                        // No iron connected
    pD->msgNoIron();
    return;
  }
  if (show_set_temp) {
    uint16_t temp_set = pIron->getTemp();
    temp_set = pIron->temp2humanUnits(temp_set);
    pD->tSet(temp_set, is_celsius);
  } else {
    update_screen += period;
    if (used && pIron->isCold()) {
      pD->msgCold();
      if (!cool_notified) {
        pBz->shortBeep();
        cool_notified = true;
      }
    } else {
      pD->msgOff();
      if (used && !cool_notified) pD->cooling();
    }
  }
}

//---------------------------------------- class workSCREEN [the soldering iron is ON] -------------------------
class workSCREEN : public SCREEN {
  public:
    workSCREEN(IRON* Iron, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz, CONFIG* Cfg) {
      update_screen = 0;
      pIron = Iron;
      pD    = DSP;
      pBz   = Buzz;
      pEnc  = Enc;
      pCfg  = Cfg;
      ready = false;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
  private:
    uint32_t update_screen;                     // Time in ms to update the screen
    IRON*    pIron;                             // Pointer to the iron instance
    DSPL*    pD;                                // Pointer to the DSPLay instance
    BUZZER*  pBz;                               // Pointer to the simple Buzzer instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    CONFIG*  pCfg;                              // Pointer to the configuration instance
    bool     ready;                             // Whether the iron is ready
    const uint16_t period = 1000;               // The period to update the screen (ms)
};

void workSCREEN::init(void) {
  uint16_t temp_set = pIron->getTemp();
  bool is_celsius = pIron->getTempUnits();
  uint16_t tempH = pIron->temp2humanUnits(temp_set);
  if (is_celsius)
    pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
  else
    pEnc->reset(tempH, temp_minF, temp_maxF, 1, 5);
  pIron->switchPower(true);
  ready = false;
  pD->clear();
  pD->tSet(tempH, is_celsius);
  pD->msgOn();
  forceRedraw();
  uint16_t to = pCfg->getOffTimeout() * 60;
  this->setSCRtimeout(to);
}

void workSCREEN::rotaryValue(int16_t value) {
  ready = false;
  pD->msgOn();
  update_screen = millis() + period;
  pIron->setTempHumanUnits(value);
  pD->tSet(value, pIron->getTempUnits());
}

void workSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + period;

  int temp = pIron->tempAverage();
  int temp_set = pIron->getTemp();
  int tempH = pIron->temp2humanUnits(temp);
  pD->tCurr(tempH);
  byte p = pIron->appliedPower();
  pD->percent(p);
  if (pIron->powerDispersion() >= 11) {      // The iron was used
    resetTimeout();
  }

  uint16_t td = pIron->tempDispersion();
  uint16_t pd = pIron->powerDispersion();

  if ((abs(temp_set - temp) < 4) && (td <= 30))  {
    pD->noAnimation();
    if (!ready) {
      pBz->shortBeep();
      pD->msgReady();
      ready = true;
    }
    return;
  }
  if (!ready && temp < temp_set) {
    pD->heating();
  }
}

//---------------------------------------- class errorSCREEN [the soldering iron error detected] ---------------
class errorSCREEN : public SCREEN {
  public:
    errorSCREEN(DSPL* DSP) {
      pD = DSP;
    }
    virtual void init(void) { pD->clear(); pD->msgFail(); }
  private:
    DSPL* pD;                                   // Pointer to the display instance
};

//---------------------------------------- class powerSCREEN [fixed power to the iron] -------------------------
class powerSCREEN : public SCREEN {
  public:
    powerSCREEN(IRON* Iron, DSPL* DSP, ENCODER* Enc) {
      pIron = Iron;
      pD = DSP;
      pEnc = Enc;
      on = false;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
  private:
    IRON* pIron;                                // Pointer to the iron instance
    DSPL* pD;                                   // Pointer to the DSPLay instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    uint32_t update_screen;                     // Time in ms to update the screen
    bool on;                                    // Whether the power of soldering iron is on
};

void powerSCREEN::init(void) {
  byte p = pIron->getAvgPower();
  byte max_power = pIron->getMaxFixedPower();
  pEnc->reset(p, 0, max_power, 1);
  on = true;                                    // Do start heating immediately
  pIron->switchPower(false);
  pIron->fixPower(p);
  pD->clear();
  pD->pSet(p);
  pD->noAnimation();
}

void powerSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;

  uint16_t temp = pIron->tempAverage();
  temp = pIron->temp2humanUnits(temp);
  pD->tCurr(temp);
  update_screen = millis() + 500;
}

void powerSCREEN::rotaryValue(int16_t value) {
  pD->pSet(value);
  if (on)
    pIron->fixPower(value);
  update_screen = millis() + 1000;
}

SCREEN* powerSCREEN::menu(void) {
  on = !on;
  if (on) {
    uint16_t pos = pEnc->read();
    on = pIron->fixPower(pos);
    pD->clear();
    pD->pSet(pos);
    update_screen = 0;
  } else {
    pIron->fixPower(0);
    pD->clear();
    pD->pSet(0);
    pD->msgOff();
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

//---------------------------------------- class configSCREEN [configuration menu] -----------------------------
class configSCREEN : public SCREEN {
  public:
    configSCREEN(IRON* Iron, DSPL* DSP, ENCODER* Enc, CONFIG* Cfg) {
      pIron = Iron;
      pD = DSP;
      pEnc = Enc;
      pCfg = Cfg;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
  private:
    IRON* pIron;                                // Pointer to the iron instance
    DSPL* pD;                                   // Pointer to the DSPLay instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    CONFIG*  pCfg;                              // Pointer to the config instance
    uint32_t update_screen;                     // Time in ms to update the screen
    byte mode;                                  // Which parameter to change: 0 - off timeout, 1 - C/F, 2 - tuneSCREEN, 3 cancel
    bool tune;                                  // Whether the parameter is modifiying
    bool changed;                               // Whether some configuration parameter has been changed
    bool cels;                                  // Current celsius/farenheit;
    byte brightness;                            // The display brightness [0-15]
    byte off_timeout;                           // Automatic switch-off timeout in minutes
};

void configSCREEN::init(void) {
  mode = 1;
  pEnc->reset(mode, 1, 5, 1, 0, true);          // 0 - off-timeout, 1 - C/F, 2 - brightness, 3 - tuneSCREEN, 4 - save, 5 - cancel
  tune        = false;
  changed     = false;
  cels        = pCfg->getTempUnits();
  off_timeout = pCfg->getOffTimeout();
  brightness  = pCfg->getBrightness(); 
  pD->clear();
  pD->setupMode(0);
  this->setSCRtimeout(30);
}

void configSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;
  force_redraw = false;
  update_screen = millis() + 10000;
  switch (mode) {
    case 0:                                     // Automatic offline timeout
      pD->setupMode(mode, off_timeout);
      break;
    case 1:                                     // The temperature units (C/F)
      pD->setupMode(mode, cels);
      break;
    case 2:                                     // Display brightness
      pD->setupMode(mode, brightness);
      break;
    case 3:                                     // tune screen
      pD->msgTune();
      break;
    case 4:                                     // Save
      pD->msgApply();
      break;
    case 5:                                     // cancel
      pD->msgCancel();
      break;
    default:
      break;
  }
}

void configSCREEN::rotaryValue(int16_t value) {
  update_screen = millis() + 10000;
  if (tune) {                                   // tune the temperature units
    changed = true;
    switch (mode) {
      case 0:                                   // tuning the switch-off timeout
        off_timeout = value;
        break;
      case 1:                                   // tunung the temperature units
        cels = value;
        break;
      case 2:                                   // Tuning the display brightness
        brightness = value;
        pD->brightness(value);
        break;
      default:
        break;
    }
  } else {
    mode = value;
  }
  force_redraw = true;
}

SCREEN* configSCREEN::menu(void) {
  if (tune) {
    tune = false;
    pEnc->reset(mode, 1, 5, 1, 0, true);        // The value has been tuned, return to the menu list mode
  } else {
    switch (mode) {
      case 0:                                   // automatic switch-off timeout
        pEnc->reset(off_timeout, 0, 30, 1, 0, false);
        break;
      case 1:                                   // Celsius / Farenheit
        pEnc->reset(cels, 0, 1, 1, 0, true);
        break;
      case 2:                                   // The display brightness
        pEnc->reset(brightness, 0, 15, 1, 0, false);
        break;
      case 3:                                   // Calibration
        if (next) return next;
        break;
      case 4:                                   // Save
        menu_long();
        break;
      case 5:                                   // Cancel
        if (main) return main;
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
      pCfg->saveConfig(off_timeout, cels, brightness);
      pIron->setTempUnits(cels);
    }
    return nextL;
  }
  return this;
}

//---------------------------------------- class tuneSCREEN [tune the register and calibrating the iron] -------
class tuneSCREEN : public SCREEN {
  public:
    tuneSCREEN(IRON* Iron, DSPL* DSP, ENCODER* ENC, BUZZER* Buzz, CONFIG* Cfg) {
      update_screen = 0;
      pIron = Iron;
      pD = DSP;
      pEnc = ENC;
      pBz  = Buzz;
      pCfg = Cfg;
    }
    virtual void init(void);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
  private:
    IRON* pIron;                                // Pointer to the iron instance
    DSPL* pD;                                   // Pointer to the display instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    BUZZER* pBz;                                // Pointer to the simple Buzzer instance
    CONFIG* pCfg;                               // Pointer to the configuration class
    byte mode;                                  // Which temperature to tune [0-3]: select, up temp, low temp, defaults
    bool arm_beep;                              // Whether beep is armed
    byte max_power;                             // Maximum possible power to be applied
    uint32_t update_screen;                     // Time in ms to switch information on the display
    uint16_t tul[2];                            // upper & lower temp
    byte pul[2];                                // upper and lower power
};

void tuneSCREEN::init(void) {
  max_power = pIron->getMaxFixedPower();
  mode = 0;                                     // select the element from the list
  pul[0] = 75; pul[1] = 20;
  pEnc->reset(0, 0, 4, 1, 1, true);             // 0 - up temp, 1 - low temp, 2 - defaults, 3 - apply, 4 - cancel
  update_screen = millis();
  arm_beep = true;
  tul[0] = tul[1] = 0;
  pD->clear();
  pD->msgTune();
  pD->tempLim(0, 0);
  forceRedraw();
}

void tuneSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + 1000;
  if (mode != 0) {                              // Selected upper or lower temperature
    int16_t temp = pIron->tempAverage();
    pD->tCurr(temp);
    byte power = pEnc->read();                  // applied power
    power = map(power, 0, max_power, 0, 100);
    pD->percent(power);
    if (mode == 1)
      pD->msgUpper();
    else
      pD->msgLower();
  }
  if (arm_beep && (pIron->tempDispersion() < 15)) {
    pBz->shortBeep();
    arm_beep = false;
  }
}
 
void tuneSCREEN::rotaryValue(int16_t value) {
  if (mode == 0) {                              // No limit is selected, list the menu
    switch (value) {
      case 2:
        pD->msgDefault();
        break;
      case 3:
        pD->msgApply();
        break;
      case 4:
        pD->msgCancel();
        break;
      default:
       pD->tempLim(value, tul[value]);
       break;
    }
  } else {
    pIron->fixPower(value);
    force_redraw = true;
  }
  update_screen = millis() + 1000;
}
 
SCREEN* tuneSCREEN::menu(void) {                // The rotary button pressed
  if (mode == 0) {                              // select upper or lower temperature limit
    int val = pEnc->read();
    if (val == 2) {                             // load defaults
      pCfg->setDefaults(true);                  // Write default config to the EEPROM           
      if (main) return main;                    // Return to the main screen
    }
    if (val == 3) {
      menu_long();                              // Save new values
      if (nextL) return nextL;
    }
    if (val == 4) {
      if (nextL) return nextL;                  // Return to the previous menu
    }
    mode = val + 1;
    pD->clear();
    pD->msgTune();
    switch (mode) {
      case 1:                                   // upper temp
        pD->msgUpper();
        break;
      case 2:                                   // lower temp
        pD->msgLower();
        break;
      default:
        break;
    }
    pEnc->reset(pul[mode-1], 0, max_power, 1, 5);
    pIron->fixPower(pul[mode-1]);               // Switch on the soldering iron
  } else {                                      // upper or lower temperature limit just setup     
    pul[mode-1] = pEnc->read();                 // The supplied power
    tul[mode-1] = pIron->tempAverage();
    pD->clear();
    pD->msgTune();
    pEnc->reset(mode-1, 0, 4, 1, 1, true);      // 0 - up temp, 1 - low temp, 2 - defaults, 3 - apply, 4 - cancel
    mode = 0;
    pIron->fixPower(0);
  }
  arm_beep = true;
  force_redraw = true;
  return this;
}

SCREEN* tuneSCREEN::menu_long(void) {
  pIron->fixPower(0);                           // switch off the power
  bool all_data = true;
  for (byte i = 0; i < 2; ++i) {
    if (!tul[i]) all_data = false;
  }
  if (all_data) {                               // save calibration data. Config will be written to the EEPROM later on
    pCfg->saveCalibrationData(tul[0], tul[1]);
    pIron->init(tul[0], tul[1]);
  }
  if (nextL) return nextL;
  return this;
}
//=================================== End of class declarations ================================================

DSPL       disp(M_DIN, M_CLK, M_CS, 3);
ENCODER    rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON     rotButton(R_BUTN_PIN);
IRON       iron(heaterPIN, probePIN);
CONFIG     ironCfg;
BUZZER     simpleBuzzer(buzzerPIN);

mainSCREEN   offScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);
workSCREEN   wrkScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);
errorSCREEN  errScr(&disp);
powerSCREEN  powerScr(&iron, &disp, &rotEncoder);
configSCREEN cfgScr(&iron, &disp, &rotEncoder, &ironCfg);
tuneSCREEN   tuneScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);

SCREEN *pCurrentScreen = &offScr;

// the setup routine runs once when you press reset:
void setup() {
//  Serial.begin(115200);
  disp.init();

  // Load configuration parameters
  ironCfg.init();
  bool is_cfg_valid = ironCfg.load();
  uint16_t temp_min, temp_max;
  ironCfg.getCalibrationData(temp_max, temp_min);

  iron.init(temp_max, temp_min);
  uint16_t temp = ironCfg.temp();
  iron.setTemp(temp);

  // Initialize rotary encoder
  rotEncoder.init();
  rotButton.init();
  delay(500);
  attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_BUTN_PIN), rotPushChange,  CHANGE);

  // Initialize SCREEN hierarchy
  offScr.next    = &wrkScr;
  offScr.nextL   = &cfgScr;
  wrkScr.next    = &offScr;
  wrkScr.nextL   = &powerScr;
  wrkScr.main    = &offScr;
  errScr.next    = &offScr;
  errScr.nextL   = &offScr;
  powerScr.nextL = &wrkScr;
  cfgScr.next    = &tuneScr;
  cfgScr.nextL   = &offScr;
  cfgScr.main    = &offScr;
  tuneScr.nextL  = &cfgScr;
  tuneScr.main   = &offScr;
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
  iron.keepTemp();                                // First, read the temperature

  bool iron_on = iron.isOn();
  if ((pCurrentScreen == &wrkScr) && !iron_on) {  // the soldering iron failed
    pCurrentScreen = &errScr;
    pCurrentScreen->init();
  }

  SCREEN* nxt = pCurrentScreen->returnToMain();
  if (nxt != pCurrentScreen) {                  // return to the main screen by timeout
    pCurrentScreen = nxt;
    pCurrentScreen->init();
  }

  byte bStatus = rotButton.intButtonStatus();
  switch (bStatus) {
    case 2:                                     // long press;
      nxt = pCurrentScreen->menu_long();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 1:                                     // short press
      nxt = pCurrentScreen->menu();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 0:                                     // Not pressed
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
   
  disp.show();
  delay(10);
}

