#include <Keypad.h> // Include statement for Keypad library.
#include <LiquidCrystal_I2C.h> // Include statement for LiquidCrystal_I2C library.
#include <DHT22.h> // Include statement for DHT22 library.

// Keypad row pins
#define R1 9
#define R2 8

// Keypad column pins
#define C1 5
#define C2 4
#define C3 3
#define C4 2

// Number of rows and columns on keypad
#define ROWS 2
#define COLS 4

// I2C address of LCD
#define i2cPort 0x27 

// Number of rows and columns on LCD
#define totalColumns 16
#define totalRows 2

#define PWM 10 // Pin for controlling DC fan speed
#define BUZZ 11 // Pin for controlling buzzer
#define DHT 12 // Pin to read data from DHT22

#define TIME 2000 // Delay time for retrieving information from DHT22

#define D_HUMI_MIN 30 // Default value used for humi_min
#define D_HUMI_MAX 80 // Default value used for humi_max
#define D_TEMP_MIN 26 // Default value used for temp_min

#define HUMI_MIN_LOWER_LIMIT 10 // Lower limit of humi_min used in set_parameters()
#define HUMI_MIN_UPPER_LIMIT 40 // Upper limit of humi_min used in set_parameters()
#define HUMI_MAX_LOWER_LIMIT 50 // Lower limit of humi_max used in set_parameters()
#define HUMI_MAX_UPPER_LIMIT 100 // Upper limit of humi_max used in set_parameters()
#define TEMP_MIN_LOWER_LIMIT 16 // Lower limit of temp_min used in set_parameters()
#define TEMP_MIN_UPPER_LIMIT 36 // Upper limit of temp_min used in set_parameters()

#define HUMI_STEP 10 // Steps of humi_min and humi_max used in set_parameters()
#define TEMP_STEP 1 // Step of temp_min used in set_parameters()

// Grid used to create keymap that indicates the keypad layout
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', '4'},
  {'5', '6', '7', '8'}
};

// Arrays of pins for the keypad's rows and columns
byte rowPins[ROWS] = {R1, R2};
byte colPins[COLS] = {C1, C2, C3, C4}; 

// Generates objects for the keypad, LCD, and DHT22
Keypad keypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
LiquidCrystal_I2C lcd(i2cPort, totalColumns, totalRows);
DHT22 dht(DHT);

unsigned long en_mil; // Variable to check whether the time has passed for retrieving information from DHT22

bool display_state = true; // Chooses the LCD's display mode between the fan speed and buzzer operation and the temperature and humidity displays

bool debug_enable = false; // Choose whether or not to display the output of dht.debug() on the serial monitor

byte action_state = 0; // Ascertains the operational status of action()

uint8_t humi_min = D_HUMI_MIN; // Set lower limit for humidity trigerring buzzer
uint8_t humi_max = D_HUMI_MAX; // Set upper limit for humidity trigerring buzzer
uint8_t temp_min = D_TEMP_MIN; // Set lower limit for temperature for fan speed control

bool buzzControle(uint8_t humi);
uint8_t dcFanControle(uint8_t temp);
void display();
void debugMode();
void action();
void keypadEvent(char key);
void set_parameters();
void setup();
void loop();

/**
 * @brief Controls the buzzer based on humidity level.
 * @param humi Humidity level.
 * @return True if the buzzer is activated, false otherwise.
 */
bool buzzControle(uint8_t humi) {
  if (humi <= humi_min || humi >= humi_max) {
    digitalWrite(BUZZ, LOW);
    delay(200);
    digitalWrite(BUZZ, HIGH);
    return true;
  }
  return false;
}

/**
 * @brief Controls the DC fan based on temperature.
 * @param temp Temperature.
 * @return Fan speed percentage.
 */
uint8_t dcFanControle(uint8_t temp) {
  if (temp < temp_min) {
    analogWrite(PWM, 0);
    delay(100);
    return 0;
  }

  uint8_t fanSpeed = (temp >= (temp_min + 4)) ? 100 : (temp - (temp_min -1)) * 20;
  analogWrite(PWM, fanSpeed * 255 / 100);
  delay(100);
  return fanSpeed;
}

/**
 * @brief Displays temperature and humidity data on the LCD.
 */
void display() {
  float humi = dht.getHumidity();
  float temp = dht.getTemperature();

  bool buzz = buzzControle((uint8_t)humi);
  uint8_t fspeed = dcFanControle((uint8_t)temp);

  lcd.clear(); 
  
  lcd.setCursor(0, 0);
  if(display_state){
    lcd.print("Temp: "); lcd.print(temp);  lcd.print(" "); lcd.print((char)223); lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Humi: "); lcd.print(humi); lcd.print(" %");
  }
  else{
    lcd.print("Fan speed: "); lcd.print(fspeed); lcd.print(" %");
    lcd.setCursor(0, 1);
    lcd.print("Buzz: ");
    lcd.print(buzz ? "On" : "Off");
  } 
}

/**
 * @brief Enters a debugging mode, printing DHT22 debug information to the serial monitor.
 */
void debugMode() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DHT22 Debug mode");
  lcd.setCursor(0, 1);
  lcd.print("Serial: 9600 b/s");

  Serial.println(dht.debug());
}

/**
 * @brief Controls different states of actions based on action_state.
 */
void action() {
  switch(action_state){
    case 0:
      (debug_enable ? debugMode() : display());
      break;

    case 1:
      (debug_enable ? Serial.begin(9600) : Serial.end());
      digitalWrite(BUZZ, HIGH);
      analogWrite(PWM, 0);
      action_state = 0;
      action();
      break;
    
    default:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Testiranje");
      lcd.setCursor(0, 1);
      lcd.print(action_state == 2 ? "dc ventilatora" : "Zujalice");
      digitalWrite(BUZZ, action_state == 3 ? LOW : HIGH);
      analogWrite(PWM, action_state == 2 ? 255 : 0);
      break;
  }
}

/**
 * @brief Responds to keypad inputs, triggering different actions based on the pressed key.
 * @param key Pressed key.
 */
void keypadEvent(char key) {
  if (keypad.getState() == PRESSED) {
    switch (key) {
      case '1': action_state = 1; debug_enable = false; display_state = true; break;
      case '2': action_state = 1; debug_enable = false; display_state = false; break;
      case '3': action_state = (action_state == 0 || action_state == 3) ? 2 : 1; break;
      case '4': action_state = (action_state == 0 || action_state == 2) ? 3 : 1; break;
      case '5': debug_enable = !debug_enable; action_state = 1; break;
      case '6': set_parameters();
      case '8': lcd.clear(); return;
      default: return;
    }
    action();
  }
}

/**
 * @brief Sets parameter values using keypad input and displays them on the LCD.
 */
void set_parameters() {
  while(true) {
    char key_parameters = keypad.getKey();

    if (keypad.getState() == PRESSED) {
      lcd.clear();
      lcd.setCursor(2, 0);
      lcd.print(humi_min);
      lcd.setCursor(7, 0);
      lcd.print(humi_max);
      lcd.setCursor(12, 0);
      lcd.print(temp_min);
      
      switch (key_parameters) {
        case '1': humi_min = (humi_min < HUMI_MIN_UPPER_LIMIT) ? humi_min + HUMI_STEP : humi_min; break;
        case '5': humi_min = (humi_min > HUMI_MIN_LOWER_LIMIT) ? humi_min - HUMI_STEP : humi_min; break;
        
        case '2': humi_max = (humi_max < HUMI_MAX_UPPER_LIMIT) ? humi_max + HUMI_STEP : humi_max; break;
        case '6': humi_max = (humi_max > HUMI_MAX_LOWER_LIMIT) ? humi_max - HUMI_STEP : humi_max; break;

        case '3': temp_min = (temp_min < TEMP_MIN_UPPER_LIMIT) ? temp_min + TEMP_STEP : temp_min; break;
        case '7': temp_min = (temp_min > TEMP_MIN_LOWER_LIMIT) ? temp_min - TEMP_STEP : temp_min; break;
        
        case '4': humi_min = D_HUMI_MIN; humi_max = D_HUMI_MAX; temp_min = D_TEMP_MIN; break;
        case '8': return;
      }
    }
  }
}

/**
 * @brief Sets up the system, initializes components, and displays initial messages.
 */
void setup() {
  keypad.setDebounceTime(100);

  lcd.init(); 
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Regulator");

  lcd.setCursor(0, 1);
  lcd.print("temperature");

  pinMode(BUZZ, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(DHT, INPUT);

  analogWrite(PWM, 255);
  delay(200);
  
  digitalWrite(BUZZ, HIGH);
  delay(300);

  analogWrite(PWM, 0);

  delay(1000);
}

/**
 * @brief Main loop function, checks for keypad input and performs actions accordingly.
 */
void loop() {
  if(millis() > en_mil){
    
    action();
    
    en_mil = millis() + TIME;
  }
  
  if(char key = keypad.getKey()) keypadEvent(key);

}
