#include <Keypad.h> // Include statement for Keypad library.
#include <LiquidCrystal_I2C.h> // Include statement for LiquidCrystal_I2C library.
#include <DHT22.h> // Include statement for DHT22 library.

// Keypad row pins
#define R1 9 /**< Keypad row 1 pin */
#define R2 8 /**< Keypad row 2 pin */

// Keypad column pins
#define C1 5 /**< Keypad column 1 pin */
#define C2 4 /**< Keypad column 2 pin */
#define C3 3 /**< Keypad column 3 pin */
#define C4 2 /**< Keypad column 4 pin */

// Number of rows and columns on keypad
#define ROWS 2 /**< Number of keypad rows */
#define COLS 4 /**< Number of keypad columns */

// I2C address of LCD
#define i2cPort 0x27 /**< I2C address of LCD */

// Number of rows and columns on LCD
#define totalColumns 16 /**< Total number of columns on the LCD */
#define totalRows 2 /**< Total number of rows on the LCD */

#define PWM 10 /**< PWM pin for controlling the fan speed */
#define BUZZ 11 /**< Buzzer pin */
#define DHT 12 /**< DHT22 sensor pin */

#define DEBUG_SERIAL_BAUDRATE 9600 /**< Serial baud rate for debug mode */

#define DEBOUNCE_TIME 100 /**< Keypad debounce time */

#define BUZZER_DELAY 200 /**< Delay for buzzer control */

#define DHT_UPDATE_INTERVAL 2000 /**< Interval for updating DHT22 sensor data */

#define FAN_SPEED_UPDATE_INTERVAL 100 /**< Interval for updating fan speed */

#define DEFAULT_HUMIDITY_MIN 30 /**< Default minimum humidity */
#define DEFAULT_HUMIDITY_MAX 80 /**< Default maximum humidity */
#define DEFAULT_TEMPERATURE_MIN 26 /**< Default minimum temperature */

#define HUMIDITY_MIN_LOWER_LIMIT 10 /**< Lower limit for humidity minimum */
#define HUMIDITY_MIN_UPPER_LIMIT 50 /**< Upper limit for humidity minimum */

#define HUMIDITY_MAX_LOWER_LIMIT 60 /**< Lower limit for humidity maximum */
#define HUMIDITY_MAX_UPPER_LIMIT 90 /**< Upper limit for humidity maximum */

#define TEMPERATURE_MIN_LOWER_LIMIT 16 /**< Lower limit for temperature minimum */
#define TEMPERATURE_MIN_UPPER_LIMIT 36 /**< Upper limit for temperature minimum */

#define HUMIDITY_STEP 10 /**< Step size for adjusting humidity */
#define TEMPERATURE_STEP 1 /**< Step size for adjusting temperature */

// Define keypad layout
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', '4'},
  {'5', '6', '7', '8'}
};

byte rowPins[ROWS] = {R1, R2}; /**< Array of row pins */
byte colPins[COLS] = {C1, C2, C3, C4}; /**< Array of column pins */

Keypad keypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); /**< Keypad object */
LiquidCrystal_I2C lcd(i2cPort, totalColumns, totalRows); /**< LCD object */
DHT22 dht(DHT); /**< DHT22 sensor object */

unsigned long en_mill; /**< Time variable for timing */

// Define states for action
enum ActionState {
  NORMAL_DISPLAY, /**< Normal display state */
  DEBUG_MODE,     /**< Debug mode state */
  FAN_TEST,       /**< Fan test state */
  BUZZER_TEST     /**< Buzzer test state */
};

ActionState action_state = NORMAL_DISPLAY; /**< Current action state */

bool display_state = true; /**< Flag for display state */
bool debug_enable = false; /**< Flag for debug mode */

uint8_t humidity_min = DEFAULT_HUMIDITY_MIN; /**< Minimum humidity value */
uint8_t humidity_max = DEFAULT_HUMIDITY_MAX; /**< Maximum humidity value */
uint8_t temperature_min = DEFAULT_TEMPERATURE_MIN; /**< Minimum temperature value */

// Function prototypes
bool buzzControl(uint8_t humidity);
uint8_t dcFanControl(uint8_t temperature);
void display();
void debugMode();
void action();
void keypadEvent(char key);
void set_parameters();
void setup();
void loop();

/**
 * @brief Controls the buzzer based on humidity
 * 
 * This function checks the humidity level and activates the buzzer if the humidity
 * falls below the minimum threshold or exceeds the maximum threshold.
 * 
 * @param humidity Current humidity value
 * @return True if the buzzer is activated, false otherwise
 */
bool buzzControl(uint8_t humidity) {
  if (humidity <= humidity_min || humidity >= humidity_max) {
    digitalWrite(BUZZ, LOW);
    delay(BUZZER_DELAY);
    digitalWrite(BUZZ, HIGH);
    return true;
  }
  return false;
}

/**
 * @brief Controls the speed of the DC fan based on temperature
 * 
 * This function adjusts the fan speed based on the current temperature. It calculates
 * the fan speed percentage and adjusts the PWM signal accordingly.
 * 
 * @param temperature Current temperature value
 * @return The fan speed percentage
 */
uint8_t dcFanControl(uint8_t temperature) {
  if (temperature < temperature_min) {
    analogWrite(PWM, 0);
    delay(FAN_SPEED_UPDATE_INTERVAL);
    return 0;
  }

  uint8_t fanSpeed = (temperature >= (temperature_min + 4)) ? 100 : (temperature - (temperature_min -1)) * 20;
  analogWrite(PWM, fanSpeed * 255 / 100);
  delay(FAN_SPEED_UPDATE_INTERVAL);
  return fanSpeed;
}

/**
 * @brief Displays temperature, humidity, fan speed, and buzzer state on the LCD
 */
void display() {
  float humidity = dht.getHumidity(), temperature = dht.getTemperature();
  if(dht.getLastError() != 0){
    analogWrite(PWM, 0);
    digitalWrite(BUZZ, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DHT22:");
    lcd.setCursor(0, 1);
    lcd.print("Sensor Error");
    return;
  }

  bool buzz = buzzControl((uint8_t)humidity);
  uint8_t fanSpeed = dcFanControl((uint8_t)temperature);

  lcd.clear(); 

  lcd.setCursor(0, 0);
  if(display_state){
    lcd.print("Temp: "); lcd.print(temperature); lcd.print(" "); lcd.print((char)223); lcd.print("C");
    
    lcd.setCursor(0, 1);
    lcd.print("Humi: "); lcd.print(humidity); lcd.print(" %");
  }
  else{
    lcd.print("Fan speed: "); lcd.print(fanSpeed); lcd.print(" %");
    
    lcd.setCursor(0, 1);
    lcd.print("Buzz: "); lcd.print(buzz ? "On" : "Off");
  } 
}

/**
 * @brief Enters debug mode, displays debug information from the DHT22 sensor
 */
void debugMode() {
  lcd.clear();
  
  lcd.setCursor(0, 0);
  lcd.print("DHT22 Debug mode");
  
  lcd.setCursor(0, 1); 
  lcd.print("Serial: "); lcd.print(DEBUG_SERIAL_BAUDRATE); lcd.print(" b/s");

  Serial.println(dht.debug());
}

/**
 * @brief Executes actions based on the current state
 */
void action() {
  switch(action_state){
    case NORMAL_DISPLAY:
      (debug_enable ? debugMode() : display());
      break;

    case DEBUG_MODE:
      (debug_enable ? Serial.begin(DEBUG_SERIAL_BAUDRATE) : Serial.end());
      digitalWrite(BUZZ, HIGH);
      analogWrite(PWM, 0);
      action_state = NORMAL_DISPLAY;
      action();
      break;
    
    default:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Testiranje");
      
      lcd.setCursor(0, 1);
      lcd.print(action_state == FAN_TEST ? "dc ventilatora" : "Zujalice");
      
      digitalWrite(BUZZ, action_state == BUZZER_TEST ? LOW : HIGH);
      analogWrite(PWM, action_state == FAN_TEST ? 255 : 0);
      break;
  }
}

/**
 * @brief Handles keypad events
 * 
 * This function is called whenever a keypad key is pressed. It determines the action
 * to be taken based on the pressed key.
 * 
 * @param key The pressed key
 */
void keypadEvent(char key) {
  if (keypad.getState() == PRESSED) {
    switch (key) {
      case '1': action_state = DEBUG_MODE; debug_enable = false; display_state = true; break;
      case '2': action_state = DEBUG_MODE; debug_enable = false; display_state = false; break;
      case '3': action_state = (action_state == NORMAL_DISPLAY || action_state == BUZZER_TEST) ? FAN_TEST : NORMAL_DISPLAY; break;
      case '4': action_state = (action_state == NORMAL_DISPLAY || action_state == FAN_TEST) ? BUZZER_TEST : NORMAL_DISPLAY; break;
      case '5': debug_enable = !debug_enable; action_state = DEBUG_MODE; break;
      case '6': action_state = DEBUG_MODE; debug_enable = false; set_parameters(); break;
      case '8': lcd.clear(); return;
      default: return;
    }
    action();
  }
}

/**
 * @brief Allows setting custom parameters for humidity and temperature limits
 */
void set_parameters() {
  analogWrite(PWM, 0);
  digitalWrite(BUZZ, HIGH);
  
  while(true) {
    char key_parameters = keypad.getKey();

    if (keypad.getState() == PRESSED) {
      lcd.clear();
      
      lcd.setCursor(2, 0);
      lcd.print(humidity_min);
      
      lcd.setCursor(7, 0);
      lcd.print(humidity_max);
      
      lcd.setCursor(12, 0);
      lcd.print(temperature_min);
      
      switch (key_parameters) {
        case '1': humidity_min = (humidity_min < HUMIDITY_MIN_UPPER_LIMIT) ? humidity_min + HUMIDITY_STEP : humidity_min; break;
        case '5': humidity_min = (humidity_min > HUMIDITY_MIN_LOWER_LIMIT) ? humidity_min - HUMIDITY_STEP : humidity_min; break;
        
        case '2': humidity_max = (humidity_max< HUMIDITY_MAX_UPPER_LIMIT) ? humidity_max + HUMIDITY_STEP : humidity_max; break;
        case '6': humidity_max = (humidity_max> HUMIDITY_MAX_LOWER_LIMIT) ? humidity_max - HUMIDITY_STEP : humidity_max; break;

        case '3': temperature_min = (temperature_min < TEMPERATURE_MIN_UPPER_LIMIT) ? temperature_min + TEMPERATURE_STEP : temperature_min; break;
        case '7': temperature_min = (temperature_min > TEMPERATURE_MIN_LOWER_LIMIT) ? temperature_min - TEMPERATURE_STEP : temperature_min; break;
        
        case '4': humidity_min = DEFAULT_HUMIDITY_MIN; humidity_max= DEFAULT_HUMIDITY_MAX; temperature_min = DEFAULT_TEMPERATURE_MIN; break;
        case '8': return;
      }
    }
  }
}

/**
 * @brief Setup function, initializes the system
 */
void setup() {
  keypad.setDebounceTime(DEBOUNCE_TIME);

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
  
  delay(BUZZER_DELAY); 
  digitalWrite(BUZZ, HIGH);
  delay(BUZZER_DELAY);

  analogWrite(PWM, 0);

  delay(1000);
}

/**
 * @brief Loop function, executes the main program loop
 */
void loop() {
  if(millis() > en_mill){
    action();
    
    en_mill = millis() + DHT_UPDATE_INTERVAL;
  }
  
  if(char key = keypad.getKey()) {
    if (key == NO_KEY) { /*Handle no key press*/ }
    else { keypadEvent(key); }
  }
}
