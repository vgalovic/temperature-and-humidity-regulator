#include <DHT22.h>
#include <stdint.h>

#define DHT 12
#define DELAY_TIME 2000
#define HUMIDITY_LIMIT 100
#define TEMPERATURE_LIMIT 65
#define TIME_LIMIT 40 //180

uint64_t en = 0;
uint16_t time = 0;
float humidity = 0, temperature = 0;

DHT22 dht(DHT);

void getData(){
  uint64_t rawdata = dht.getRawData();

  if(dht.getLastError() != 0){
    time = TIME_LIMIT +10;
    Serial.println("DHT22 error"); 
    return;
  }
  
  uint16_t data = rawdata >> 24;
  humidity = data/10.0;

  data = rawdata >> 8;
  temperature = (bitRead(data,15)) ? (data & 0x7FFF)/-10.0 : data/10.0;

  time += DELAY_TIME/1000;

//  Serial.print(humidity); 
//  Serial.print(" "); 
  Serial.print(temperature); 
  Serial.print(" "); 
  Serial.println(time);  
}

void setup() {
  pinMode(DHT, INPUT);

  Serial.begin(9600);

  while(!Serial);

//  Serial.print("Humidity");
//  Serial.print(" ");
  Serial.print("Temperature");
  Serial.print(" ");
  Serial.println("Time");
}

void loop() {
  if(millis() > en){
//   if(time < TIME_LIMIT){
        if(temperature < TEMPERATURE_LIMIT){ //if(humidity < HUMIDITY_LIMIT){
            getData();
            en = millis() + DELAY_TIME;
        }
//     }
  }
}
