// Basic demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h> //Ihr braucht den Fake-Sensor Fix!!! https://github.com/UraniumDonut/Adafruit_MPU6050_0x72/commit/78f2b543fde344745caa9e98cfe7b9dac794c4c8
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <FastLED.h>
#define DATA_PIN 5
const int DELAY = 25;



class Vector3{
  public:
    float x;
    float y;
    float z;
    void reset(){
      x = 0;
      y = 0;
      z = 0;
    }
};

class Handschuh{ //Connected to pin 22 on black/orange and pin 21 on yellow/yellow
  private:
    Adafruit_MPU6050 mpu;
    sensors_event_t a, g, temp;
    float speed;
    Vector3 v, d, ac ,a_avg;
    int throwThreshold = 10; //Change this to make throw detection finer
    float slowdownThreshold = 5; //Make this higher on premature throws
    float speedbefore = 0;
    void getEvents(){
      mpu.getEvent(&a, &g, &temp);
      
      d.x += g.gyro.x;
      d.y += g.gyro.y;
      d.z += g.gyro.z;

      a_avg.x = a_avg.x * 0.8 + a.acceleration.x*0.2;
      a_avg.y = a_avg.y * 0.8 + a.acceleration.y*0.2;
      a_avg.z = a_avg.z * 0.8 + a.acceleration.z*0.2;

      ac.x = a_avg.x - a.acceleration.x;
      ac.y = a_avg.y - a.acceleration.y;      
      ac.z = a_avg.z - a.acceleration.z;

      v.x += ac.x - a_avg.x;
      v.y += ac.y - a_avg.y;
      v.z += ac.z - a_avg.z;

      speed = sqrt(sq(ac.x)+sq(ac.y)+sq(ac.z));
      if(speedbefore == 0){
        speedbefore = speed;
      }
    }
    
  public:
  Handschuh(){}
  void warmUp(){
      mpu.getEvent(&a, &g, &temp);
      
      d.x += g.gyro.x;
      d.y += g.gyro.y;
      d.z += g.gyro.z;

      a_avg.x = a_avg.x * 0.8 + a.acceleration.x*0.2;
      a_avg.y = a_avg.y * 0.8 + a.acceleration.y*0.2;
      a_avg.z = a_avg.z * 0.8 + a.acceleration.z*0.2;
      Serial.println(a_avg.x);
    }
  void resetAll(){
      v.reset();
      d.reset();
      ac.reset();
      a_avg.reset();
      speed = 0;
      speedbefore = 0;
    }
  void init(){
    resetAll();
    if(!mpu.begin()){
      Serial.println("Failed to find MPU6050 chip!");
      delay(1000);
      ESP.restart();
    }
    delay(10);
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    delay(10);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    delay(10);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    delay(10);
    for(int i = 0; i < 20; i++){
      warmUp();
      delay(DELAY);
    }
  }

  void printAll(){
    
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    Serial.println("");
  }

  bool getThrown(){ //False = aiming, True = throwing
    speedbefore = speed;
    getEvents();
    //Serial.print("Acceleration: ");
    //Serial.println(a.acceleration.x);
    //Serial.print("High-Pass: ");
    //Serial.println(ac.x);

    //Serial.println(speed);
    if(speedbefore > throwThreshold && (speedbefore - (speed + slowdownThreshold)) > 0){
      return true;
    }
    return false;
  }
  int getAngle(){
    if(d.x > 40){
      return 40;
    }
    if(d.x < -40){
      return -40;
    }
    return d.x;
  }
  int getStrength(){
    if(speedbefore > 100){
      return 100;
    }
    return speedbefore;
  }
};

// Zuklappen
class LEDStrip{
  private:
    CRGB leds[5];
  public:
    LEDStrip(){

    }
    void init(){
      FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, 5);
    }
    void update(int schritt){
      if(schritt < 0){
        for(int i = 0; i < 5; i++){
          leds[i] = CRGB::Red;
        }
      }
      else if(schritt > 0){
        for(int i = 0; i < 5; i++){
          leds[i] = CRGB::Yellow;
        }
      }
      else{
        for(int i = 0; i < 5; i++){
          leds[i] = CRGB::Green;
        }
      }
      FastLED.show();
    }
};

LEDStrip strip;
Handschuh handschuh;
int schritt;
// -1 = Stop
// 0 = Move!
// 1 = Wait, WarmUp
// 2 = Wait, DriftReset
int counter = 0;

void setup(void) {
  schritt = -1;
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("");
  delay(100);
  handschuh.init();
  strip.init();
  schritt = 2;
}
void loop() {
  if(schritt == 1){ //====WARMUP====
    if(counter < 20){
      handschuh.warmUp();
      counter++;
    }
    else{
      counter = 0;
      schritt = 0;
      Serial.println("Warmup finished!");
    }
  }
  else if(schritt == 2){ //====DRIFTRESET====
    //===== HIER FEHLT DIE MOTION DETECTION, ABER JOA MAN BRAUCHTS AUCH NED WIRKLICH SO KRASS
    strip.update(schritt);
    delay(5000);
    handschuh.resetAll(); 
    Serial.println("Drift was reset!");
    schritt = 1;
    counter = 0;
  }
  else if(schritt == 0){//====MOVE====
    if(handschuh.getThrown()){ //We have a throw!
      int strength = handschuh.getStrength();
      Serial.print("Throw detcted with Strength ");//servo.send(strength);
      Serial.println(strength);
      counter = 0;
      schritt = -1;
    }
    else{ //We just aimed
      int angle = handschuh.getAngle();
      Serial.println(angle);//servo.send(angle);
      counter++;
    }
    if(counter > (15000/DELAY)){
      counter = 0;
      schritt = 2;
      Serial.println("Too much drift! Reset!");
    }
  }
  else if(schritt == -1){//====STOP====
    delay(2000);
    schritt = 2;
    Serial.println("Stopped Waiting");
  }
  strip.update(schritt);
  delay(DELAY);
}