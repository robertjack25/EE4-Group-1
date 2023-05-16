#include <Arduino.h>
//#include <Serial.h>
#include <Wire.h>
#include <AccelStepper.h>

#define I2C0_SLAVE_ADDR 55
#define INT_PIN A2

const char header_mv = 61;                    // 'a'
const char header_ok = 76;                    // 'v'

const int enablePin_x = 18;
const int enablePin_y = 21;
const int dirPin_x = 16;
const int stepPin_x = 17;
const int dirPin_y = 19;
const int stepPin_y = 20;

char header;
float x;
float y;
float x_steps;
float y_steps;

int prev_x1;
int prev_y1;

AccelStepper stepper_x = AccelStepper(1, stepPin_x, dirPin_x);
AccelStepper stepper_y = AccelStepper(1, stepPin_y, dirPin_y);

int Displacement_Step_Converter(int displacement_um) //function returns the step value after converting um input displacement
{
    float pinion_circumference = 37800; //this is in um //this is been calculated by hand (C = pi*D)
    int step_resolution = round(pinion_circumference/3200);
    int StepValue = round(displacement_um/step_resolution);
    return StepValue;
  }

void setup() {
  // put your setup code here, to run once:
  stepper_x.setCurrentPosition(0);
  stepper_x.setMaxSpeed(5000);              // stepper max speed of steps per second
  stepper_x.setSpeed(1000); 
  stepper_x.setAcceleration(200); 
  stepper_y.setCurrentPosition(0);
  stepper_y.setMaxSpeed(5000);              // stepper max speed of steps per second
  stepper_y.setSpeed(1000); 
  stepper_y.setAcceleration(200); 

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INT_PIN, OUTPUT);
  pinMode(stepPin_x, OUTPUT);
	pinMode(dirPin_x, OUTPUT);
  pinMode(stepPin_y, OUTPUT);
	pinMode(dirPin_y, OUTPUT);

  digitalWrite(enablePin_x,HIGH);
  digitalWrite(enablePin_y,HIGH);

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  //while (!Serial);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("T5 XY CONTROLLER");

  Serial.println("initialising I2C0 (Wire)...");
  Serial.println("mode:\tslave");
  Serial.print("addr:\t");
  Serial.print(I2C0_SLAVE_ADDR);
  Wire.begin(I2C0_SLAVE_ADDR);
  Serial.print("I2C0 initialised");
  Wire.onReceive(receiveEvent);           // register event
  Wire.onRequest(requestEvent);
  Serial.println(" and I2C0 callback registered");

}

void requestEvent()
  {
    if ((stepper_y.currentPosition() == prev_y1)) 
      {
        Wire.write(1); //sending number 1 once stepper.currentPosition 
      }
    else
      {
        //Wire.write(0);
      } 
  }

void receiveEvent(int howMany) // howMANY is always equal to no. of bytes received
  {                             //an int is 4 bytes, therefore taking up 4 spaces in the array                                    
    Serial.println("Transmission from master device detected!");
    Serial.print("Reading message...\t");
    char buf[howMany];              //creating a character storage buffer
    Serial.print(howMany);
    Serial.println(" bytes");       //no. of received bytes along i2c bus
    for (int i=0; i<=howMany; i++)    // loop through all bytes received
      {
        buf[i] = Wire.read();           // append byte to buffer
      }
    Serial.print("Message received:\n");

    char header = buf[0];           //only element 0 as a char is 1 byte
    Serial.print("header:\t");
    Serial.println(header);

    int x1;                          
    memcpy(&x1, &buf[1], sizeof(x1));    //decoding the character to an int, from elements 1-4 of the buffer, since int=4 bytes
    Serial.print("z-coord:\t");
    Serial.println(x1);
    x1 = Displacement_Step_Converter(x1);
    Serial.print("Motor Steps:\t"); 
    Serial.println(x1);

    int y1;                          
    memcpy(&y1, &buf[5], sizeof(y1));    //decoding the character to an int, from elements 1-4 of the buffer, since int=4 bytes
    Serial.print("z-coord:\t");
    Serial.println(y1);
    y1 = Displacement_Step_Converter(y1);
    Serial.print("Motor Steps:\t"); 
    Serial.println(y1);
    //This set of if/else if statement is for typical movement
    Serial.println("Wait and watch me move dad!!");
    delay(1000);
    if ((x1 >= 0)&&(x1 < Displacement_Step_Converter(150000))) //z needs to be within range
      {
        if((stepper_x.currentPosition() != x1))    //move down from origin when not at position and not triggered failsafe
          {
              // Set the target position:
              digitalWrite(enablePin_x, LOW);
              digitalWrite(enablePin_y, HIGH);
              stepper_x.moveTo(x1);
              // Run to target position with set speed and acceleration/deceleration:
              stepper_x.runToPosition();
          }
        prev_x1 = x1;   
      }
    else
      {
        Serial.println("Out of range");
       // digitalWrite(LED_OutofRange,HIGH); 
        delay(1000);
       // digitalWrite(LED_OutofRange,LOW); 
      }


    if ((y1 >= 0)&&(y1 < Displacement_Step_Converter(150000))) //z needs to be within range
      {
        if((stepper_y.currentPosition() != y1))    //move down from origin when not at position and not triggered failsafe
          {
              // Set the target position:
              digitalWrite(enablePin_x, HIGH);
              digitalWrite(enablePin_y, LOW);
              stepper_y.moveTo(y1);
              // Run to target position with set speed and acceleration/deceleration:
              stepper_y.runToPosition();
          }
        prev_y1 = y1;   
      }
    else
      {
        Serial.println("Out of range");
      //  digitalWrite(LED_OutofRange,HIGH); 
        delay(1000);
      //  digitalWrite(LED_OutofRange,LOW); 
      }

    Serial.println("Current Coordinate finished"); 
    delay(100); 
  }


void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1);
  digitalWrite(LED_BUILTIN, LOW);
  delay(24);
}