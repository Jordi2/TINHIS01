/*
 *
 * Things that you need:
 * - Arduino
 * - A Sharp IR Sensor
 *
 *
 * The circuit:
 * - Arduino 5V -> Sensor's pin 1 (Vcc)
 * - Arduino GND -> Sensor's pin 2 (GND)
 * - Arduino pin A0 -> Sensor's pin 3 (Output)
 * - Arduino pin A1 -> Sensor's pin 3 (Output)
 * 
 * See the Sharp sensor datasheet for the pin reference, the pin configuration is the same for all models.
 * There is the datasheet for the model GP2Y0A41SK0F:
 * 
 * http://www.robotstore.it/open2b/var/product-files/78.pdf
 *
 */

//import the library in the sketch
#include <SharpIR.h>

//Create a new instance of the library
//Call the two sensors "sensorA" and "sensorB"
//The model of the sensors is "GP2YA41SK0F"
//The sensor output pin for sensorA and B is attached to the pin A0 and A1 respectivly
int sensorType = GP2Y0A02YK0F;//the sensor you use
SharpIR sensorA(sensorType, A0);SharpIR sensorB(sensorType, A1);

int count = 0;                                      //counter
int totalDistanceA = 0;int totalDistanceB = 0;      //sum of measurments
int averageOf = 25;                                 //take average of *averageOf* distances
int averageDistanceA = 0; int averageDistanceB = 0; //average of measurments
int range = 25;                                     //allowed distance between sensors before detecting people

int people = 1;                                     //the number of people that have passed the sensor

//variables for display en shift register
int latchPin = 8;   //Pin connected to ST_CP rck of 74HC595 
int clockPin = 12;  //Pin connected to SH_CP sck of 74HC595
int dataPin = 11;   //Pin connected to DS ser of 74HC595

byte dataArray[12];
 
int curNum = 1; //digit to display

//digits 
int dig1 = 4; 
int dig2 = 3; 
int dig3 = 2; 
int dig4 = 1; 

int del = 1;//delay time for multiple unique digits 

void setup()
{
  //set pins to output so you can control the shift register
  pinMode(latchPin, OUTPUT);//RCK
  pinMode(clockPin, OUTPUT);//SCK
  pinMode(dataPin, OUTPUT);//SER
  
  dataArray[0] = 0; //uit 
  dataArray[1] = 144; //1=b+c=16+128 
  dataArray[2] = 91; //2=abged=8+16+64+1+2=91 
  dataArray[3] = 218; //3=abcdg=8+16+128+2+64=218 
  dataArray[4] = 240; //4=fgbc=32+16+64+128=240 
  dataArray[5] = 234; //5 
  dataArray[6] = 235; //6 
  dataArray[7] = 152; //7 
  dataArray[8] = 251; //8 
  dataArray[9] = 248; //9 
  dataArray[10] = 187; // 
  
  //pins for controlling digit selection 
  pinMode(dig1, OUTPUT); 
  digitalWrite(dig1, LOW); 
  pinMode(dig2, OUTPUT); 
  digitalWrite(dig2, LOW); 
  pinMode(dig3, OUTPUT); 
  digitalWrite(dig3, LOW); 
  pinMode(dig4, OUTPUT); 
  digitalWrite(dig4, LOW); 
  
  Serial.begin(9600); //Enable the serial comunication
}

void loop()
{
  int distanceA = sensorA.getDistance(); //Calculate the distance in centimeters and store the value in a variable
  int distanceB = sensorB.getDistance(); 
  if(count < averageOf){
    //count multiple measurements together
    totalDistanceA += distanceA; totalDistanceB += distanceB;
    count++;//increment counter
  } else{
    //Calculate and print the averaged value to the serial monitor for Sensors A and B
    Serial.print("Mean distance A: " );
    averageDistanceA = IsInSensorTypeRange(totalDistanceA/averageOf);
    Serial.println(averageDistanceA);
    Serial.print("Mean distance B: " );
    averageDistanceB = IsInSensorTypeRange(totalDistanceB/averageOf);
    Serial.println(averageDistanceB);
    
    totalDistanceA = 0;totalDistanceB = 0;//reset distance
    count = 0;//reset counter
    delay(250);//give time to read monitor

    // Are people walking by the sensors?
    if(averageDistanceA < averageDistanceB-range){
      Serial.println("-------------Left-------------");
      people++;
    } 
    if(averageDistanceA > averageDistanceB+range){
      Serial.println("-------------Right-------------");
      people++;
    }
    if((averageDistanceA < averageDistanceB-range) || (averageDistanceA > averageDistanceB+range)){
      multiDigitUpdate();  
      delay(500);//this delay is to prevent the people counter from counting one person twice
    }
    Serial.println(people);
  }
  multiDigitUpdate();
  //updateDisplay();

  if(people == 10){people = 1;}
}

int IsInSensorTypeRange(int dis){
  
  //sensor type 'GP2Y0A02YK0F' has a range of 20-150 cm
  if(sensorType = GP2Y0A02YK0F){
    if(dis < 20){//if distance below range just make sensor minimum
      dis = 20;
    } else if (dis > 150){//if distance above range just make sensor maximum
      dis = 150;
    }
  }
  return dis;
}

void clearLEDs(){ 
  curNum = 0; 
  updateDisplay(); 
} 
void multiDigitUpdate(){ 
  /* for other digits if you need them
  clearLEDs(); 
  digitalWrite(dig1, LOW); 
  digitalWrite(dig2, HIGH); 
  digitalWrite(dig3, HIGH); 
 // curNum = 0; 
  updateDisplay(); 
  delay(del); 
  
  clearLEDs(); 
  digitalWrite(dig1, HIGH); 
  digitalWrite(dig2, LOW); 
  digitalWrite(dig3, HIGH); 
  //curNum = 0; 
  updateDisplay(); 
  delay(del);
  */
  clearLEDs(); 
  digitalWrite(dig1, HIGH); 
  digitalWrite(dig2, HIGH); 
  digitalWrite(dig3, LOW); 
  updateDisplay(); 
  delay(del); 
} 
void updateDisplay(){ 
  // take the latchPin low so 
  // the LEDs don't change while you're sending in bits: 
  digitalWrite(latchPin, LOW); 
  // shift out the bits: 
  shiftOut(dataPin, clockPin, MSBFIRST, dataArray[people]);
  //take the latch pin high so the LEDs will light up: 
  digitalWrite(latchPin, HIGH); 
  // pause before next value: 
  //delay(10); 
} 
