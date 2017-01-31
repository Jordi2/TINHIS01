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

void setup()
{
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
    } 
    if(averageDistanceA > averageDistanceB+range){
      Serial.println("-------------Right-------------");
    }
  }
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
