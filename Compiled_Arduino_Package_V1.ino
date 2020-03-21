//Right Sensor  
int trigPin1=2; 
int echoPin1=3;

//Left Sensor
int trigPin2=4;
int echoPin2=5;

//Front Sensor
int trigPin3=6;
int echoPin3=7;

//Filter Initialization
int R1=0; //Right Ultrasonic Sensor Initialization
int Alpha=0.5; //Alpha Value (Modify this value to adjust the stability of the filter)
int prevDistanceRight=0;
int distanceRight=0;
int prevDistanceLeft=0;
int distanceLeft=0;
int prevDistanceFront=0;
int distanceFront=0;

#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;

void setup() {
  Serial.begin (9600);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

//IMU
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();
    if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));
}


void loop() {
//IMU
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (quatReal * quatK + quatI * quatJ);
    float cosy_cosp = 1 - 2 * (quatJ * quatJ + quatK * quatK);
    float angles_yaw = atan2(siny_cosp, cosy_cosp); 

    //Serial.println(angles_yaw, 2); //This line was moved to the print section at the end of this code



//Right Sensor
  long duration1, distance1;
  digitalWrite(trigPin1, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1/2) / 29.1;

   if (distance1 >= 200 || distance1 <= 0){
  /*  Serial.print ( "Right Sensor  ");
    Serial.print ( prevDistanceRight);
    Serial.println("cm");
    */
    prevDistanceRight=distanceRight;
  }
  else { 
    distanceRight=Alpha*prevDistanceRight+(1-Alpha)*distance1;
    /*
    Serial.print ( "Right Sensor  ");
    Serial.print ( distanceRight);
    Serial.println("cm");
    */
    prevDistanceRight=distanceRight;
  }

//Left Sensor
long duration2, distance2;
  digitalWrite(trigPin2, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2= (duration2/2) / 29.1;

  if (distance2 >= 200 || distance2 <= 0){
    /*Serial.print ( "Right Sensor  ");
    Serial.print ( prevDistanceLeft);
    Serial.println("cm");
    */
    prevDistanceLeft=distanceLeft;
  }
  else {
    distanceLeft=Alpha*prevDistanceLeft+(1-Alpha)*distance2;
    /*Serial.print("Left Sensor  ");
    Serial.print(distanceLeft);
    Serial.println("cm");
    */
    prevDistanceLeft=distanceLeft;
  }
//  delay(250);


//Front Sensor
long duration3, distance3;
  digitalWrite(trigPin3, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distance3= (duration3/2) / 29.1;

  if (distance3 >= 500 || distance3 <= 0){
    /*Serial.print ( "Front Sensor  ");
    Serial.print ( prevDistanceFront);
    Serial.println("cm");
    */
    prevDistanceFront=distanceFront;
    Serial.print(distanceRight);
    Serial.print(",");
    Serial.print(distanceLeft);
    Serial.print(",");
    Serial.print(distanceFront);
    Serial.print(",");
    Serial.print(angles_yaw, 2);
    Serial.println();
  }
  else {
    distanceFront=Alpha*prevDistanceLeft+(1-Alpha)*distance3;
    /*Serial.print("Front Sensor  ");
    Serial.print(distanceFront);
    Serial.println("cm");
    */
    Serial.print(distanceRight);
    Serial.print(",");
    Serial.print(distanceLeft);
    Serial.print(",");
    Serial.print(distanceFront);
    Serial.print(",");
    Serial.print(angles_yaw, 2);
    Serial.println();
    prevDistanceFront=distanceFront;
  }
  }  }
  //delay(250);
