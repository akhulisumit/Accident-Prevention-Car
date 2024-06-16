#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 53
#define RST_PIN 9

MFRC522 rfid(SS_PIN, RST_PIN);

// Define motor control pins
const int sw = 9;
int motor1Pin1 = 2;
int motor1Pin2 = 3;
int motor2Pin1 = 4;
int rfno = 1;
int motor2Pin2 = 5;
int carState = 0;
const int irSensorPin = A1;

const int xAxisPin = A8;
const int yAxisPin = A9;

// Define motor speed control pins
int enableMotor1 = 6;
int enableMotor2 = 7;

// Define ultrasonic sensor pins
int trigPin = 12;
int echoPin = 11;

// Define alcohol sensor pins
int alcoholSensorPin = A0;
int alcoholSensorThreshold = 300; // Adjust the threshold as needed

// Speed levels for RFID tags
int speedLevel1 = 72;  // Adjust these values based on your needs
int speedLevel2 = 150;

void decreaseSpeed() {
  analogWrite(motor1Pin1, 200);  // Adjust the speed as needed
  analogWrite(motor1Pin2, LOW);
  analogWrite(motor2Pin1, 200);  // Adjust the speed as needed
  analogWrite(motor2Pin2, LOW);
}

void increaseSpeed() {
  analogWrite(motor1Pin1, 255);  // Full speed
  analogWrite(motor1Pin2, LOW);
  analogWrite(motor2Pin1, 255);  // Full speed
  analogWrite(motor2Pin2, LOW);
}

void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  }

  void moveBackward() {
  digitalWrite(motor1Pin1, 0);
  digitalWrite(motor1Pin2, 1);
  digitalWrite(motor2Pin1, 0);
  digitalWrite(motor2Pin2, 1);
  }

void stopVehicle() {
    analogWrite(motor1Pin1,0);  // Adjust the speed as needed
  analogWrite(motor1Pin2, LOW);
  analogWrite(motor2Pin1, 0);  // Adjust the speed as needed
  analogWrite(motor2Pin2, LOW);
  delay(3000);
  }


void setup() {
  pinMode(sw, INPUT_PULLUP);
  pinMode(8, OUTPUT);
  // Set motor control pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Set motor speed control pins as outputs
  pinMode(enableMotor1, OUTPUT);
  pinMode(enableMotor2, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set alcohol sensor pin
  pinMode(alcoholSensorPin, INPUT);

  //   For RFID
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();
}

void loop() {
  // Measure distance using ultrasonic sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1; // Convert the time to distance in cm

  // Measure alcohol concentration
  int alcoholValue = analogRead(alcoholSensorPin);

   int sensorValue = analogRead(irSensorPin);
  
  // Using the sensor's datasheet, convert sensor value to distance in centimeters
  long distancee = 2076.0 / (sensorValue - 11.0);

  Serial.print("Distance: ");
  Serial.println(distance);
  Serial.print ("alcohol: ");
  Serial.println (alcoholValue);
  Serial.print("IRDist: ");
  Serial.println(distancee);


  // Print distance and alcohol level to serial monitor


  //Adjust the distance to stop and the alcohol level to start the vehicle ssp
  if (distance > 20 & alcoholValue < 1000 ) {
    
    Serial.println("Button"+String(digitalRead(sw)));
    /*if(digitalRead(sw) == 1){
    moveForward();
    delay(1000);
    }*/
    int xAxisValue = analogRead(xAxisPin);
  int yAxisValue = analogRead(yAxisPin);
  Serial.println("X:"+String(xAxisValue));
   Serial.println("Y:"+String(yAxisValue));
   delay(300);
  // You may need to adjust the threshold values based on your specific joystick
  if(xAxisValue < 550 & xAxisValue > 450){
    stopVehicle();
    Serial.print("Stopping");
  }
  else if (xAxisValue > 120) {
    Serial.print("Hrituraj lau age dike");
    Serial.println("Top");
//moveForward();
  } else if(xAxisValue > 900) {
    Serial.print("Hrituraj lau pechon dike");
    Serial.println("Bottom");
   // moveBackward();
  }
  }
 else{
  Serial.print("Stopping");
    stopVehicle();
  }


  if (distancee < -100){
    decreaseSpeed();
    delay(20000);
    }else{
      increaseSpeed();
    }

  if(distance < 30){
    digitalWrite(8, HIGH);
    }
  if (distance > 30){
    digitalWrite(8,LOW);
  }


}
