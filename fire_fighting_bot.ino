#include<Servo.h>
#include<TinyGPS.h>
#include<SoftwareSerial.h>

#define m11 12
#define m12 11
#define m21 10
#define m22 9
#define led 13
#define pir 2
Servo myservo;



float lat = 28.5458,lon = 77.1703; // create variable for latitude and longitude object 
SoftwareSerial gpsSerial(3,4);//rx,tx
SoftwareSerial BLU(0,1);
char data;
TinyGPS gps; // create gps object

int pos=0;
int isflamepin1=7;
int isflamepin2=6;
int isflame1=HIGH;
int isflame2=HIGH;
const int trigger=A4;
const int echo=A3;
int vcc=A5;
int gnd=A2;
long Time;
float distanceCM;
float resultCM;


void forward()
{
  digitalWrite(m11,HIGH);
  digitalWrite(m12,LOW);
  digitalWrite(m21,LOW);
  digitalWrite(m22,HIGH);
}
void left
()
{
  digitalWrite(m11,HIGH);
  digitalWrite(m12,LOW);
  digitalWrite(m21,LOW);
  digitalWrite(m22,LOW);
  
}

void backward()
{
  digitalWrite(m11,LOW);
  digitalWrite(m12,HIGH);
  digitalWrite(m21,HIGH);
  digitalWrite(m22,LOW);
}

void right()
{
  digitalWrite(m11,LOW);
  digitalWrite(m12,LOW);
  digitalWrite(m21,LOW);
  digitalWrite(m22,HIGH);
}

void halt()
{
  digitalWrite(m11,LOW);
  digitalWrite(m12,LOW);
  digitalWrite(m21,LOW);
  digitalWrite(m22,LOW);
  
}

void rotate(){
  right();
  delay(50);
  halt;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  gpsSerial.begin(9600); // connect gps sensor
  BLU.begin(9600);

  myservo.attach(5);
  pinMode(m11,OUTPUT); 
  pinMode(m12,OUTPUT);
  pinMode(m21,OUTPUT);  
  pinMode(m22,OUTPUT);  
  pinMode(isflamepin1,INPUT);
  pinMode(isflamepin2,INPUT);
  pinMode (echo,INPUT);
  pinMode(trigger,OUTPUT);
  pinMode(vcc,OUTPUT);
  pinMode(gnd,OUTPUT);
  pinMode(pir, INPUT); //Pin 2 as INPUT 

}

int is_obstacle(){
    digitalWrite(trigger,LOW);
    delay(10);
    digitalWrite(trigger,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger,LOW);
    Time=pulseIn(echo,HIGH);
    distanceCM=Time*0.034;
    resultCM=distanceCM/2;
    //Serial.print("Distance in cm: ");
    //Serial.print(resultCM);
    if(resultCM < 6){
      return 1;
    }
    else{
      return 0;
    }
}

void servo(){
  for (pos = 0; pos <= 360; pos += 1) {
    myservo.write(pos);              
    //delay(15);
  }
}
 


void loop() {
   digitalWrite(vcc,HIGH);
   digitalWrite(gnd,LOW);
   isflame1=digitalRead(isflamepin1);
   isflame2=digitalRead(isflamepin2);
   if(is_obstacle()==1){
    halt();
    delay(5000);
    right();
    delay(500);
   }
   if(isflame1==LOW) {
        right();
        delay(1000);
        forward();
        delay(3000);
        halt();
        delay(2000);
        servo();
        delay(1000);
    }
    else if(isflame2==LOW) {
        left();
        delay(1000);
        forward();
        delay(3000);
        halt();
        delay(2000);
        servo();
        delay(1000);
    }
    else{
        right();
        delay(2000);
        halt();
    }
   if (digitalRead(2) == HIGH){
      String latitude, longitude;
      Serial.println("Human Detected");
      digitalWrite(led,HIGH);
      if(gpsSerial.available()){ // check for gps data
        if(gps.encode(gpsSerial.read())){ 
          gps.f_get_position(&lat,&lon); // get latitude and longitude
        }
        latitude = String(lat,6);
        longitude = String(lon,6);
        Serial.println(latitude+";"+longitude);
        delay(1000);
      }
      if(BLU.available()){
       //Serial.println("bluetooth");
       Serial.println("12.9345 N 77.54345 E");       
      } 
   }
}

