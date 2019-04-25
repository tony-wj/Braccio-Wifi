/*
  Gyro+Wifi Controlled Braccio Robot
  created 11 Apr 2019
  by Jian Wu

  This program controls Braccio robot using Arduino UNO Wifi Rev2
  UNO Wifi Rev2 has built-in gyroscope module and wifi module. 
  Connect device to AP and move the robot using REST-like control 
*/
#define MAX_POS 20 // Max num of critical points
#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h" 

#include <SparkFunLSM6DS3.h>
#include <Wire.h>
#include <Braccio.h>
#include <Servo.h>

//servo motors
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

//Gyro
LSM6DS3 myIMU(SPI_MODE, SPIIMU_SS);

////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int pos[6][MAX_POS];
int posLoc = -1;
          
int status = WL_IDLE_STATUS;
WiFiServer server(80);

double ax, ay, az;              //define the x y z acceleration 
double gx, gy, gz;              //define the x y z speed
double gx0, gy0, gz0;           //define the zero offset
double sum_gz_drift, gz_drift;  //define the gz drift value
double accXangle;               //define the x angle
double accYangle;               //define the y angle

double gyroXangle = 90;        //initial x angle
double gyroYangle = 90;        //initial y angle
double gyroZangle = 90;        //initial z angle

int m1, m2, m3, m4, m5, m6;

uint32_t timer;
uint32_t braccio_timer;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  Serial.println("Access Point Web Server");

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address of will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();


  // start Braccio
  Braccio.begin();
  
  Serial.println("Braccio Begins\n");
  
  //IMU begins
  myIMU.begin();

  //get the offset
  gx0 = (int) myIMU.readFloatGyroX();
  gy0 = (int) myIMU.readFloatGyroY();
  gz0 = (int) myIMU.readFloatGyroZ();

  //get the drfit
  timer = micros();
  for (int i=0;i<1000;i++){
    sum_gz_drift +=  myIMU.readFloatGyroZ()-gz0;
    delay(1);
  }
  gz_drift = sum_gz_drift/(micros()-timer);

  //set timer
  timer = micros();
  braccio_timer = micros();
}


void loop() {
  //Get all parameters using IMU
  ax =  myIMU.readFloatAccelX();
  ay =  myIMU.readFloatAccelY();
  az =  myIMU.readFloatAccelZ();
  gx =  myIMU.readFloatGyroX()-gx0;
  gy =  myIMU.readFloatGyroY()-gy0;
  gz =  myIMU.readFloatGyroZ()-gz0;
  
  accXangle = (atan2(ay,az) + PI) * RAD_TO_DEG;
  accYangle = (atan2(ax,az) + PI) * RAD_TO_DEG;


  double gyroXrate = -((double)gx / 50.0);
  double gyroYrate = -((double)gy / 50.0);
  double gyroZrate = ((double)gz / 50.0);
  
  //comprehensive filter, you can adjust the factor 0.9 and 0.1
  gyroXangle = 0.9*(gyroXangle+gyroXrate*((double)(micros()-timer)/10000))+0.1*accXangle; 
  gyroYangle = 0.9*(gyroYangle-gyroYrate*((double)(micros()-timer)/10000))+0.1*(360-accYangle); 
  gyroZangle = gyroZangle-(gyroZrate-gz_drift / 50.0)*(double)(micros()-timer)/10000; //not using filter because my accZangle is not accurate 

  
  m1 = (int) gyroZangle;
  m2 = (int) gyroYangle;
  m3 = (int) (gyroYangle)/2;
  m4 = (int) gyroYangle;
  m5 = (int) gyroXangle;
  
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("<h1>Gripper</h1>");
            client.print("<h1>Click <a href=\"/H\">here</a> turn the Gripper on</h1><br>");
            client.print("<h1>Click <a href=\"/L\">here</a> turn the Gripper off</h1><br>");
            client.print("<h1>Path</h1>");
            client.print("<h1>Click <a href=\"/S\">here</a> to save critical position</h1><br>");
            client.print("<h1>Click <a href=\"/D\">here</a> to start executing path</h1><br>");
            client.print("<h1>Click <a href=\"/R\">here</a> to reset path</h1><br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          m6=10;
        }
        if (currentLine.endsWith("GET /L")) {
          m6=60;
        }
        if (currentLine.endsWith("GET /S")) { 
          braccioSavePoint();
        }
        if (currentLine.endsWith("GET /D")) {
          braccioPath();
        }
        if (currentLine.endsWith("GET /R")) {
          braccioReset();
        }
      }
    }
    // close the connection:
    client.stop();
  }

  Braccio.ServoMovement(30, m1, m2, m3, m4, m5, m6);
  
  timer = micros();
  
  delay(30);
}

void braccioPath(){
  for(int i=0;i<posLoc+1;i++){
    Braccio.ServoMovement(30, pos[0][i], pos[1][i], pos[2][i], pos[3][i], pos[4][i], pos[5][i]);
    delay(1000);
  }
}
void braccioSavePoint(){
  if(posLoc == MAX_POS){
  }else{
     posLoc++;
     pos[0][posLoc] = m1;
     pos[1][posLoc] = m2;
     pos[2][posLoc] = m3;
     pos[3][posLoc] = m4;
     pos[4][posLoc] = m5;
     pos[5][posLoc] = m6;
  }
}
void braccioReset(){
  posLoc = -1;
  for(int i=0;i<MAX_POS;i++){
    for(int j=0;j<6;j++){
      pos[j][i] = 0;
    }
  }
}
