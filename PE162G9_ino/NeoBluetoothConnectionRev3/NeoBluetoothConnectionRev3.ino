/**
 * code by KilnerJhow and sandieg2k
 * 
 * Credits to Hayk Martirosyan for creat Bluetooth communication using the BlueSMiRF module.
 */

#include <SoftwareSerial.h>
#include <Servo.h>

// Set to match your hardware
int RX_PIN = 8;
int TX_PIN = 9;

Servo srvMotor1; //Horizontal servo
Servo srvMotor2; //Verical servo

String data = "";//Stores the value received by bluetooth

String sensorValues[5];//Stores each value of the magnetic sensor
int sensorIndex;
int index;
boolean reading;
boolean startValue;//Used to define the initial values(Azimuth and Roll)

float srvMotor1Value;
float srvMotor2Value;


int azimuthZero = 0;//Initual value of azimuth
int rollZero = 0;//Initial value of roll
int rotateSrvMotor1 = 0;
int newAzimuth, newRoll;//New values of azimuth and roll, update each time the sensor value change

int deltaAzimuth = 0;//Difference between newAzimuth and azimuthZero
int oldDeltaAzimuth = 0;//The last deltaAzimuth value written in srvMotor1 
int deltaRoll = 0;
int oldDeltaRoll = 0;
int range = 5;//Min. variation of angle to rotate srvMotor1 ou srvMotor2
int maxRange = 50;//Max variation of angle to rotate srvMotor1 or srvMotor2 (unused in this moment)
int aux = 0;

boolean isHighAzimuth = true;//To verify if actual azimuth value is higher than the old azimuth value
boolean isHighRoll = true;//To verify if actual roll value is higher than the old roll value

String ang = "";

// Serial interface for the BlueSMiRF
SoftwareSerial bluetooth(RX_PIN, TX_PIN);//Define the RX e TX pins to receive values by bluetooth connection

// Send and receive buffers
String bluetooth_rx_buffer = "";
String bluetooth_tx_buffer = "";

// Delimiter used to separate messages
char DELIMITER = '&';//Delimiter used to separet the strings received

void setup() {
  
  // Start USB communication
  Serial.begin(19200);
  
  // Uncomment this only once, when configuring a new BlueSMiRF
  //set_bluetooth_baudrate();
  //Attach servo 1 to pin 3 and servo 2 to pin 4
  srvMotor1.attach(3);
  srvMotor2.attach(4);
  //Write 90 in servo 1 and servo 2
  srvMotor1.write(90);
  srvMotor2.write(90);
  sensorIndex = 0;
  index = 0;
  reading = true;
  startValue = true;
  delay(200);
  // Start bluetooth communication
  bluetooth.begin(19200);
  
  Serial.println("Initialized.");
}

/**
 * Called when a complete message is received.
 */
void gotMessage(String message) {
  
  Serial.println("[RECV] " + message);
  //delay(1000);
  
  // Do something!
  switch(sensorIndex){
      
        case 0: 
          sensorValues[0] = message;
          sensorIndex++;
          break;
        case 1:
          sensorValues[1] = message;
          sensorIndex++;
          break; 
        case 2:
          sensorValues[2] = message;
          //motorsRotation(sensorValues[0].toInt(), sensorValues[2].toInt());
          motorsRotation(sensorValues[0].toInt(), sensorValues[2].toInt());
          sensorIndex = 0;
          break;  
      }

    if(message.charAt(0) == '$') {
      setInitialValue(sensorValues[0],sensorValues[2]);
    }
}

/**
 * Finds complete messages from the rx buffer.
 */
void parseReadBuffer() {
  
  // Find the first delimiter in the buffer
  int inx = bluetooth_rx_buffer.indexOf(DELIMITER);
  
  // If there is none, exit
  if (inx == -1) return;
  
  // Get the complete message, minus the delimiter
  String s = bluetooth_rx_buffer.substring(0, inx);
  
  // Remove the message from the buffer
  bluetooth_rx_buffer = bluetooth_rx_buffer.substring(inx + 1);
  
  // Process the message
  gotMessage(s);
  
  // Look for more complete messages
  parseReadBuffer();
}

void parseWriteBuffer() {
  
  // Find the first delimiter in the buffer
  int inx = bluetooth_tx_buffer.indexOf(DELIMITER);
  
  // If there is none, exit
  if (inx == -1) return;
  
  // Get the complete message, including the delimiter
  String message = bluetooth_tx_buffer.substring(0, inx + 1);
  
  // Remove the message from the buffer
  bluetooth_tx_buffer = bluetooth_tx_buffer.substring(inx + 1);
  
  // Send off the message
  bluetooth.print(message);
  //Serial.print("[SENT] " + message);
  
  // Look for more
  parseWriteBuffer();
}

/**
 * Continuously sends messages sent from USB and reads in messages from
 * the Bluetooth connection.
 */
void loop() {
  
  // Forward anything received via USB to bluetooth
  if(Serial.available()) {
    
    while(Serial.available()) {
      bluetooth_tx_buffer += (char)Serial.read();
    }

    Serial.println(bluetooth_tx_buffer);
    // Look for complete messages
    parseWriteBuffer();
  }
  
  // Add bytes received over bluetooth to the buffer
  if(bluetooth.available()) {
    
    while(bluetooth.available()) {
      bluetooth_rx_buffer += (char)bluetooth.read();
    }
    // Look for complete messages
    parseReadBuffer();
  }
}

//TODO: Adaptar LowPass para o arduino
/*
float lowPass( float input[], float output[] ) {
    if ( output == null ) return input;
    for ( int i=0; i<input.length; i++ ) {
        output[i] = output[i] + ALPHA * (input[i] - output[i]);
    }
    return output;
  }*/


//Used to Set the initial values of azimuth and roll
void setInitialValue(int azimuth, int roll) {

  azimuthZero = azimuth.toInt();
  rollZero = roll.toInt();  
}

//this function defines the rotation of each servo
void motorsRotation(int azimuth, int roll){
//Set the initial values when the first message is received.
  if (startValue == true){
    
    setInitialValue(azimuth, roll);
    startValue = false;
    }else{
      
      newAzimuth = azimuth;
      newRoll = roll;
        
        Serial.print("nA: ");
        Serial.println(newAzimuth);
        Serial.print("aZ: ");
        Serial.println(azimuthZero);
        Serial.print("nR: ");
        Serial.println(newRoll);
        Serial.print("rZ: ");
        Serial.println(rollZero);
        //delay(10);
        //fix the value if the received data is negative
        if(azimuthZero > 90 && isHighAzimuth && newAzimuth < 0){

            newAzimuth = (360 + (newAzimuth));
            Serial.print("AzimuthModulo: ");
            Serial.println(newAzimuth); 
          }
        //verify if the value is growing up and is in the limit
        //calcs and write the value in the servo1
        if(newAzimuth > azimuthZero && newAzimuth < (azimuthZero + 90)){

          deltaAzimuth = newAzimuth - azimuthZero;
          Serial.print("dZ maior: ");
          Serial.println(deltaAzimuth);
          //verify if the value to write is bigger than the minimun value to write
          if(((deltaAzimuth - oldDeltaAzimuth) > range) || ((oldDeltaAzimuth - deltaAzimuth) > range)){
            
            srvMotor1.write(90 + deltaAzimuth);
            delay(100);
            oldDeltaAzimuth = deltaAzimuth;
            isHighAzimuth = true;
          }
         }
         //verify if the value is growing down and is in the limit
        //calcs and write the value in the servo1
         if(newAzimuth < azimuthZero && newAzimuth > (azimuthZero - 90)){

          deltaAzimuth = azimuthZero - newAzimuth;
          Serial.print("dZ menor: ");
          Serial.println(deltaAzimuth);
           //verify if the value to write is bigger than the minimun value to write
           if(((deltaAzimuth - oldDeltaAzimuth) > range) || ((oldDeltaAzimuth - deltaAzimuth) > range)){

              srvMotor1.write(90 - deltaAzimuth);
              //delay(50);
              oldDeltaAzimuth = deltaAzimuth;
              isHighAzimuth = false;
            }
         }

// Move Servo 2
         if(rollZero > 90 && isHighRoll && newRoll < 0){

            newRoll = (180 + (-newRoll));
            Serial.print("RowModulo: ");
            Serial.println(newRoll); 
          }
          
        if(newRoll > rollZero && newRoll < (rollZero + 90)){

          deltaRoll = newRoll - rollZero;
          Serial.print("dR maior: ");
          Serial.println(deltaRoll);
          if(((deltaRoll - oldDeltaRoll) > range) || ((oldDeltaRoll - deltaRoll) > range)){
            
            //srvMotor2.write(90 + deltaRoll);
            //delay(50);
            oldDeltaRoll = deltaRoll;
            isHighRoll = true;
          }
         }
         
         if(newRoll < rollZero && newRoll > (rollZero - 90)){

          deltaRoll = rollZero - newRoll;
          Serial.print("dR menor: ");
          Serial.println(deltaRoll);
           if(((deltaRoll - oldDeltaRoll) > range) || ((oldDeltaRoll - deltaRoll) > range)){

              //srvMotor2.write(90 - deltaRoll);
              //delay(50);
              oldDeltaAzimuth = deltaRoll;
              isHighRoll = false;
            }
         }
  }
}
