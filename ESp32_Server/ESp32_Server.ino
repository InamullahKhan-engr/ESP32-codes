//ESP32 use as server to access it on web Page using Same WiFi

#include <WiFi.h>                 //ESP32 WiFi Library
#include <Wire.h>                 //I2C Library For ESp32
#include <Adafruit_MLX90614.h>    //Temperature Sensor Library
#include <MAX30105.h>            //heart Beat  sensor library
#include <heartRate.h>           // Heart beat Measuring library

 //ESP32 Pin GPIO22 used for SCL
 //ESP32 pin GPIO21 used for SDA
 
// ADXL I2C address
 #define Addr 0x53

//Interrupt Pin
 const uint8_t INT1 = 19; //D1
int FlagFallD=0;


//Replace with your network Credentials
const char* ssid ="vivo Y15C";
const char* password ="qwert12345";

// Set Web server port number to 80
WiFiServer server(80);
//Object created for Temeperature sensor library
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
//Object created for Heart Beat Sensor
MAX30105 particleSensor;


//Variable to Store the HTTP request
String Header;

//Auxiliar variable to store the current output state

String myString="30";


const byte RATE_SIZE = 4;                //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];                  //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;                     //Time at which the last beat occurred
float beatsPerMinute, xAccl, yAccl, zAccl;;                  //measured valued of Heart beat per minute veriable
int beatAvg;                           //Veriable for Measuring Average of heat beat


void setup() {
  
  Serial.begin(115200);
  //initialize the mlx object of temperature sensor library Adafruit_MLX90614
  mlx.begin(); 
  //Attaching Interrupet function
  attachInterrupt(digitalPinToInterrupt(INT1), ISR, RISING);

  //connect to Wi-Fi network with SSID and Password
  Serial.print("Connect to ");
  Serial.println(ssid);
  WiFi.begin(ssid,password);
  while (WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
    }
    
    //print local IP Address and start webserver
    Serial.println(" ");
    Serial.println("WiFi connected.");
    Serial.println("IP Address: ");
    Serial.println(WiFi.localIP());
    
    server.begin();

   particleSensor.setup(); //Configure sensor with default settings
   particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
   particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop() {
 
  //Heart beat function called here
  HeartBeat();
  WiFiClient client = server.available();//Listen a new client connection

    if(client){
      Serial.println("New client");
      String currentLine="";
      while (client.connected()){
          if(client.available()){
          char c=client.read();
          Serial.write(c);
          Header +=c;
          if(c=='\n'){
              if(currentLine.length()==0){
                client.println("HTTP/1.1 200 ok");
                client.println("Content-type:text/html");
                client.println("Connection: close");
                client.println();
                
                // Display the HTML web page
                client.println("<!DOCTYPE html><html>");
                client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
                client.println("<link rel=\"icon\" href=\"data:,\">");

                 // Web Page Heading
                client.println("<body><h1>ESP32 Web Server</h1>");
                 // Display current state, and ON/OFF buttons for GPIO 26
                // myString=String(mlx.readAmbientTempC());  
                client.println("<p>Ambient temperature in degree: "  + myString+ "</p>") ;
                // myString=String(mlx.readObjectTempC()); 
                client.println("<p>Object temperature in degree:  "  + myString+ "</p>");
                myString=String(beatAvg);
                client.println("<p>heart Beat " + myString + "</p>");
                myString=String(FlagFallD);
                client.println("<p>Fall  " + myString + "</p>");
                client.println("</body></html>");
                 // The HTTP response ends with another blank line
                client.println();
                // Break out of the while loop
               break;
              }
              else {
              // if you got a newline, then clear currentLine
                currentLine = "";
              }
          }
          else if (c != '\r') {
          // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
       }
       }
    }
    // Clear the header variable
    Header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}



// function for heart Beat Measuring from Sensor
void HeartBeat(){
    long irValue = particleSensor.getIR();

   if (checkForBeat(irValue) == true)
     {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
         rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
         rateSpot %= RATE_SIZE; //Wrap variable

         //Take average of readings
         beatAvg = 0;
         for (byte x = 0 ; x < RATE_SIZE ; x++)
             beatAvg += rates[x];
             beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
     Serial.print(" No finger?");

     Serial.println();
  }

  /*ICACHE_RAM_ATTR*/ void ISR() {
  Serial.println("FALL DETECTED!!!");
  FlagFallD=FlagFallD+1;
  Serial.print("Fall Detected:");
  Serial.print(FlagFallD);
}

void freefalldetectionSetup()
{
    // FreeFall Detection
  Wire.beginTransmission(Addr);
  // Select freefall threshold register
  Wire.write(0x28);
  // 600mg
  Wire.write(0x09);
  // Select freefall time register
  Wire.write(0x29);
  // 30ms
  Wire.write(0x14);
  // Select Absolute inactive register
  //Wire.write(0x27);
  // enable
  //Wire.write(0x04);  
  // Select INT1 register
  Wire.write(0x2E);
  // Enable interrupt
  Wire.write(0x04);
  //Resolution
  Wire.write(0x2F);
  // Enable interrupt
  Wire.write(0x00);
  //PowerSaving
  Wire.write(0x2D);
  // not power saving
  Wire.write(0x08);
  // Stop I2C transmission
  Wire.endTransmission(); 
}
void handleroot()
{
  unsigned int data[6];

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select bandwidth rate register
  Wire.write(0x2C);
  // Normal mode, Output data rate = 100 Hz
  Wire.write(0x0A);
  // Stop I2C transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select power control register
  Wire.write(0x2D);
  // Auto-sleep disable
  Wire.write(0x0A);
  // Stop I2C transmission
  Wire.endTransmission(); 
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select data format register
  Wire.write(0x31);
  // Self test disabled, 4-wire interface, Full resolution, Range = +/-2g
  Wire.write(0x08);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);

  for (int i = 0; i < 6; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((50 + i));
    // Stop I2C transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);

    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }

  // Convert the data to 10-bits
  int xAccl = (((data[1] & 0x03) * 256) + data[0]);
  if (xAccl > 511)
  {
    xAccl -= 1024;
  }
  int yAccl = (((data[3] & 0x03) * 256) + data[2]);
  if (yAccl > 511)
  {
    yAccl -= 1024;
  }
  int zAccl = (((data[5] & 0x03) * 256) + data[4]);
  if (zAccl > 511)
  {
    zAccl -= 1024;
  }

  // Output data to serial monitor
  Serial.print("Acceleration in X-Axis : ");
  Serial.println(xAccl);
  Serial.print("Acceleration in Y-Axis : ");
  Serial.println(yAccl);
  Serial.print("Acceleration in Z-Axis : ");
  Serial.println(zAccl);
  delay(300);

}
