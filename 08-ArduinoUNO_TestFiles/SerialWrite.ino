/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 2 (connect to TX of other device)
 * TX is digital pin 3 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Native USB only
  }


  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(115200);
  //mySerial.println("Hello, world?");
}

void loop() // run over and over
{
    Serial.write("S1");
    mySerial.write("S1");
    delay(500);

    Serial.write("S2");
    mySerial.write("S2");
    delay(500);

    Serial.write("S3");
    mySerial.write("S3");
    delay(500);

    Serial.write("S4");
    mySerial.write("S4");
    delay(500);


    Serial.write("R1");
    mySerial.write("R1");
    delay(500);

    Serial.write("R2");
    mySerial.write("R2");
    delay(500);

    Serial.write("R3");
    mySerial.write("R3");
    delay(500);

    Serial.write("R4");
    mySerial.write("R4");
    delay(500);
}

