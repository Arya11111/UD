/*
  UD Disk read/write
 This example shows how to read and write data to and from an UD disk file
 */
#include <UD.h>

UDFile myFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!SerialUSB) {
    ; // wait for serial port to connect. Needed for native USB port only
   SerialUSB.println(1);
  }

  Serial.print("Initializing UD disk...");
  if (!UD.begin()) {
    SerialUSB.println("initialization failed!");
    while (1);
  }
  SerialUSB.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = UD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    SerialUSB.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    SerialUSB.println("done.");
  } else {
    // if the file didn't open, print an error:
    SerialUSB.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = UD.open("test.txt");
  if (myFile) {
    Serial.print("test.txt:");
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      SerialUSB.print(myFile.read());
    }
	SerialUSB.println();
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    SerialUSB.println("error opening test.txt");
  }
}

void loop() {
  // nothing happens after setup
}


