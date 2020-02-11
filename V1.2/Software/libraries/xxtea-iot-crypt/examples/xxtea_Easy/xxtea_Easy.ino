/*

  Easy to Use example of xxtea-iot-crypt Library

  This example shows the calling convention for the various functions.

  For more information about this library please visit us at
  http://github.com/boseji/xxtea-iot-crypt

  Created by Abhijit Bose (boseji) on 04/03/16.
  Copyright 2016 - Under creative commons license 3.0:
        Attribution-ShareAlike CC BY-SA

  @version API 1.2.1
  @author boseji - salearj@hotmail.com

*/

#include <xxtea-iot-crypt.h>

void setup() {
  Serial.begin(115200);

  Serial.println();

  // Text to Encrypt - ! Carefull no to more than 80 bytes ! - Or See `Limitations`
  String plaintext = F("Hi There we can work with this");

  // Set the Password
  xxtea.setKey("Hello Password");

  // Perform Encryption on the Data
  Serial.print(F(" Encrypted Data: "));
  String result = xxtea.encrypt(plaintext);
  result.toLowerCase(); // (Optional)
  Serial.println(result);

  // Perform Decryption
  Serial.print(F(" Decrypted Data: "));
  Serial.println(xxtea.decrypt(result));
}

void loop() {}
