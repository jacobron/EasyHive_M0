# xxtea-iot-crypt library for Arduino #

[![Build Status](https://travis-ci.org/boseji/xxtea-iot-crypt.svg?branch=master)]("https://travis-ci.org/boseji/xxtea-iot-crypt")

## Encryption Library for IoT devices ##

This library implements both RAW data encryption as well as string encryption and decryption for XXTEA.

**Updates 18 July 2017**

 * Fixed issue reported for Signed arithmetic in Arduino Mega ADK board
 * Changed Doc format to Markdown

**Updates 11 Jan 2017**

 * Changed the size to more C like `size_t` for all RAW API.
 * Added `golang` example

**Update 2 May 2017**

 * Added Travis-CI

**Update 08 May 2017**
 
 * *WARNING Do not use outdated implementation* https://www.tools4noobs.com/online_tools/xxtea_encrypt/ to evaluate this Library
   
   This online implementation is for an older version of XXTEA with bugs as per
   https://en.wikipedia.org/wiki/XXTEA
   
   The `xxtea-iot-crypt` uses the latest version updated by *Needham and Wheeler*
   as in https://en.wikipedia.org/wiki/XXTEA
   
   Please use the `golang` version listed below instead
  

## Example - Easy String based Encryption / Decryption ##

```arduino
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
```


## Example - RAW Encryption / Decryption ##

```arduino
#include <xxtea-iot-crypt.h>

void setup() {
  Serial.begin(115200);

  Serial.println();
  uint8_t keybuf[] = "Hello Password";
  uint8_t plaintext[] = "Hi There we can work with this";
  uint8_t buffer[200];
  size_t len = 200, i;

  // Setup the Key - Once
  if(xxtea_setup(keybuf, strlen((char *)keybuf)) != XXTEA_STATUS_SUCCESS)
  {
    Serial.println(" Assignment Failed!");
    return;
  }

  // Perform Encryption on the Data
  len = 200;  // - Initialize the Maximum buffer length
  if(xxtea_encrypt(plaintext, strlen((char*)plaintext), buffer, &len) !=
    XXTEA_STATUS_SUCCESS)
  {
    Serial.println(" Encryption Failed!");
    return;
  }
  else
  {
    Serial.println(" Encrypted Data: ");
    for(i = 0;i<len;i++)
      Serial.println(buffer[i], HEX);
    Serial.println();
  }

  // Perform Decryption
  if(xxtea_decrypt(buffer, len) != XXTEA_STATUS_SUCCESS)
  {
    Serial.println(" Decryption Failed!");
    return;
  }
  else
  {
    Serial.printf(" Decrypted Data: %s\n", buffer);
  }
}

void loop() {}
```


### Limitations ###

 * At a time only 80bytes max can be encrypted due to Buffer limitations
 * Modification needed increase the 80byte at `xxtea_iot_crypt.h` file at
    `line 36`. This indicates the size of the data buffer in Uint32 size location.
    So, for example if the total size is 240 then the value should be 30
    to accommodate.


### Dependencies ###

 Thread Safe: No
 Extendable: Yes

For more information about this library please visit us at
http://github.com/boseji/xxtea-iot-crypt


### Verification ###

We are using the `golang` for verification using the library
https://github.com/hillu/go-xxtea

Do a `go get github.com/hillu/go-xxtea` before running the below program

Example Program in **Golang** to verify the Encryption & Decryption:

```go
// Program to Check the XXTEA Encryption Library
package main

import (
  "encoding/hex"
  "fmt"
  "github.com/hillu/go-xxtea"
)

func main() {

  //str := "Hare Krishna"
  //key := "Hare Ram"
  str := "Hi There we can work with this"
  key := "Hello Password"
  // Create the Padded Key of 16 Bytes
  k := []byte(key)
  for (len(k) % 16) != 0 {
    k = append(k, 0)
  }

  // Create the New Cypher
  cipher, err := xxtea.NewCipher(k)
  if err != nil {
    panic(err)
  }

  // Create the Byte Array for Input and Output
  plain := []byte(str)
  for (len(plain) % cipher.BlockSize()) != 0 {
    plain = append(plain, 0)
  }
  // Encryption Buffer
  crypted := make([]byte, len(plain))
  // Perform Encryption
  cipher.Encrypt(crypted, plain)

  fmt.Println("Encrypted Data:")
  for i := 0; i < len(crypted); i++ {
    fmt.Printf("%X\n", crypted[i])
  }
  // For Hex Representation
  fmt.Println()
  dst := make([]byte, hex.EncodedLen(len(crypted)))
  hex.Encode(dst, crypted)
  fmt.Println(string(dst))

  // Decrypt the Encrypted Message
  cipher.Decrypt(plain, crypted)
  fmt.Println("\nDecrypted Data:")
  fmt.Println(string(plain))
}
```


## License ##

Released Under creative commons license 3.0: Attribution-ShareAlike CC BY-SA
