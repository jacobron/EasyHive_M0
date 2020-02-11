// ---------------------------------------------------------------------------
// Created by Abhijit Bose (boseji) on 04/03/16.
// Copyright 2016 - Under creative commons license 3.0:
//        Attribution-ShareAlike CC BY-SA
//
// This software is furnished "as is", without technical support, and with no
// warranty, express or implied, as to its usefulness for any purpose.
//
// Thread Safe: No
// Extendable: Yes
//
// @file xxtea-iot-crypt.h
//
// @brief
// Library to provide the XXTEA Encryption and Decryption Facility both for
// Raw input and Strings
//
// @attribution
// This is based on the prior work done by Alessandro Pasqualini
// http://github.com/alessandro1105/XXTEA-Arduino
//
// @version API 1.2.0 - Added Travis CI & Fixed redundent code
//              1.1.0 - Updated the Size inputs and more standard Conversion
//                      for buffer between the uint32_t and uint8_t types
//
//
// @author boseji - salearj@hotmail.com
// ---------------------------------------------------------------------------

#ifndef _XXTEA_IOT_CRYPT_H_
#define _XXTEA_IOT_CRYPT_H_

#include <stdint.h>

#include <Arduino.h>

#define MAX_XXTEA_DATA32 20
#define MAX_XXTEA_KEY32  4
#define MAX_XXTEA_KEY8   (MAX_XXTEA_KEY32 * 4)
#define MAX_XXTEA_DATA8  (MAX_XXTEA_DATA32 * 4)

#define XXTEA_STATUS_SUCCESS          0
#define XXTEA_STATUS_GENERAL_ERROR    1
#define XXTEA_STATUS_PARAMETER_ERROR  2
#define XXTEA_STATUS_SIZE_ERROR       3
#define XXTEA_STATUS_ALIGNMENT_ERROR  4

// Find size of Uint32 array from an input of Byte array size
#define UINT32CALCBYTE(X) ( ( ( X & 3 ) != 0 ) ? ((X >> 2) + 1) : (X >> 2) )

// Find size of Byte array from an input of Uint32 array size
#define BYTECALCUINT32(X) ( X << 2 )

/**
 * Function to Setup the Key in order to perform
 *
 * @param key [in] pointer to the Array containing the Key
 * @param len [in] Length of the Key - maximum it can be 16
 *
 * @note The Key length should not exceed the @ref MAX_XXTEA_KEY32 parameter
 *
 * @return Status of operation
 *   - XXTEA_STATUS_SUCCESS for successful association
 *   - XXTEA_STATUS_PARAMETER_ERROR for error in input parameters
 *   - XXTEA_STATUS_SIZE_ERROR in case the key is too long
 *
 * TODO: Add `__attribute_deprecated__` at end of the functions to be Disabled
 */
int xxtea_setup(uint8_t *key, size_t len);

int xxtea_encrypt(uint8_t *data, size_t len, uint8_t *buf, size_t *maxlen);
int xxtea_decrypt(uint8_t *data, size_t len);

class xxtea_c
{
  private:
  bool keyset;
  uint8_t data[MAX_XXTEA_DATA8];
  public:
  xxtea_c(){this->keyset = false;}
  bool setKey(String key);
  String encrypt(String data);
  String decrypt(String data);
};

extern xxtea_c xxtea;

#endif /* _XXTEA_IOT_CRYPT_H_ */
