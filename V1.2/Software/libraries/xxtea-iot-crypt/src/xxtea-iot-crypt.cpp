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
// @file xxtea-iot-crypt.cpp
//
// @brief
// Library to provide the XXTEA Encryption and Decryption Facility both for
// Raw input and Strings
//
// @version API 1.2.1 - Fixed the Signed Arithmetic Problem
//              1.2.0 - Added Travis CI & Fixed redundent code
//              1.1.0 - Updated the Size inputs and more standard Conversion
//                      for buffer between the uint32_t and uint8_t types
//
//
// @author boseji - salearj@hotmail.com
// ---------------------------------------------------------------------------

#include "xxtea-iot-crypt.h"

#include "core/xxtea_internal.h"

#include <string.h>

static uint32_t xxtea_data[MAX_XXTEA_DATA32];
static uint32_t xxtea_key[MAX_XXTEA_KEY32];

int xxtea_setup(uint8_t *key, size_t len)
{
  int ret = XXTEA_STATUS_GENERAL_ERROR;
  size_t osz;

  do{

    // Parameter Check
    if(key == NULL || len <= 0 || len > MAX_XXTEA_KEY8)
    {
      ret = XXTEA_STATUS_PARAMETER_ERROR;
      break;
    }

    osz = UINT32CALCBYTE(len);

    // Check for Size Errors
    if(osz > MAX_XXTEA_KEY32)
    {
      ret = XXTEA_STATUS_SIZE_ERROR;
      break;
    }

    // Clear the Key
    memset((void *)xxtea_key, 0, MAX_XXTEA_KEY8);

    // Copy the Key from Buffer
    memcpy((void *)xxtea_key, (const void *)key, len);

    // We have Success
    ret = XXTEA_STATUS_SUCCESS;
  }while(0);

  return ret;
}

int xxtea_encrypt(uint8_t *data, size_t len, uint8_t *buf, size_t *maxlen)
{
  int ret = XXTEA_STATUS_GENERAL_ERROR;
  int32_t l;
  do{
    if(data == NULL || len <= 0 || len > MAX_XXTEA_DATA8 ||
      buf == NULL || *maxlen <= 0 || *maxlen < len)
    {
      ret = XXTEA_STATUS_PARAMETER_ERROR;
      break;
    }
    // Calculate the Length needed for the 32bit Buffer
    l = UINT32CALCBYTE(len);

    // Check if More than expected space is needed
    if(l > MAX_XXTEA_DATA32 || *maxlen < (l * 4))
    {
      ret = XXTEA_STATUS_SIZE_ERROR;
      break;
    }

    // Clear the Data
    memset((void *)xxtea_data, 0, MAX_XXTEA_DATA8);

    // Copy the Data from Buffer
    memcpy((void *)xxtea_data, (const void *)data, len);

    // Perform Encryption - Change for Unsigned Handling while using memory Pointer (18th July 2017)
    dtea_fn1(xxtea_data, l, (const uint32_t *)xxtea_key);

    // Copy Encrypted Data back to buffer
    memcpy((void *)buf, (const void *)xxtea_data, (l*4));

    // Assign the Length
    *maxlen = l*4;

    ret = XXTEA_STATUS_SUCCESS;
  }while(0);
  return ret;
}

int xxtea_decrypt(uint8_t *data, size_t len)
{
  int ret = XXTEA_STATUS_GENERAL_ERROR;
  int32_t l;
  do {
    if(data == NULL || len <= 0 || (len%4) != 0)
    {
      ret = XXTEA_STATUS_PARAMETER_ERROR;
      break;
    }
    if(len > MAX_XXTEA_DATA8)
    {
      ret = XXTEA_STATUS_SIZE_ERROR;
      break;
    }
    // Copy the Data into Processing Array
    memset((void *)xxtea_data, 0, MAX_XXTEA_DATA8);
    memcpy((void *)xxtea_data, (const void *)data, len);
    // Get the Actual Size in 32bits - Negative for Decryption
    l = -((int32_t)len / 4); // - Change for Using correct Signed arithmetic (18th July 2017)
    // Performn Decryption - Change for Unsigned Handling while using memory Pointer (18th July 2017)
    dtea_fn1(xxtea_data, l, (const uint32_t *)xxtea_key);
    // Copy Encrypted Data back to buffer
    memcpy((void *)data, (const void *)xxtea_data, len);
    ret = XXTEA_STATUS_SUCCESS;
  }while(0);
  return ret;
}

bool xxtea_c::setKey(String key)
{
  // Initially reset the Status of the Key
  keyset = false;
  if(key.length() <= MAX_XXTEA_KEY8)
  {
    // Setup the Key
    xxtea_setup((uint8_t *)key.c_str(), key.length());
    // Say that the Key has been Initialized
    keyset = true;
    return true;
  }
  return false;
}

static char tohex(uint8_t b)
{
  if(b>9)
    return (char)(b + 'A' - 10);
  return (char)(b + '0');
}

String xxtea_c::encrypt(String data)
{
  // Works only if the Key in setup
  if(this->keyset && data.length() != 0)
  {
    // If the Data within the limits of the Engine
    if(data.length() < MAX_XXTEA_DATA8)
    {
      size_t len;
      // Assign the Maximum buffer we have
      len = MAX_XXTEA_DATA8;
      // Perform Encryption
      if(xxtea_encrypt((uint8_t *)data.c_str(),data.length(),
        this->data,&len) == XXTEA_STATUS_SUCCESS)
      {
        String result;
        size_t i;
        result.reserve(len*2 + 1);
        result = "";
        for(i=0;i<len;i++)
        {
          result+=tohex((this->data[i] >> 4) & 0x0F);
          result+=tohex(this->data[i] & 0x0F);
        }
        return result;
      }
    }
  }
  return String(F("-FAIL-"));
}

static uint8_t c2h(char *data)
{
  uint8_t b = 0;
  if(data[0] >= 'A')
    b |= data[0] - 'A' + 10;
  else
    b |= data[0] - '0';
  b <<= 4;
  if(data[1] >= 'A')
    b |= data[1] - 'A' + 10;
  else
    b |= data[1] - '0';
  return b;
}

String xxtea_c::decrypt(String data)
{
  // Works only if the Key in setup
  if(this->keyset && data.length() != 0 && (data.length() % 4) == 0)
  {
    // If the Data within the limits of the Engine
    if(data.length() < (MAX_XXTEA_DATA8 * 2))
    {
      uint32_t len,i,k;
      memset(this->data, 0, MAX_XXTEA_DATA8);
      len = data.length()/2;
      data.toUpperCase(); // Converting all Hex to Upper case for easy conversion
      // Fill up the Data Buffer
      for(i=0,k = 0;i<len;i++,k+=2)
      {
        this->data[i] = c2h((char *)data.substring(k,k+2).c_str());
      }
      // Perform Decryption
      if(xxtea_decrypt(this->data, len) == XXTEA_STATUS_SUCCESS)
      {
        String result;
        result = (char *)this->data;
        return result;
      }
    }
  }
  return String("-FAIL-");
}


xxtea_c xxtea;
