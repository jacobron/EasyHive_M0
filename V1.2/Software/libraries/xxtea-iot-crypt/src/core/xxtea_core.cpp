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
// @file xxtea_core.cpp
//
// @brief 
// Library to provide the XXTEA Encryption and Decryption Facility both for
// Raw input and Strings
// 
// @version API  1.2.1 - Fixed the Signed Arithmetic Problem
//               1.0.0 - Initial Version
//
//
// @author boseji - salearj@hotmail.com
// ---------------------------------------------------------------------------
#include "xxtea_internal.h"

#define DELTA 0x9e3779b9
#define MX ((((z>>5)^(y<<2)) + ((y>>3)^(z<<4))) ^ ((sum^y) + (key[(p&3)^e] ^ z)))

/*
// -- Original Logic has Signed Arithmetic Problem for smaller Processors
void dtea_fn(uint32_t *v, int32_t n, uint32_t const key[4])
{
  uint32_t y, z, sum;
  uint32_t p, rounds, e;
  if (n > 1)
  { // Coding Part
    rounds = 6 + 52/n;
    sum = 0;
    z = v[n-1];
    do {
      sum += DELTA;
      e = (sum >> 2) & 3;
      for (p=0; p<n-1; p++)
      {
        y = v[p+1];
        z = v[p] += MX;
      }
      y = v[0];
      z = v[n-1] += MX;
    } while (--rounds);
  } // End of Encoding
  else if (n < -1)
  {  // Decoding Part
    n = -n;
    rounds = 6 + 52/n;
    sum = rounds*DELTA;
    y = v[0];
    do {
      e = (sum >> 2) & 3;
      for (p=n-1; p>0; p--)
      {
        z = v[p-1];
        y = v[p] -= MX;
      }
      z = v[n-1];
      y = v[0] -= MX;
      sum -= DELTA;
    } while (--rounds);
  } // End of Decoding
}
*/

void dtea_fn1(uint32_t *v, int32_t n, uint32_t const key[4])
{
  uint32_t y, z, sum;
  uint32_t p, rounds, e, an;
  an = n;
  if (n > 1)
  { // Coding Part
    rounds = 6 + 52/an;
    sum = 0;
    z = v[an-1];
    do {
      sum += DELTA;
      e = (sum >> 2) & 3;
      for (p=0; p<an-1; p++)
      {
        y = v[p+1];
        z = v[p] += MX;
      }
      y = v[0];
      z = v[an-1] += MX;
    } while (--rounds);
  } // End of Encoding
  else if (n < -1)
  {  // Decoding Part
    an = (uint32_t)-n;
    rounds = 6 + 52/an;
    sum = rounds*DELTA;
    y = v[0];
    do {
      e = (sum >> 2) & 3;
      for (p=an-1; p>0; p--)
      {
        z = v[p-1];
        y = v[p] -= MX;
      }
      z = v[an-1];
      y = v[0] -= MX;
      sum -= DELTA;
    } while (--rounds);
  } // End of Decoding
}
