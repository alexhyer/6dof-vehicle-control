#ifndef GNSS_H
#define GNSS_H

//Credit for 10hz ublox gnss code goes to iforce2d and can be found here: https://youtu.be/TwhCX0c8Xe0

  struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};
    
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

class Gnss{
    public:
    NAV_POSLLH posllh;
    bool processGPS();
    void calcChecksum(unsigned char* CK);
};

#endif