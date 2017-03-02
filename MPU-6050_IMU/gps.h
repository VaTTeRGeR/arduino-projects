#ifndef GPS_H
#define GPS_H

#define GPS_Serial Serial4
#define USB_Serial Serial

//[HEADER, ID   , LENGTH, PAYLOAD, CHECKSUM]
//[0...1 , 2...3, 4...5 , 6...33 , 34...35 ]
//Little Endian
const byte messageSize = 36;
const byte dataBegin = 6;
const byte dataEnd = 33;
byte data[28];
byte pos = 0;

signed long   lon, lat, elev_sl;
unsigned long h_acc, v_acc;
float         lonf,latf;

unsigned long readU4(byte *data) {
  uint32_t value = (uint32_t)(*data);  data++;
  value |= ((uint32_t)(*data)) << 8;    data++;
  value |= ((uint32_t)(*data)) << 16;   data++;
  value |= ((uint32_t)(*data)) << 24;
  return value;
}

unsigned long readS4(byte *data) {
  int32_t value = (int32_t)(*data);  data++;
  value |= ((int32_t)(*data)) << 8;    data++;
  value |= ((int32_t)(*data)) << 16;   data++;
  value |= ((int32_t)(*data)) << 24;
  return value;
}

void sendUBX(byte *UBXmsg, byte msgLength) {
  for(int i = 0; i < msgLength; i++) {
    GPS_Serial.write(UBXmsg[i]);
    GPS_Serial.flush();
  }
  GPS_Serial.println();
  GPS_Serial.flush();
}

void calcChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}

void setup_gps() {
  USB_Serial.begin(115200);
  
  GPS_Serial.begin(9600);

  delay(25);
  
  byte prt[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x72};
  calcChecksum(&prt[2], sizeof(prt)-4);
  sendUBX(&prt[0], sizeof(prt));

  delay(25);
  
  GPS_Serial.end();
  GPS_Serial.begin(115200);
  
  delay(25);

  byte posllh[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x17, 0xCD};
  calcChecksum(&posllh[2], sizeof(posllh)-4);
  sendUBX(&posllh[0], sizeof(posllh));

  delay(25);
  
  byte nav5[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0xDC};
  calcChecksum(&nav5[2], sizeof(nav5)-4);
  sendUBX(&nav5[0], sizeof(nav5));

  delay(25);

  byte rate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96};
  calcChecksum(&rate[2], sizeof(rate)-4);
  sendUBX(&rate[0], sizeof(rate));

  delay(25);

  byte gnss[] = {0xB5, 0x62, 0x06, 0x3E, 0x24, 0x00, 0x00, 0x00, 0x16, 0x04, 0x00, 0x04, 0xFF, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x06, 0x08, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x01, 0xA4, 0x25};
  calcChecksum(&gnss[2], sizeof(gnss)-4);
  sendUBX(&gnss[0], sizeof(gnss));
}

void readData(){
  lon     = readS4(data + 4); //Longitude * e-7 [deg]
  lat     = readS4(data + 8); //Latitude  * e-7 [deg]
  elev_sl = readS4(data + 16); //Height above mean sea-level [mm]
  h_acc   = readU4(data + 20); //Horizontal accuracy [mm]
  v_acc   = readU4(data + 24); //Vertical accuracy [mm]

  lonf = ((float)lon)/10000000.0;
  latf = ((float)lat)/10000000.0;

  USB_Serial.print("lon:");
  USB_Serial.println(lon);
  USB_Serial.print("lat:");
  USB_Serial.println(lat);
  USB_Serial.print("height:");
  USB_Serial.println(elev_sl);
  USB_Serial.print("hacc:");
  USB_Serial.println(h_acc);
  USB_Serial.print("vacc:");
  USB_Serial.println(v_acc);
}

void update_parser(byte b){
  switch (pos) {
    case 0:
      if(b == 0xB5) pos++;
      else pos = 0;
    break;
    
    case 1:
      if(b == 0x62) pos++;
      else pos = 0;
    break;
    
    case 2:
      if(b == 0x01) pos++;
      else pos = 0;
    break;
    
    case 3:
      if(b == 0x02) pos++;
      else pos = 0;
    break;
    
    case 4:
      if(b == 28) pos++;
      else pos = 0;
    break;
    
    case 5:
      if(b == 0) pos++;
      else pos = 0;
    break;
    
    default:
      if(pos >= dataBegin && pos <= dataEnd) {
        data[pos - dataBegin] = b;
        pos++;
      } else {
        readData();
        pos = 0;
      }
    break;
  }
}

void update_gps() {
  while (GPS_Serial.available() > 0) {
    byte b = GPS_Serial.read();
    USB_Serial.write(b);
    update_parser(b);
  }
  while (USB_Serial.available() > 0) {
    GPS_Serial.write(USB_Serial.read());
  }
}

#endif
