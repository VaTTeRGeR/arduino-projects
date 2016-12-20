void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  Serial.begin(115200);
  while (!Serial) {}
  
  Serial1.begin(9600);

  delay(50);

  byte baud[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x72};
  calcChecksum(&baud[2], sizeof(baud)-4);
  sendUBX(&baud[0], sizeof(baud));

  delay(50);

  Serial1.end();
  Serial1.begin(115200);
  
  delay(50);
  
  byte rate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96};
  calcChecksum(&rate[2], sizeof(rate)-4);
  sendUBX(&rate[0], sizeof(rate));

  delay(50);
  
  byte posllh[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x17, 0xCD};
  calcChecksum(&posllh[2], sizeof(posllh)-4);
  sendUBX(&posllh[0], sizeof(posllh));
}

void loop() { // run over and over
  if (Serial1.available()) {
    byte b = Serial1.read();
    Serial.write(b);
    updateState(b);
  }
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
}

//[HEADER, ID   , LENGTH, PAYLOAD, CHECKSUM]
//[0...1 , 2...3, 4...5 , 6...33 , 34...35 ]
//Little Endian
const byte messageSize = 36;
const byte dataBegin = 6;
const byte dataEnd = 33;
byte data[28];
byte pos = 0;

void updateState(byte b){
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

void readData(){
  byte * pointer = data + 20; //Horizontal accuracy
  if(readU4(pointer) < 20000) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}

uint32_t readU4(byte *data) {
  uint32_t value = (uint32_t)(*data);  data++;
  value |= ((uint32_t)(*data)) << 8;    data++;
  value |= ((uint32_t)(*data)) << 16;   data++;
  value |= ((uint32_t)(*data)) << 24;
  return value;
}

uint16_t readU2(byte *data) {
  uint16_t value = (uint16_t)(*data);  data++;
  value |= ((uint16_t)(*data)) << 8;
  return value;
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

void sendUBX(byte *UBXmsg, byte msgLength) {
  for(int i = 0; i < msgLength; i++) {
    Serial1.write(UBXmsg[i]);
    Serial1.flush();
  }
  Serial1.println();
  Serial1.flush();
}

