#include <Wire.h>
#define AS5600_ADDR 0x36

bool readReg8(uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)1) != 1) return false;
  out = Wire.read();
  return true;
}

bool readReg16(uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return false;
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  out = ((hi << 8) | lo) & 0x0FFF;
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  delay(50);
  Serial.println("ms,raw,deg,agc,MD,ML,MH");   // CSV header
}

void loop() {
  uint16_t raw;
  uint8_t status, agc;
  if (!readReg16(0x0C, raw) || !readReg8(0x0B, status) || !readReg8(0x1A, agc)) {
    return;  // skip bad reads silently
  }
  float angle = raw * (360.0f / 4096.0f);
  Serial.print(millis()); Serial.print(',');
  Serial.print(raw);      Serial.print(',');
  Serial.print(angle, 3); Serial.print(',');
  Serial.print(agc);      Serial.print(',');
  Serial.print((status >> 5) & 1); Serial.print(',');
  Serial.print((status >> 4) & 1); Serial.print(',');
  Serial.println((status >> 3) & 1);
  delay(20);  // 50 Hz
}
