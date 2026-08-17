#include <Wire.h>

// Common I2C addresses
const uint8_t MPU_ADDR   = 0x68; // MPU6050 default
const uint8_t AS5600_ADDR = 0x36; // AS5600 typical

// ---- I2C helpers ----
bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

bool readReg8(uint8_t addr, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

bool writeReg8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

void scanI2C() {
  Serial.println("\nI2C scan:");
  uint8_t count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2cDevicePresent(addr)) {
      Serial.print("  Found 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (!count) Serial.println("  No I2C devices found.");
}

// ---- MPU6050 minimal init + WHO_AM_I ----
void checkMPU() {
  Serial.println("\nMPU6050 check:");
  if (!i2cDevicePresent(MPU_ADDR)) {
    Serial.println("  MPU6050 not found at 0x68.");
    return;
  }

  // Wake up MPU6050: PWR_MGMT_1 (0x6B) = 0
  if (!writeReg8(MPU_ADDR, 0x6B, 0x00)) {
    Serial.println("  Failed to write PWR_MGMT_1.");
    return;
  }

  uint8_t who = 0;
  if (readReg8(MPU_ADDR, 0x75, who)) { // WHO_AM_I register
    Serial.print("  WHO_AM_I = 0x");
    Serial.println(who, HEX);
    // Typical expected: 0x68
  } else {
    Serial.println("  Failed to read WHO_AM_I.");
  }
}

// ---- AS5600 minimal check (read raw angle) ----
// Raw angle registers: RAW_ANGLE(0x0C MSB, 0x0D LSB) on AS5600
bool readAS5600RawAngle(uint16_t &angle) {
  uint8_t msb = 0, lsb = 0;
  if (!readReg8(AS5600_ADDR, 0x0C, msb)) return false;
  if (!readReg8(AS5600_ADDR, 0x0D, lsb)) return false;
  angle = ((uint16_t)msb << 8) | lsb;
  angle &= 0x0FFF; // 12-bit
  return true;
}

void checkAS5600() {
  Serial.println("\nAS5600 check:");
  if (!i2cDevicePresent(AS5600_ADDR)) {
    Serial.println("  AS5600 not found at 0x36.");
    return;
  }

  uint16_t ang = 0;
  if (readAS5600RawAngle(ang)) {
    Serial.print("  RAW ANGLE (0..4095) = ");
    Serial.println(ang);
    Serial.println("  Move the magnet and confirm this number changes.");
  } else {
    Serial.println("  Found AS5600 but failed to read angle regs.");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Fast I2C helps

  Serial.println("I2C + MPU6050 + AS5600 bring-up");
  scanI2C();
  checkMPU();
  checkAS5600();
}

void loop() {
  // Re-check AS5600 angle at 10 Hz
  static uint32_t last = 0;
  if (millis() - last >= 100) {
    last = millis();
    uint16_t ang = 0;
    if (readAS5600RawAngle(ang)) {
      Serial.print("AS5600 angle: ");
      Serial.println(ang);
    }
  }
}
