#include <PS2X_lib.h>
PS2X ps2x;

#define PS2_DAT 50
#define PS2_CMD 51
#define PS2_SEL 53
#define PS2_CLK 52

void setup() {
  Serial.begin(115200);
  delay(800);
  int err = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  Serial.print("error="); Serial.println(err);
  if(err==0){
    Serial.print("type="); Serial.println(ps2x.readType());
    Serial.println("CONNECTED");
  }
}

void loop() {
  ps2x.read_gamepad(false, 0);
  Serial.print("LX="); Serial.print(ps2x.Analog(PSS_LX));
  Serial.print(" LY="); Serial.print(ps2x.Analog(PSS_LY));
  Serial.print(" X=");  Serial.println(ps2x.Button(PSB_CROSS));
  delay(150);
}