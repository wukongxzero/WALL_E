#include "TankStatus/TankStatus.h"
#include "Unity/src/unity.h"
#include <string.h>

// This runs BEFORE every individual test
void setUp(void) {
  // Reset variables or allocate memory if needed
}

// This runs AFTER every individual test
void tearDown(void) {
  // Clean up memory or close files
}

void test_Serialization_ShouldMatchPacketLength(void) {
  struct TankStatus ts = {100, 200, 1.1f, 2.2f, 3.3f};
  unsigned char buffer[TANKSTATUS_PACKET_LENGTH];

  makeByteTankStatus(buffer, TANKSTATUS_PACKET_LENGTH, &ts);

  // Simple check: did we at least get the drive values in the right spot?
  TEST_ASSERT_EQUAL_HEX8(100, buffer[0]);
  TEST_ASSERT_EQUAL_HEX8(200, buffer[1]);
}

void test_RoundTrip_ShouldPreserveData(void) {
  struct TankStatus input = {50, 60, 12.5f, -45.0f, 180.0f};
  struct TankStatus output;
  unsigned char buffer[16];

  makeByteTankStatus(buffer, 16, &input);
  readByteTankStatus(buffer, 16, &output);

  TEST_ASSERT_EQUAL_INT(input.driveLeft, output.driveLeft);
  TEST_ASSERT_EQUAL_FLOAT(input.eulerX, output.eulerX);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_Serialization_ShouldMatchPacketLength);
  RUN_TEST(test_RoundTrip_ShouldPreserveData);
  return UNITY_END();
}
