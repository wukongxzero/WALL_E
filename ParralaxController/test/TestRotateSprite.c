#include "Graphics/RotatePixelArt.h"
#include "Unity/src/unity.h"
#include <Graphics/RotatePixelArt.h>
#include <Graphics/pixelArtSampleData.h>
#include <string.h>

struct RotatingSprite testSprite;
struct PixelDataRGB_8bit testFrame;

// A simple 8x8 horizontal beam in the middle of the sprite
unsigned char
    horizontalBeam[CENTER_GRAPHIC_PIXEL_MAX][CENTER_GRAPHIC_PIXEL_MAX] = {
        {_, _, _, _, _, _, _, _}, {_, _, _, _, _, _, _, _},
        {_, _, _, _, _, _, _, _}, {X, X, X, X, X, X, X, X}, // Row 3 is full
        {_, _, _, _, _, _, _, _}, {_, _, _, _, _, _, _, _},
        {_, _, _, _, _, _, _, _}, {_, _, _, _, _, _, _, _}};

void setUp(void) {
  // Initialize the structures
  constructGraphicsSpriteRotatableWOrigin(&testSprite, &testFrame);
  // Clear the frame memory
  memset(testFrame.frame, _, sizeof(testFrame.frame));
}

void tearDown(void) {}

/**
 * Test 1: Verify extraction of the 8x8 sprite into sparse elements
 */
void test_Extraction_CorrectlyIdentifiesForeground(void) {
  extractForegroundFromSprite(&testSprite, horizontalBeam, 64);

  TEST_ASSERT_EQUAL_INT(8, testSprite.elementCount);

  // Check first and last element coordinates
  TEST_ASSERT_EQUAL_INT(3, testSprite.centerGraphic[0]._row);
  TEST_ASSERT_EQUAL_INT(0, testSprite.centerGraphic[0]._col);
  TEST_ASSERT_EQUAL_INT(3, testSprite.centerGraphic[7]._row);
  TEST_ASSERT_EQUAL_INT(7, testSprite.centerGraphic[7]._col);
}

/**
 * Test 2: Verify 0-degree rotation (Identity transformation)
 * At 0 degrees, the beam should remain horizontal.
 */
void test_Rotate_ZeroDegrees(void) {
  extractForegroundFromSprite(&testSprite, horizontalBeam, 64);
  rotateSprite(&testSprite, 0);

  // Midpoint is 7.5. Row 3 in 8x8 space is offset from the center.
  // Let's verify at least one point is drawn where we expect.
  // Row 3 becomes row (3 - 7.5 + 7.5) = 3
  // Col 0 becomes col (0 - 7.5 + 7.5) = 0
  TEST_ASSERT_EQUAL_UINT8(X, testFrame.frame[0][3]);
  TEST_ASSERT_EQUAL_UINT8(X, testFrame.frame[7][3]);
}

/**
 * Test 3: Verify 90-degree rotation
 * A horizontal beam should become a vertical beam.
 */
void test_Rotate_90Degrees(void) {
  extractForegroundFromSprite(&testSprite, horizontalBeam, 64);

  // Rotate 90 degrees clockwise
  rotateSprite(&testSprite, 90);

  // In a 90-degree rotation:
  // Original Row 3 (Horizontal) should map to a Column in the 16x16 space.
  // We check a specific coordinate that should have been hit.
  // Calculation: rotX = (0-7.5)*cos(90) - (3-7.5)*sin(90) = 0 - (-4.5 * 1)
  // = 4.5 round(4.5 + 7.5) = 12
  TEST_ASSERT_EQUAL_UINT8(X, testFrame.frame[12][7]);
}

/**
 * Test 4: Verify the frame is cleared between rotations
 */
void test_Rotate_ClearsPreviousFrame(void) {
  extractForegroundFromSprite(&testSprite, horizontalBeam, 64);

  // Draw at 0 degrees
  rotateSprite(&testSprite, 0);
  TEST_ASSERT_EQUAL_UINT8(X, testFrame.frame[0][3]);

  // Rotate to 90 degrees
  rotateSprite(&testSprite, 90);

  // The old pixel at [0][3] should now be 0 (background _)
  TEST_ASSERT_EQUAL_UINT8(_, testFrame.frame[0][3]);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_Extraction_CorrectlyIdentifiesForeground);
  RUN_TEST(test_Rotate_ZeroDegrees);
  RUN_TEST(test_Rotate_90Degrees);
  RUN_TEST(test_Rotate_ClearsPreviousFrame);
  return UNITY_END();
}
