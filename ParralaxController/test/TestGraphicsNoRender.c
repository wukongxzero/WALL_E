#include "Graphics/AGraphicsObject.h"
#include "Graphics/PixelDataFrame.h"
#include "Unity/src/unity.h"
#include "Unity/src/unity_internals.h"
#include <string.h>

// Mock data for sprites
unsigned char mockPixels[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE];

// Global test objects
struct GraphicsObject tankObj;
struct GraphicsObjectSprite animatedTank;
struct PixelDataRGB_8bit_Animation tankAnim;
struct PixelDataRGB_8bit frames[MAX_FRAME_COUNT];

void setUp(void) {
  // Initialize pixels with a pattern
  memset(mockPixels, 0, sizeof(mockPixels));
  mockPixels[0][0] = 219;

  // Initialize 2 frames for animation
  constructSprite(&frames[0], mockPixels);
  constructSprite(&frames[1], mockPixels);

  // Setup the animation struct
  constructSpriteAnimation(&tankAnim, frames);
  tankAnim.isAnimated = 1;

  // Setup the base graphics object (Bounding Box)
  constructGraphicsObject(&tankObj, 10, 10, 16, 16);

  // Link them into the Sprite Object
  constructGraphicsSprite(&animatedTank, &tankObj);
  animatedTank.spriteAnimation = &tankAnim;
}

void tearDown(void) {
  // If you had heap allocation in deconstructSprite, call it here
}

// --- TEST CASES ---

void test_Movement_AbsolutePosition(void) {
  moveGraphicAbs(&tankObj, 50, 60);

  TEST_ASSERT_EQUAL_UINT16(50, tankObj.boundingBox._x);
  TEST_ASSERT_EQUAL_UINT16(60, tankObj.boundingBox._y);
}

void test_Movement_RelativeValue(void) {
  // Starting at 10, 10 (from setUp)
  moveGraphicAbs(&tankObj, 10, 10);
  moveGraphicByVal(&tankObj, 5, -2);

  TEST_ASSERT_EQUAL_UINT16(15, tankObj.boundingBox._x);
  TEST_ASSERT_EQUAL_UINT16(8, tankObj.boundingBox._y);
}

void test_Animation_CyclesFramesCorrectly(void) {
  // Initially frame 0
  TEST_ASSERT_EQUAL_INT(0, animatedTank.spriteAnimation->frameCurrent);

  // Update 1: Move to frame 1
  updateAnimate(&animatedTank);
  TEST_ASSERT_EQUAL_INT(1, animatedTank.spriteAnimation->frameCurrent);

  // Update 2: Should wrap back to 0 (since MAX_FRAME_COUNT is 2)
  updateAnimate(&animatedTank);
  TEST_ASSERT_EQUAL_INT(0, animatedTank.spriteAnimation->frameCurrent);
}

void test_Animation_DoesNotCycleWhenDisabled(void) {
  animatedTank.spriteAnimation->isAnimated = 0;
  animatedTank.spriteAnimation->frameCurrent = 0;

  updateAnimate(&animatedTank);

  // Should still be 0 because isAnimated is false
  TEST_ASSERT_EQUAL_INT(0, animatedTank.spriteAnimation->frameCurrent);
}

void test_Integration_MovementAndAnimationSimultaneously(void) {
  // Simulate a single game loop tick
  moveGraphicAbs(&tankObj, 10, 1);
  moveGraphicByVal(&tankObj, 1, 1);
  updateAnimate(&animatedTank);

  TEST_ASSERT_EQUAL_UINT16(11, animatedTank.graphicsObject->boundingBox._x);
  TEST_ASSERT_EQUAL_INT(1, animatedTank.spriteAnimation->frameCurrent);
}

void test_manual_change_frame(void) {
  changeFrame(&animatedTank, 1);
  TEST_ASSERT_EQUAL_UINT8(1, animatedTank.spriteAnimation->frameCurrent);

  changeFrame(&animatedTank, 2);
  TEST_ASSERT_EQUAL_UINT8(2, animatedTank.spriteAnimation->frameCurrent);

  changeFrame(&animatedTank, 3);
  TEST_ASSERT_EQUAL_UINT8(2, animatedTank.spriteAnimation->frameCurrent);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_Movement_AbsolutePosition);
  RUN_TEST(test_Movement_RelativeValue);
  RUN_TEST(test_Animation_CyclesFramesCorrectly);
  RUN_TEST(test_Animation_DoesNotCycleWhenDisabled);
  RUN_TEST(test_Integration_MovementAndAnimationSimultaneously);
  RUN_TEST(test_manual_change_frame);
  return UNITY_END();
}
