#include "TankStatus/ATankStatusPublisher.h"
#include "Unity/src/unity.h"
#include <string.h>

struct TankStatusPublisher pub;
struct TankStatus sub1;
struct TankStatus sub2;

#include <pthread.h> // For threading support
void setUp(void) {
  // Initialize the publisher and two "fake" subscribers
  constructTankStatusPublisher(&pub);
  constructTankStatus(&sub1);
  constructTankStatus(&sub2);
}

void tearDown(void) {}

// ... previous globals (pub, sub1, sub2) ...

/**
 * Thread Worker Function
 * This simulates a background process (like a sensor loop)
 * calling notify from another "core".
 */
void *async_notify_worker(void *arg) {
  struct TankStatusPublisher *publisher = (struct TankStatusPublisher *)arg;

  // Simulate some work
  pub._localStatus.driveLeft = 255;
  pub._localStatus.eulerZ = 90.0f;

  notify(publisher);
  return NULL;
}

void test_Publisher_Initialization(void) {
  TEST_ASSERT_EQUAL_INT(0, pub.subscriberCount);
  // Verify local status is zeroed (assuming constructTankStatus does this)
  TEST_ASSERT_EQUAL_UINT8(0, pub._localStatus.driveLeft);
}

void test_Publisher_SubscribeIncrementsCount(void) {
  subscribe(&pub, &sub1);
  TEST_ASSERT_EQUAL_INT(1, pub.subscriberCount);
  TEST_ASSERT_EQUAL_PTR(&sub1, pub.tankStatusSubscriberList[0]);

  subscribe(&pub, &sub2);
  TEST_ASSERT_EQUAL_INT(2, pub.subscriberCount);
  TEST_ASSERT_EQUAL_PTR(&sub2, pub.tankStatusSubscriberList[1]);
}

void test_Publisher_NotifyUpdatesSubscribers(void) {
  // 1. Setup subscribers
  subscribe(&pub, &sub1);
  subscribe(&pub, &sub2);

  // 2. Change the Publisher's "master" data
  pub._localStatus.driveLeft = 150;
  pub._localStatus.driveRight = 75;
  pub._localStatus.eulerX = 12.5f;

  // 3. Trigger Notify
  notify(&pub);

  // 4. Assert that the data was pushed to the subscribers' memory addresses
  TEST_ASSERT_EQUAL_UINT8(2, pub.subscriberCount);
  TEST_ASSERT_EQUAL_UINT8(150, sub1.driveLeft);
  TEST_ASSERT_EQUAL_UINT8(150, sub2.driveLeft);
  TEST_ASSERT_EQUAL_UINT8(75, sub1.driveRight);
  TEST_ASSERT_EQUAL_FLOAT(12.5f, sub2.eulerX);
}

void test_Publisher_IndependentUpdates(void) {
  subscribe(&pub, &sub1);

  // Change publisher
  pub._localStatus.driveLeft = 200;
  notify(&pub);
  TEST_ASSERT_EQUAL_UINT8(200, sub1.driveLeft);

  // Change publisher again
  pub._localStatus.driveLeft = 50;
  notify(&pub);
  TEST_ASSERT_EQUAL_UINT8(50, sub1.driveLeft);
}

void test_Publisher_NotifyFromDifferentThread(void) {
  pthread_t thread_id;

  // 1. Reset values and subscribe
  pub._localStatus.driveLeft = 0;
  sub1.driveLeft = 0;
  sub1.changeFlag = 0;
  subscribe(&pub, &sub1);

  // 2. Spawn a thread to run the notify function
  // This represents a different "cog" or "core"
  int result = pthread_create(&thread_id, NULL, async_notify_worker, &pub);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, result, "Failed to create thread");

  // 3. Wait for the thread to finish (joining)
  pthread_join(thread_id, NULL);

  // 4. Verify the data arrived in the subscriber memory
  TEST_ASSERT_EQUAL_UINT8(255, sub1.driveLeft);
  TEST_ASSERT_EQUAL_FLOAT(90.0f, sub1.eulerZ);
  TEST_ASSERT_EQUAL_UINT8(1, sub1.changeFlag);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_Publisher_Initialization);
  RUN_TEST(test_Publisher_SubscribeIncrementsCount);
  RUN_TEST(test_Publisher_NotifyUpdatesSubscribers);
  RUN_TEST(test_Publisher_IndependentUpdates);
  RUN_TEST(test_Publisher_NotifyFromDifferentThread);
  return UNITY_END();
}
