#include <Arduino.h>
#include <unity.h>

#include "ClockMath.h"

namespace {

void testEpochSanity() {
  TEST_ASSERT_FALSE(clockEpochIsSane(0));
  TEST_ASSERT_TRUE(clockEpochIsSane(CLOCK_DEFAULT_EPOCH_SEC));
  TEST_ASSERT_FALSE(clockNtpEpochIsSane(CLOCK_DEFAULT_EPOCH_SEC));
  TEST_ASSERT_TRUE(clockNtpEpochIsSane(1767225600ULL));
}

void testDefaultAndRestoredInitialization() {
  ClockInitialSelection selected = clockSelectInitialEpoch(0, 0);
  TEST_ASSERT_EQUAL_UINT64(CLOCK_DEFAULT_EPOCH_SEC, selected.epochSec);
  TEST_ASSERT_FALSE(selected.restored);

  selected = clockSelectInitialEpoch(CLOCK_DEFAULT_EPOCH_SEC + 100,
                                     CLOCK_DEFAULT_EPOCH_SEC + 200);
  TEST_ASSERT_EQUAL_UINT64(CLOCK_DEFAULT_EPOCH_SEC + 200,
                           selected.epochSec);
  TEST_ASSERT_TRUE(selected.restored);

  selected = clockSelectInitialEpoch(CLOCK_DEFAULT_EPOCH_SEC + 300, 42);
  TEST_ASSERT_EQUAL_UINT64(CLOCK_DEFAULT_EPOCH_SEC + 300,
                           selected.epochSec);
  TEST_ASSERT_TRUE(selected.restored);
}

void testTimeOfDayAndMidnightRollover() {
  TEST_ASSERT_EQUAL_UINT32(86390,
      clockLocalTimeOfDaySec(CLOCK_DEFAULT_EPOCH_SEC + 86390, 0));
  TEST_ASSERT_EQUAL_UINT32(10,
      clockLocalTimeOfDaySec(CLOCK_DEFAULT_EPOCH_SEC + 86410, 0));
  TEST_ASSERT_EQUAL_UINT32(8 * 3600,
      clockLocalTimeOfDaySec(CLOCK_DEFAULT_EPOCH_SEC, 8 * 60));
}

void testManualTimePreservesHiddenLocalDay() {
  uint64_t result = 0;
  TEST_ASSERT_TRUE(clockEpochForLocalTimeOfDay(
      CLOCK_DEFAULT_EPOCH_SEC + 12 * 3600, 8 * 60,
      8 * 3600 + 30 * 60, result));
  TEST_ASSERT_EQUAL_UINT64(CLOCK_DEFAULT_EPOCH_SEC + 30 * 60, result);

  TEST_ASSERT_TRUE(clockEpochForLocalTimeOfDay(
      CLOCK_DEFAULT_EPOCH_SEC + 2 * 86400 + 23 * 3600, 0,
      6 * 3600, result));
  TEST_ASSERT_EQUAL_UINT64(CLOCK_DEFAULT_EPOCH_SEC + 2 * 86400 + 6 * 3600,
                           result);
}

void testWateringAgeSurvivesClockCorrections() {
  TEST_ASSERT_EQUAL_UINT64(
      9700, clockRebaseTimestampPreservingAge(2000, 10000, 1700));
  TEST_ASSERT_EQUAL_UINT64(
      1200, clockRebaseTimestampPreservingAge(2000, 1500, 1700));
  TEST_ASSERT_EQUAL_UINT64(
      1500, clockRebaseTimestampPreservingAge(2000, 1500, 2100));
  TEST_ASSERT_EQUAL_UINT64(
      0, clockRebaseTimestampPreservingAge(2000, 1500, 0));
}

}  // namespace

void setup() {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(testEpochSanity);
  RUN_TEST(testDefaultAndRestoredInitialization);
  RUN_TEST(testTimeOfDayAndMidnightRollover);
  RUN_TEST(testManualTimePreservesHiddenLocalDay);
  RUN_TEST(testWateringAgeSurvivesClockCorrections);
  UNITY_END();
}

void loop() {}
