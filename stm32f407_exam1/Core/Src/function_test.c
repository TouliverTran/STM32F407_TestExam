#include <stdio.h>
#include "function_test.h"
#include "unity.h"
#include "function.h"

void test_add(void){
    TEST_ASSERT_EQUAL_INT(5, add(2,3));
    TEST_ASSERT_EQUAL_INT(0, add(-1,1));
    TEST_ASSERT_EQUAL_INT(-2, add(-1,-1));
}

void test_subtract(void){
    TEST_ASSERT_EQUAL_INT(1, subtract(3,2));
    TEST_ASSERT_EQUAL_INT(-2, subtract(-1,1));
    TEST_ASSERT_EQUAL_INT(0, subtract(-1,-1));
}

void test_multiply(void){
    TEST_ASSERT_EQUAL_INT(6, multiply(2,3));
    TEST_ASSERT_EQUAL_INT(-1, multiply(-1,1));
    TEST_ASSERT_EQUAL_INT(1, multiply(-1,-1));
}

void test_divide(void){
    TEST_ASSERT_EQUAL_INT(2, divide(6,3));
    TEST_ASSERT_EQUAL_INT(-1, divide(-1,1));
    TEST_ASSERT_EQUAL_INT(0, divide(1,0));
}

void tc_add(void){
    UNITY_BEGIN();
    RUN_TEST(test_add);
    RUN_TEST(test_add);
    TEST_ASSERT_EQUAL_INT(5, add(1,3));
    UNITY_END();

    // UNITY_BEGIN();
    // TEST_ASSERT_EQUAL_INT(5, add(1,3));
    // UNITY_END();
}

void tc_subtract(void){
  UNITY_BEGIN();
  RUN_TEST(test_subtract);
  UNITY_END();

  UNITY_BEGIN();
  TEST_ASSERT_EQUAL_INT(2, subtract(3,2));
  UNITY_END();
}

void tc_multiply(void){
  UNITY_BEGIN();
  RUN_TEST(test_multiply);
  UNITY_END();

  UNITY_BEGIN();
  TEST_ASSERT_EQUAL_INT(5, multiply(2,3));
  UNITY_END();
}

void tc_divide(void){
  UNITY_BEGIN();
  RUN_TEST(test_divide);
  UNITY_END();

  UNITY_BEGIN();
  TEST_ASSERT_EQUAL_INT(3, divide(6,3));
  UNITY_END();
}

void setUp(){

}
void tearDown(){

}