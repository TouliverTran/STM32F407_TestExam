/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FUNCTION_TEST_H
#define __FUNCTION_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

void test_add(void);
void test_subtract(void);
void test_multiply(void);
void test_divide(void);
void tc_add(void);
void tc_subtract(void);
void tc_multiply(void);
void tc_divide(void);

void setUp();
void tearDown();

#ifdef __cplusplus
}
#endif

#endif /* __FUNCTION_TEST_H */
