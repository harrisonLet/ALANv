#ifndef TEST_CHECK_H
#define TEST_CHECK_H

#include <stdio.h>

extern int check_count;    // CHECKs run, defined in test_main.c
extern int check_failures; // CHECKs failed

// Prints the file, line and condition on failure; keeps going either way.
#define CHECK(cond) do { \
    check_count++; \
    if (!(cond)) { \
        check_failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

void test_geo(void);
void test_route(void);
void test_tack(void);

#endif // TEST_CHECK_H
