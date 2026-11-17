#include "test_check.h"

int check_count;
int check_failures;

static void run(const char *name, void (*test)(void))
{
    int count = check_count, failures = check_failures;
    test();
    printf("%-6s %3d checks, %d failed\n", name, check_count - count, check_failures - failures);
}

int main(void)
{
    run("geo", test_geo);
    run("route", test_route);
    run("tack", test_tack);

    printf("%s: %d checks, %d failed\n", check_failures ? "FAILED" : "PASSED", check_count, check_failures);
    return check_failures != 0;
}
