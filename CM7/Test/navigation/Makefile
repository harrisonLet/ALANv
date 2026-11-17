# Laptop tests for the pure navigation files. Outside App/, so the firmware build ignores it.
# Run from this directory with any host gcc (WSL on Windows):  make
# The three source files must never include system.h (HAL + FreeRTOS).

CFLAGS = -std=c11 -Wall -Wextra -DNAV_HOST_TEST -I../../App/Inc -I../../../Common/Inc
SRC    = ../../App/Src/navigationRoute.c ../../App/Src/navigationGeo.c ../../App/Src/navigationTack.c

.PHONY: test clean

test: test_route.c test_geo.c test_tack.c test_main.c $(SRC)
	gcc $(CFLAGS) $^ -lm -o run_tests && ./run_tests

clean:
	rm -f run_tests
