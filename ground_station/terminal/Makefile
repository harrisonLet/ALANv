CC     ?= cc
CFLAGS ?= -O2 -Wall -Wextra -std=c11

gs_term: gs_term.c telemetry.h
	$(CC) $(CFLAGS) -o $@ gs_term.c -lm

clean:
	rm -f gs_term

.PHONY: clean
