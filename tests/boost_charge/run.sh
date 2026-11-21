#!/bin/sh
# Host test for motor/boost_charge.c. Needs a host C compiler: CC=gcc sh run.sh
# boost_charge.c is read from stdin, so that its includes find the stubs and not
# the firmware headers next to it.
cd "$(dirname "$0")" || exit 1
CFLAGS="-std=gnu11 -Wall -Wextra -O1 -Istubs -I../../motor"
${CC:-cc} $CFLAGS -c -x c - -o boost_charge.o < ../../motor/boost_charge.c &&
${CC:-cc} $CFLAGS test.c boost_charge.o -lm -o boost_charge_test && ./boost_charge_test
