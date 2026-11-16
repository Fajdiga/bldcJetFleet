#!/bin/sh
# Host test for motor/boost_charge.c. Needs a host C compiler: CC=gcc sh run.sh
cd "$(dirname "$0")" || exit 1
${CC:-cc} -std=gnu11 -Wall -Wextra -O1 -Istubs -I../../motor ../../motor/boost_charge.c test.c -lm -o boost_charge_test && ./boost_charge_test
