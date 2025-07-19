#include "run_tests.h"

void run_all_tests(void) {
    PINSEL_RunTests();
    GPIO_RunTests();
    SYSTICK_RunTests();
}
