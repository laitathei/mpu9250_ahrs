#include "utils.h"

void time_sleep(float dt, float time)
{
    clock_t time_end;
    time_end = clock() + (time) * CLOCKS_PER_SEC;
    while (float(clock()+dt) < time_end)
    {
    }
}