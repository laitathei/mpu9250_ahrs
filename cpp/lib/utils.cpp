#include "utils.h"

using namespace std;

void time_sleep(float dt, float time)
{
    clock_t time_end;
    time_end = clock() + (time) * CLOCKS_PER_SEC;
    while (float(clock()+dt) < time_end)
    {
    }
}

string round_to_string(float number, int precision)
{
    // Round the number to the given precision
    float rounded_number = round(number * pow(10, precision)) / pow(10, precision);

    // Use a string stream to convert the number to a string with the specified precision
    ostringstream out;
    out << fixed << setprecision(precision) << rounded_number;
    return out.str();
}