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

void print_progressbar(int progress, int total) {
    int bar_width = 50;  // progress bar length
    float percentage = (float)progress / total;
    
    cout << "[";
    int pos = bar_width * percentage;
    for (int i = 0; i < bar_width; ++i) {
        if (i < pos) cout << "=";
        else if (i == pos) cout << ">";
        else cout << " ";
    }
    cout << "] " << int(percentage * 100.0) << " %\r";
    cout.flush();
}