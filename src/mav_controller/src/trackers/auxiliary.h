/* The following file holds a set of common definitions */
#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <cmath>
#include <algorithm>  //std::min
#include <vector>
#include <iostream>

float magnitude(const int &n, ...)
{
    va_list arglist;
    int j;
    double sum = 0;
 
    va_start(arglist, n); /* Requires the last fixed parameter (to get the address) */
    for (j = 0; j < n; j++) {
        sum += pow(va_arg(arglist, double),2); /* Increments ap to the next argument. */
    }
    va_end(arglist);
 
    return sqrt(sum);
}
#endif /*STATE_H*/