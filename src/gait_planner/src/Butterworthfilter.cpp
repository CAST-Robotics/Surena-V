#include "Butterworthfilter.h"



void ButterworthFilter::setfilterparameters(double wd, double ws)
{
    dt = 1 / ws;
  
    C = tan(wd * dt / 2);
    D = 1 + sqrt(2) * C + C * C;
    a0 = 1;
    a1 = 2 * (C * C - 1) / D;
    a2 = (1 - sqrt(2) * C + C * C) / D;

    b0 = C * C / D;
    b1 = 2 * b0;
    b2 = b0;
}

double ButterworthFilter::FilterData(double wd, double ws, double unfiltered_data)
 {
    
    setfilterparameters(wd, ws);

    filtered_data = b0 * data_temp[2] + b1 * data_temp[1] + b2 * data_temp[0] - a1 * data_filtered_temp[1] - a2 * data_filtered_temp[0];
    
    data_temp = {data_temp[1], data_temp[2], unfiltered_data};
    data_filtered_temp = {data_filtered_temp[1], filtered_data};

    return filtered_data;
}