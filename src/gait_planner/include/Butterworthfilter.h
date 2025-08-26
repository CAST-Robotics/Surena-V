#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

using namespace std;

class ButterworthFilter
{
public:
    ButterworthFilter(){}
    void setfilterparameters(double wd, double ws);
    double FilterData(double wd, double ws, double unfiltered_data);

private:
    double dt;
    std::vector<double> data_temp{0, 0, 0};
    std::vector<double> data_filtered_temp{0, 0};
    double C;
    double D;
    double a0;
    double a1;
    double a2;

    double b0;
    double b1;
    double b2;

    double filtered_data;
};