#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "tools.h"

const int N = 10;
const double dt = 0.05;
const double Lf = 2.67;
const double latency = 0.12;

double ref_v = tools::mphtoms(40);


const double cte_factor = 1.0;
const double epsi_factor = 1.0;
const double v_factor = 1.0;
const double delta_factor = 100.0;
const double a_factor = 1.0;
const double delta_diff_factor = 400.0;
const double a_diff_factor = 1.0;

#endif
