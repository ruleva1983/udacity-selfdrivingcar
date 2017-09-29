#ifndef UTILS_H
#define UTILS_H

#include <vector>
//#include "matplotlibcpp.h"


//using namespace matplotlibcpp;

using plot_type = std::vector<std::pair<std::vector<double>, std::vector<double>>>;
using plot_type_ref = std::vector<std::pair<std::vector<double>, std::vector<double>>>&;

/*
namespace plot{
    void plot_objects(plot_type objects){
        
        for (auto o : objects)
            matplotlibcpp::plot(o.first, o.second);
        matplotlibcpp::show();
    }
}

*/
namespace utils{

double mph_to_ms(double v){return v*0.44704;}
double ms_to_mph(double v){return v/0.44704;}

}

#endif
