#ifndef MAP_H
#define MAP_H

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

#include "spline.h"


using namespace std;


const double MaxS = 6945.554;
const double LaneWidth = 4.0;
const int nb_lanes = 3;


struct Waypoints{
    
    Waypoints();
    Waypoints(const string& map_file);

    int nb_waypoints;
    vector<double> x;
    vector<double> y;
    vector<double> s;
    vector<double> d_x;
    vector<double> d_y;
};



class Map{
public:
    
    /// \brief Parametrized Constructor
    /// \param map_file a string containing the path to the file of the map waypoints
    Map(const string& map_file);
    
    /// \brief Trasformation from the Frenet coordinates system (s, d) to the cartesian coordinate system (x, y)
    /// \param s the s component of the Frenet coordinate
    /// \param d the d component of the Frenet coordinate
    /// \return the (x,y) map coordinates, as a pair
    std::pair<double, double> frenet_to_xy(double s, double d);
    
    /// \brief Given the d Frenet coordinate indentifies in which lane the vehicle is currently residing.
    /// \param d the d Frenet coordinate
    /// \return The lane number as an integer
    int current_lane(float d) const;
    
    /// \brief Static function. Evaluates the distance between two vehicles given their s Frenet coodinates.
    ///        It assumes that the vehicles are in the same lane.
    /// \param s1 the s component of the Frenet coordinate for first vehicle
    /// \param s2 the s component of the Frenet coordinate for second vehicle
    /// \return the (x,y) map coordinates, as a pair
    static double distance(double s1, double s2){
        double dist = s1 - s2;
        if (dist < -0.5 * MaxS){
            dist += MaxS;
        }
        else if (dist > 0.5 * MaxS){
            dist -= MaxS;
        }

        return dist;
    }
    
    int size(){
        return raw_points.nb_waypoints;
    }
    
    
    
private:
    void eval_spline();
    
    // Member variables
    Waypoints raw_points;
    tk::spline spline_x_;
    tk::spline spline_y_;
    tk::spline spline_dx_;
    tk::spline spline_dy_;
};



Waypoints::Waypoints(): x(), y(), s(), d_x(), d_y(){}
Waypoints::Waypoints(const string& map_file){
    int nb_waypoints = 0;
    ifstream in_map_(map_file.c_str(), ifstream::in);
    
    string line;
    while (std::getline(in_map_, line))
        {
        istringstream iss(line);
        double x_i, y_i, s_i, d_x_i, d_y_i;
        iss >> x_i >> y_i >> s_i >> d_x_i >> d_y_i;
        x.push_back(x_i);
        y.push_back(y_i);
        s.push_back(s_i);
        d_x.push_back(d_x_i);
        d_y.push_back(d_y_i);
        nb_waypoints++;
        }
    }

Map::Map(const string& map_file){
        raw_points = Waypoints(map_file);
        eval_spline();
    }

void Map::eval_spline(){
        raw_points.s.push_back(MaxS);
        raw_points.x.push_back(raw_points.x[0]);
        raw_points.y.push_back(raw_points.y[0]);
        raw_points.d_x.push_back(raw_points.d_x[0]);
        raw_points.d_y.push_back(raw_points.d_y[0]);

        spline_x_.set_points(raw_points.s, raw_points.x);
        spline_y_.set_points(raw_points.s, raw_points.y);
        spline_dx_.set_points(raw_points.s, raw_points.d_x);
        spline_dy_.set_points(raw_points.s, raw_points.d_y);
    }

std::pair<double, double> Map::frenet_to_xy(double s, double d){
        double x = spline_x_(s) + d*spline_dx_(s);
        double y = spline_y_(s) + d*spline_dy_(s);
        return std::make_pair(x, y);
    }

int Map::current_lane(float d) const{
        return std::floor(d / LaneWidth);
    }
    






#endif
