//
//  Map.hpp
//  planningLibrary
//  This is the declaration of the basic objects in a navigation algorithm
//  The objects includes the map cell and the map
//
//  Copyright © 2015 Yuhan Long. All rights reserved.
//

#ifndef Map_hpp
#define Map_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
using namespace std;



// map type that defines accessble/inaccessable areas in the map
typedef vector<vector<bool> > MapGrid;

struct Grid
{
    //This is the class for a grid in the map.
    
public:
    int x; //X location of the grid
    int y; //Y location of the grid
    Grid() = default; // default constructor
    Grid(int x, int y):x(x),y(y){}; //constructor with x,y location
    
    // sum arithmetic of two Grid objects
    friend const Grid operator+(const Grid &lhs, const Grid &rhs) {return Grid(lhs.x+rhs.x,lhs.y+rhs.y);};
    // equal arithmetic of two Grid objects
    friend const bool operator==(const Grid &lhs, const Grid &rhs) {return ((lhs.x == rhs.x)&&(lhs.y==rhs.y));};
    // inequal arithmetic of two Grid objects
    friend const bool operator!=(const Grid &lhs, const Grid &rhs) {return !(lhs==rhs);};
    // output stream overload of the Grid objects
    friend ostream & operator<<(ostream &os, Grid &g){os<<g.x<<g.y;return os;};
    
};

class Map
{
    // This is a class for the map to navigate
public:
    // defautl constructor
    Map()=default;
    // constructer with size of the map
    Map(int rows, int cols);
    // The obstacle layouts of the map
    MapGrid map;
    
    
    Grid start;  // Start postion of the navigation
    Grid goal;  // Goal position of the navigation
    
    void addObstacle(Grid topleft, Grid botright);  // method for adding obstacle blocks in the map
    inline
    void setStart(Grid start) {this->start =start;};  // method for adding start location in the map
    inline
    void setGoal(Grid goal) {this->goal=goal;};  // method for setting goal location in the map
    bool validGrid(Grid current);  //method for checking if the current block is valid to access
    
    int _rows;  // rows of the map
    int _cols;  // cols of the map

    friend ostream & operator<<(ostream & os, Map & m); // method for showing the navigation map


};


#endif /* Map_hpp */
