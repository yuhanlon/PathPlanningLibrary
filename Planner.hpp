//
//  Planner.hpp
//  planningLibrary
//  This is the declaration of the planner class
//
//  Copyright © 2015 Yuhan Long. All rights reserved.
//

#ifndef Planner_hpp
#define Planner_hpp

#include <stdio.h>
#include <vector>
#include <deque>
#include <list>
#include <cmath>
#include "Map.hpp"
#include <queue>

using namespace std;

struct Key
{
    // this is the Key class in the d star lite algorithm.
    // the key value is used to pioritize the next grid the algorithm will expand
    
    int key1;
    int key2;
    
    
    friend const bool operator<(const Key & lhs, const Key & rhs)
    {
        // overloaded compare operator, the compare will based on the first key first and then check the second key
        if(lhs.key1<rhs.key1)
            return true;
        else
        {
            if ((lhs.key1==rhs.key1)&&(lhs.key2<rhs.key2))
                return true;
            else
                return false;
        }
    }
};


// prioirty queue element type, include a the location of the grid and the key value of the grid
typedef struct
{
    Grid point;
    Key key;
} Uelem;




class Planner
{
    // Planner class:
    // Two path planning algorithms are implemented
    // 1. A star path planning
    // 2. D star lite path panning
    // Life Long A star planning is to be installed in the future
public:
    
    // default constructor of the planner
    Planner()=default;
    
    // A star planning algorithm
    vector<Grid> aStarPlanning(Map &map);
    
    //[TODO] life long A star planning algorithm
    vector<Grid> lifelongAStarPlanning(Map &m);
    
    // D start lite planning algorithm
    vector<Grid> dStarLite(Map &map);
    
    // Method for key calculation in D star lite algorithm
    Key calculateKey(Grid &grid, vector<vector<int> > &g, vector<vector<int> > &rhs, Grid &start,int km);
   
    // vertex value update function in D star lite algorithm
    void updateVertex(list<Uelem> &U, Grid &u, vector<vector<int> > &g, vector<vector<int> > &rhs, Map &map,int km);
    
    // acquire valid neighbout around current cell
    vector<Grid> findNeighbour(Map &map, Grid current,int type);
    
    // distance calculation between to grid on the map
    static float calculateDistance(Grid current, Grid goal);
    
    // compensate step value in D star lite algorithm for dynamical planning
    int km;
    
    
private:
    list<Uelem> U; // Priority list in D star Lite algorithm

    
};

#endif /* Planner_hpp */
