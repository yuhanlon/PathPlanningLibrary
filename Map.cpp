//
//  Map.cpp
//  planningLibrary
//  This is the definition of the basic object classes in the navigation planning algorithm
//
//  Copyright © 2015 Yuhan Long. All rights reserved.
//

#include "Map.hpp"



Map::Map(int rows, int cols):
    _rows(rows),
    _cols(cols),
    start({-1,-1}),
    goal({-1,-1})

{
    // constructor that initialize the map
    map.resize(rows,vector<bool>(cols,false));
}

void Map::addObstacle(Grid topleft, Grid botright)
{
    // add obstatacle cells in the map.
    // topleft: top-left location of the obstacle block
    // botright: bottom-right location of the obstacle block
    
    size_t x1 =topleft.x;
    size_t y1 =topleft.y;
    
    size_t x2 =botright.x;
    size_t y2 =botright.y;
    
    for(size_t row = y1; row<=y2; ++row)
        for(size_t col = x1; col<=x2; ++col)
            map[row][col]=true;
}

bool Map::validGrid(Grid current)
{
    // method that checks if the block is available to be accessed
    // A valided block should be within the boundary of the map and should not be a obstacle grid
    if ((current.x<_cols)&&(current.x>=0)&&(current.y<_rows)&&(current.y>=0)&&(!map[current.y][current.x]))
        return true;
    else
        return false;
}

ostream & operator<<(ostream & os, Map & m)
{
    // output function to visualize the cells.
    for(int i=0;i<m._rows;++i)
    {
        for(int j=0;j<m._cols;++j)
        {
            if(m.start==Grid(j,i))
            {
                os<<'s';
            }
            else
            {
                if(m.goal == Grid(j,i))
                {
                    os<<'g';
                }
                else
                {
                    if(m.map[i][j])
                        cout<<'x';
                    else
                        cout<<'o';
                }
                
                
            }
        }
        os<<endl;
    }
    return os;
}