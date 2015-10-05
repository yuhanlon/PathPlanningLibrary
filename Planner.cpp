//
//  Planner.cpp
//  planningLibrary
//  This is a definiation of the planner class.
//
//  Copyright © 2015 Yuhan Long. All rights reserved.
//

#include "Planner.hpp"

vector<Grid> Planner::aStarPlanning(Map &map)
{
    // implemtation of the a star algorithm
    
    // initialize the way point list
    vector<Grid> wayPoint;
    
    // return an empty list if map is invalid
    if(map.start.x==-1||map.start.y==-1||map.goal.x==-1||map.goal.y==-1)
        return wayPoint;
    
    
    // type of the A star priority list element
    typedef struct {Grid pos; float f;} AStarUnit;
    
    
    list<AStarUnit> openlist; // open list
    vector<vector<float> > h(map._rows, vector<float>(map._cols, -1.0));  // heuristic value
    vector<vector<float> > g(map._rows, vector<float>(map._cols, -1.0));  // optimized value of the grid
    vector<vector<float> > f(map._rows, vector<float>(map._cols, -1.0));  // the key value of the A star algorithm (search priority)
    vector<vector<Grid> > b(map._rows, vector<Grid>(map._cols, Grid(-1,-1)));  // back pointer to pred node
    vector<vector<bool> > closelist(map._rows, vector<bool>(map._cols, false));  // close list

    
    
    // value initialization
    h[map.start.y][map.start.x] = calculateDistance(map.start,map.goal);
    g[map.start.y][map.start.x] = 0;
    f[map.start.y][map.start.x] = h[map.start.y][map.start.x];
    
    AStarUnit startP={map.start,calculateDistance(map.start,map.goal)};
    openlist.push_back(startP);
    
    while (openlist.size()>0)
    {
        // iterate when the open list is not empty
        // pop the first item in the open list to expand
        Grid currentP  = openlist.front().pos;
        closelist[currentP.y][currentP.x] = true;
        openlist.pop_front();
        
        // if already reach the goal, stop iterating
        if (currentP==map.goal) break;
        
        // find its direct neigbour
        vector<Grid> neighbourList = findNeighbour(map,currentP,4);
        
        for(Grid neighbour:neighbourList)
        {
            
            // update the shorest distance of the cells neighbour if a shorter path is finded
            if (((g[neighbour.y][neighbour.x]==-1.0)||(g[neighbour.y][neighbour.x]>g[currentP.y][currentP.x]+calculateDistance(neighbour,currentP)))&&(!closelist[neighbour.y][neighbour.x]))
            {
                
                h[neighbour.y][neighbour.x] = calculateDistance(neighbour,map.goal);
                g[neighbour.y][neighbour.x] = g[currentP.y][currentP.x]+calculateDistance(neighbour,currentP);
                f[neighbour.y][neighbour.x] = h[map.start.y][map.start.x]+g[map.start.y][map.start.x];
                b[neighbour.y][neighbour.x] = currentP;
                float _f =f[map.start.y][map.start.x];
                
                // find the correct position to insert the cell into the priority list, according to f value
                auto insert_pos = find_if(openlist.begin(),openlist.end(),[_f](AStarUnit list_elem){return _f<list_elem.f;});
                openlist.insert(insert_pos,{neighbour,_f});
            }
            
        }
        
        
    }
    
    // form the waypoint list from the goal cell in the map
    deque<Grid> tempWayPoint;
    Grid trace = map.goal;
    while (trace!=Grid(-1.0,-1.0))
    {
        tempWayPoint.push_front(trace);
        
        trace = b[trace.y][trace.x];
    }
    
    wayPoint.assign(tempWayPoint.cbegin(),tempWayPoint.cend());
    
    
    return wayPoint;
}


Key Planner::calculateKey(Grid &grid, vector<vector<int> > &g, vector<vector<int> > &rhs, Grid &start,int km)
{
    // method to calculate the key value for grids.
    // it will pick the shortest path to the goal plus the distance to the start as the primery key
    // it will pick the shortest path to the goal to be the secondary key
    int minVal=g[grid.y][grid.x]>rhs[grid.y][grid.x]?rhs[grid.y][grid.x]:g[grid.y][grid.x];
    int key1 = minVal+calculateDistance(grid,start)+minVal+km;
    int key2 = minVal;
    
    return {key1,key2};
}

void Planner::updateVertex(list<Uelem> &U, Grid &u, vector<vector<int> > &g, vector<vector<int> > &rhs, Map &map,int km)
{
    // update the vertex value and update the priority queue
    if(u!=map.goal)
    {
        vector<Grid> neighbour = findNeighbour(map, u, 8);
        vector<float> dist;
        
        for_each(neighbour.begin(),neighbour.end(),[&](Grid &it){dist.push_back((float)g[it.y][it.x]+calculateDistance(it,u));});
        
        rhs[u.y][u.x]=*min_element(dist.begin(),dist.end());
    }
    
    auto pt =find_if(U.begin(),U.end(),[u](Uelem &pt){return pt.point==u;});
    if (pt!=U.end()) U.erase(pt);
    if (g[u.y][u.x]!=rhs[u.y][u.x])
    {
        Key knew = calculateKey(u,g,rhs,map.start,0);
        auto pt = find_if(U.begin(),U.end(),[&](Uelem &a){return a.key<knew;});
        U.insert(pt,{u,knew});
    }
}


vector<Grid> Planner::dStarLite(Map &map)
{
    // this is the implementation of the D star lite algorithm
    
    vector<Grid> wayPoint;

    
    // Initialization
    vector<vector<int> > g(map._rows,vector<int>(map._cols,map._rows*map._cols+1));
    vector<vector<int> > rhs(map._rows,vector<int>(map._cols,map._rows*map._cols+1));
    km =0;
    rhs[map.goal.y][map.goal.x] =0;
    Key n = calculateKey(map.goal,g,rhs,map.start,0);
    U.push_back({map.goal,n});
    
    
    // Computer current shortest path
    // Update the states of the map if the first element in the priority list can be updated or the goal is not reached
    while(U.front().key<calculateKey(map.goal,g,rhs,map.start,0)||(rhs[map.start.y][map.start.x]!=g[map.start.y][map.start.x]))
          {
              // take the key value and postion of the first element in the priority queue
              Key kold = U.front().key;
              Grid u = U.front().point;
              
              U.pop_front();
              Key knew = calculateKey(u,g,rhs,map.start,km); // calculate the new key value
              
              // update map if old value is different from the new value
              if (kold<knew)
              {
                  // if the new key is larger, the cost of the edge of the grid might be change
                  // the current grid should be updated and re-expanded
                  // insert it in the priority queue
                  auto pt =find_if(U.begin(),U.end(),[knew](Uelem &u){return u.key<knew;});
                  U.insert(pt,{u,knew});
              }
              else if (g[u.y][u.x]>rhs[u.y][u.x])
              {
                  // if the grid is overconstraint, there are new shorter paths detected
                  g[u.y][u.x] = rhs[u.y][u.x];
                  
                  // update all its neighbour value
                  vector<Grid> neightbour = findNeighbour(map, u, 8);
                  for(auto &n:neightbour)
                  {
                      updateVertex(U,n,g,rhs,map,km);
                  }
              }
              else
              {
                  // if the grid is underconstraint, the grid it self and its neightbour should all be updated
                  g[u.y][u.x] = map._cols*map._rows;
                  vector<Grid> neightbour = findNeighbour(map, u, 8);
                  for(auto &n:neightbour)
                  {
                      updateVertex(U,n,g,rhs,map,km);
                  }
                  updateVertex(U,u,g,rhs,map,km);
              }
              
                  
              
              
              
          }
    
    return wayPoint;
}



vector<Grid> Planner::findNeighbour(Map &map, Grid current, int type)
{
    // find the neighbour of current cell
    // type: 4:four neigbour expansion 8:eight neighbour expansion
    vector<Grid> neighbourList;
    vector<Grid> offset = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
    if(type==4)
        offset = {{-1,0},{0,-1},{0,1},{1,0}};
    for(auto offset_item:offset)
    {
        if (map.validGrid(offset_item+current))
            neighbourList.push_back(offset_item+current);
    }
    return neighbourList;
}

float Planner::calculateDistance(Grid current, Grid goal)
{
    // return the distance of two cells
    return sqrt(pow((current.x-goal.x),2)+pow((current.y-goal.y),2));
    
}