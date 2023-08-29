//
//  Graph.h
//  Tree
//
//  Created by Maria on 31.05.2023.
//

#ifndef Graph_h
#define Graph_h
#include <shape_tiff_model.h>

#include <vector>
#include <math.h>
#include <map>



class Graph {
public:
    Graph(std::vector<std::vector<Vertex>>& vertices, Coord start_point, Coord goal_point, int n, int m, int road_len, int move_step, int max_deviation, double max_weight, double max_angle);
    std::vector<Coord> GetValidNeighbors(Coord v);
    std::vector<Coord> find_path_Dijkstra(Coord CameFrom = { INF,INF });
    std::vector<Coord> ConvertGraphToPath(Coord goalPoint);
    //double GetWeight(Coord start, Coord end);
    //double GetDistance(Coord start, Coord end);
    unsigned int GetIndex(Coord v);
    double GetRoute(Coord move, Coord dest);
    double GetLabel(Coord c);
    double GetStraightLabel(Coord start, Coord end);
    std::vector<int> GetCategories() { return TotCategories; }
    double GetRouteLen() { return route_len / (M - 1); }
    ~Graph() = default;
private:
    // Количество вершин по оси Ox
    int N;
    // Количество вершин по оси Oy
    int M;
    int RoadLen;
    int Move_Step;
    int Max_Deviation;
    double Max_Weight;
    double Max_Angle;
    Coord Start_Point, Goal_Point;
    std::vector<Vertex> Vertices;
    std::vector<int> TotCategories = { 0,0,0,0 };
    double route_len = 0;
};

#endif /* Graph_h */
