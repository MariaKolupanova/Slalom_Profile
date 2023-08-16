////
//  Graph.cpp
//  Tree


#include "Graph.h"
#include <stack>
#include <fstream>
#include <sstream>
#include <fstream>
Graph::Graph(std::vector<std::vector<Vertex>>& vertices, int n, int m, int road_len, int move_step, int max_deviation, double max_weight, double max_angle) :
    N(n), M(m), RoadLen(road_len), Move_Step(move_step), Max_Deviation(max_deviation), Max_Weight(max_weight), Max_Angle(max_angle)
{
    Coord startPoint = { 100,0 };
    Coord finishPoint = { 100,M - 1 };
    Vertices.resize(2 * Max_Deviation * (2 * Move_Step + 1) * (M + 1) + 1);
    std::cout << 2 * Max_Deviation * (2 * Move_Step + 1) * (M + 2) + 1 << std::endl;
    for (auto y = 1; y < M; y++) {
        for (auto x = -Max_Deviation + startPoint.x; x <= Max_Deviation + startPoint.x - RoadLen; x++) {

            for (int i = 0; i <= Move_Step; i++) {
                Coord current_vertex = {x, y, i};
                int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * current_vertex.y +
                                    (2 * Move_Step + 1) * (current_vertex.x - startPoint.x + Max_Deviation) +
                                    current_vertex.edge;
                if (current_vertex.edge == 0) {
                    for (int idx = 0; idx <= RoadLen; idx++) {
                        if (current_vertex.y > 0)
                            Vertices[current_index].Weight += vertices[current_vertex.x + idx][current_vertex.y -
                                                                                               1].Weight;
                        if (current_vertex.y < 999)
                            Vertices[current_index].Weight += vertices[current_vertex.x + idx][current_vertex.y].Weight;
                    }
                } else {

                    for (int idy = 0; idy < i; idy++) {
                        if (y >= idy) {
                            for (int idx = -1; idx <= RoadLen; idx++) {
                                Vertices[current_index].Weight += vertices[current_vertex.x + idx][current_vertex.y -
                                                                                                   idy].Weight;
                            }
                        }
                    }
                }
            }
            for (int i = Move_Step + 1; i <= 2 * Move_Step; i++) {
                Coord current_vertex = {x, y, i};
                int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * current_vertex.y +
                                    (2 * Move_Step + 1) * (current_vertex.x - startPoint.x + Max_Deviation) +
                                    current_vertex.edge;
                for (int idy = 0; idy < i - Move_Step; idy++) {
                    if (y >= idy) {
                        for (int idx = 0; idx <= RoadLen + 1; idx++) {
                            Vertices[current_index].Weight += vertices[current_vertex.x + idx][current_vertex.y -
                                                                                               idy].Weight;
                        }
                    }
                }
            }
        }
        }

    for (int i = 0; i <= RoadLen; i++)
        Vertices[i].Weight += vertices[startPoint.x + i][0].Weight;
    Vertices[2 * Max_Deviation * (2 * Move_Step + 1) * 1000 + (2 * Move_Step + 1) * Max_Deviation] = vertices[finishPoint.x][finishPoint.y];
}

std::vector<Coord> Graph::GetValidNeighbors(Coord v) {
    std::vector<Coord> neighbours;
    if (v.y == 0 && v.x == 100) {
        neighbours.push_back({ 100,-1,0 });
        return neighbours;
    }
    if (v.y == M-1) {
        neighbours.push_back({ v.x, v.y - 1, 0 });
        for (int i = 1; i < Move_Step; i++) {
            neighbours.push_back({ v.x - 1, v.y - i, i });
        }
        for (int i = Move_Step + 1; i <= 2 * Move_Step; i++) {
            neighbours.push_back({ v.x + 1, v.y - i + Move_Step, i });
        }
    }
    else if (v.edge == 0) {
        neighbours.push_back({ v.x,v.y - 1, 0 });
        if (Move_Step >= 4) {
            for (int i = 4; i <= Move_Step; i++)
                neighbours.push_back({ v.x - 1, v.y - i, i });
            for(int i = Move_Step + 4; i <= 2*Move_Step;i++)
                neighbours.push_back({ v.x + 1, v.y - i + Move_Step, i  });

        }
    }
    else if (v.edge == 1) {
        neighbours.push_back({ v.x - 1, v.y - 1, 1 });
        neighbours.push_back({ v.x - 1, v.y - 2, 2  });
    }
    else if (v.edge == Move_Step + 1) {
        neighbours.push_back({ v.x + 1, v.y - 1, 1 + Move_Step });
        neighbours.push_back({ v.x + 1, v.y - 2, 2 + Move_Step });
    }
    else if (v.edge == Move_Step) {
        neighbours.push_back({ v.x , v.y - 1, 0 });
        neighbours.push_back({ v.x - 1, v.y - Move_Step,  Move_Step });
        neighbours.push_back({ v.x - 1, v.y - Move_Step + 1, Move_Step - 1 });
        if (Move_Step >= 6) {
            for (int i = 0; i <= Move_Step - 6; i++)
                neighbours.push_back({ v.x + 1, v.y - Move_Step + i, 2*Move_Step - i });
        }
    }
    else if (v.edge == 2 * Move_Step) {
        neighbours.push_back({ v.x , v.y - 1, 0 });
        neighbours.push_back({ v.x + 1, v.y - Move_Step, 2*Move_Step });
        neighbours.push_back({ v.x + 1, v.y - Move_Step + 1, 2*Move_Step - 1 });
        if (Move_Step >= 6) {
            for (int i = 0; i <= Move_Step - 6; i++)
                neighbours.push_back({ v.x - 1, v.y - Move_Step + i,  Move_Step - i });
        }
    }
    else {
        if (v.edge > Move_Step) {
            neighbours.push_back({ v.x + 1, v.y - v.edge + Move_Step, v.edge });
            neighbours.push_back({ v.x + 1, v.y - v.edge + Move_Step - 1, v.edge + 1 });
            neighbours.push_back({ v.x + 1, v.y - v.edge + Move_Step + 1, v.edge - 1 });
        }
        else {
            neighbours.push_back({ v.x - 1, v.y - v.edge , v.edge });
            neighbours.push_back({ v.x - 1, v.y - v.edge - 1, v.edge + 1 });
            neighbours.push_back({ v.x - 1, v.y - v.edge + 1, v.edge - 1 });
        }

    }

    return neighbours;
}

std::vector<Coord> Graph::ConvertGraphToPath(Coord goalPoint)
{
    std::vector<Coord> result;
    result.clear();
    std::stack<Coord> tmp_path;
    Coord current_vertex = goalPoint;
    unsigned int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * 1000 + (2 * Move_Step + 1) * Max_Deviation;
    Coord end = { INF,INF };
    while (current_vertex.y < M-1)
    {

        tmp_path.push(current_vertex);
        current_vertex = Vertices[current_index].CameFrom;

        current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * (999-current_vertex.y) + (2 * Move_Step + 1) * (current_vertex.x - 100 + Max_Deviation) + current_vertex.edge;
        if (Vertices[current_index].CameFrom != end) {
            int y =  Vertices[current_index].CameFrom.y - current_vertex.y;
            int x = current_vertex.x - Vertices[current_index].CameFrom.x;
            route_len += sqrt(abs(x * x) + y * y);
        }
    }
    while (!tmp_path.empty())
    {
        result.push_back(tmp_path.top());

        tmp_path.pop();
    }

    return result;
}


std::vector<Coord>Graph::find_path_Dijkstra(Coord startPoint, Coord goalPoint, Coord CameFrom)
{

    //сортировка по весу, результат - значение
    std::multimap<double, Coord> min_weigth_map;
    // Vertices[startPoint.x][startPoint.y].Weight = GetWeight(startPoint, startPoint);
    int start_index = (2 * Move_Step + 1) * Max_Deviation;
    Vertices[start_index].Label = Vertices[start_index].Weight;// (startPoint, startPoint);
    Vertices[start_index].CameFrom = CameFrom;
    unsigned int goal_index = 2 * Max_Deviation * (2 * Move_Step + 1) * 1000 + (2 * Move_Step + 1) * Max_Deviation;
  // std::cout << goal_index << std::endl;
    min_weigth_map.insert({ 0,startPoint });
    while (!min_weigth_map.empty() && !Vertices[goal_index].IsVisited)
    {
        auto [current_weight, current_coord] = *(min_weigth_map.begin()); //минимальное значение
        min_weigth_map.erase(min_weigth_map.begin());
        unsigned int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * (999-current_coord.y) + (2 * Move_Step + 1) * (100-current_coord.x +Max_Deviation) + current_coord.edge;
        if (Vertices[current_index].IsVisited)
        {
            continue;
        }
        Vertices[current_index].IsVisited = true;
        auto neighbors = GetValidNeighbors(current_coord);
        for (auto neighbor : GetValidNeighbors(current_coord))
        {
            int neighbor_index = 2 * Max_Deviation * (2 * Move_Step + 1) *(999- neighbor.y) + (2 * Move_Step + 1) * (neighbor.x - startPoint.x + Max_Deviation) + neighbor.edge;
            if (neighbor.x < startPoint.x - Max_Deviation || neighbor.x > startPoint.x - RoadLen +  Max_Deviation || neighbor.x < startPoint.x  ||  neighbor_index > goal_index || neighbor_index < 0) continue;
            if (!Vertices[neighbor_index].IsVisited)
            {


                double next_label = current_weight + Vertices[neighbor_index].Weight;
                if (next_label < Vertices[neighbor_index].Label)
                {
                    Vertices[neighbor_index].Label = next_label;
                    Vertices[neighbor_index].CameFrom = current_coord;
                    min_weigth_map.insert({ next_label, neighbor });
                }

            }
        }
    }
    return ConvertGraphToPath(Vertices[goal_index].CameFrom);
}
double Graph::GetLabel(Coord c)
{
    unsigned int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * (999-c.y) + (2 * Move_Step + 1) * (c.x - 100 + Max_Deviation) + c.edge ;
    return Vertices[current_index].Label;
}

