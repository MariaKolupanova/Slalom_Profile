////
//  Graph.cpp
//  Tree


#include "Graph.h"
#include <stack>

Graph::Graph(std::vector<std::vector<Vertex>>& vertices, Coord start_point, Coord goal_point, int n, int m, int road_len, int move_step, int max_deviation, double max_weight, double max_angle) :
        Start_Point(start_point), Goal_Point(goal_point), N(n), M(m), RoadLen(road_len), Move_Step(move_step), Max_Deviation(max_deviation), Max_Weight(max_weight), Max_Angle(max_angle)
{
int param = 0;
    Vertices.resize(2 * Max_Deviation * (2 * Move_Step + 1) * M +  (2*Max_Deviation-RoadLen + 1) * (2 * Move_Step + 1) + 1);
    for (auto y = 0; y < M; y++) {
        for (auto x = -Max_Deviation + Start_Point.x; x <= Max_Deviation + Start_Point.x - RoadLen; x++) {
            for (int i = 0; i <= 2*Move_Step; i++) {
                Coord current_vertex = { x,y,i };
                int current_index = GetIndex(current_vertex);
                for(int idx = std::min(-Move_Step + i,0); idx<= std::max(RoadLen,RoadLen - Move_Step + i); idx++){
                    Vertices[current_index].Weight += vertices[current_vertex.x + idx][current_vertex.y].Weight;
                    for(int thck = 0; thck <=3; thck ++){
                        Vertices[current_index].Categories[thck] += vertices[current_vertex.x + idx][current_vertex.y].Categories[thck];
                    }
                }
//                    if (Vertices[current_index].Weight != 0)
//                        std::cout <<current_vertex.x<<" "<< current_vertex.y<<" "<<current_vertex.edge<<" " << Vertices[current_index].Weight<< " "<<current_index << std::endl;
            }
        }
    }
//    for (int i = 0; i <= RoadLen; i++)
//        Vertices[i].Weight += vertices[Start_Point.x + i][0].Weight;

    //Vertices[2 * Max_Deviation * (2 * Move_Step + 1) * 1000 + (2 * Move_Step + 1) * Max_Deviation]= vertices[Goal_Point.x][Goal_Point.y];
}

unsigned int Graph::GetIndex(Coord v){
    return 2 * Max_Deviation * (2 * Move_Step + 1) * v.y + (2 * Move_Step + 1) * (v.x - Start_Point.x + Max_Deviation) + v.edge ;
}
std::vector<Coord> Graph::GetValidNeighbors(Coord v) {
    std::vector<Coord> neighbours;
    neighbours.clear();
    if(v.y == Start_Point.y && v.x == Start_Point.x){
        for (int i = 0; i <= 2 * Move_Step; i++) {
            neighbours.push_back({v.x - Move_Step + i, v.y + 1, i});
        }
        return neighbours;
    }
    if (v.y == Goal_Point.y && v.x == Goal_Point.x ) {
        neighbours.push_back({ Goal_Point.x,Goal_Point.y + 1,0 });
        return neighbours;
    }
    else{
        for (int i = 0; i <= 2 * Move_Step; i++) {
            double angle = ((Move_Step - v.edge) * (i - Move_Step) + N * N) /
                           (sqrt(pow((v.edge - Move_Step), 2) + pow(N, 2)) *
                            sqrt(pow((i - v.edge), 2) + pow(N, 2)));
            if (abs(angle > Max_Angle)) {
                neighbours.push_back({v.x - Move_Step + i, v.y + 1, i});
            }
        }
    }
    return neighbours;
}


double Graph::GetRoute(Coord move, Coord dest) {
        return abs(move.x - dest.x)/10;
}

std::vector<Coord> Graph::ConvertGraphToPath(Coord goalPoint)
{
//    for(int i = 0; i <=3; i++){
//        TotCategories[i] = 0;
//    }
    std::vector<Coord> result;
    result.clear();
    std::stack<Coord> tmp_path;
    unsigned int current_index = GetIndex(goalPoint);
    Coord current_vertex = goalPoint;
    Coord end = { INF,INF };
    ShapeTiffGridSettings st_model;
    route_len += pow((current_vertex.x - Vertices[current_index].CameFrom.x)/st_model.user_grid, 2) + pow(current_vertex.y - Vertices[current_index].CameFrom.y, 2);
    while (current_vertex.y > 0 && current_vertex !=end)
    {
        tmp_path.push(current_vertex);
        current_index = GetIndex(current_vertex);
        current_vertex = Vertices[current_index].CameFrom;

        if (Vertices[current_index].CameFrom != end) {
            double x = (current_vertex.x - Vertices[current_index].CameFrom.x)/st_model.user_grid;
            route_len += sqrt(abs(x * x) + 1);
            if(Vertices[current_index].Weight != 0){
                std::cout <<Vertices[current_index].Weight<<" "<<current_vertex.y<<std::endl;
                for (int i = 0; i <= 3; i++)
                    TotCategories[i] += Vertices[current_index].Categories[i];
            }
        }
    }
    std::cout <<Vertices[GetIndex(Start_Point)].Weight<<" "<<current_vertex.y<<std::endl;
     for (int i = 0; i <= 3; i++) {
             TotCategories[i] += Vertices[GetIndex(Start_Point)].Categories[i];
     }
    tmp_path.push(current_vertex);
    while (!tmp_path.empty())
    {
        result.push_back(tmp_path.top());

        tmp_path.pop();
    }

    return result;
}
std::vector<Coord>Graph::find_path_Dijkstra(Coord CameFrom)
{
    //сортировка по весу, результат - значение
    std::multimap<double, Coord> min_weigth_map;
    int start_index = GetIndex(Start_Point);
    std::cout << start_index<<std::endl;
    Vertices[start_index].Label = Vertices[start_index].Weight;
    Vertices[start_index].CameFrom = CameFrom;
    unsigned int goal_index = GetIndex({Goal_Point.x,M,0});
    min_weigth_map.insert({ Vertices[start_index].Label,Start_Point });
    while (!min_weigth_map.empty() && !Vertices[goal_index].IsVisited)
    {
        auto [current_weight, current_coord] = *(min_weigth_map.begin()); //минимальное значение
        min_weigth_map.erase(min_weigth_map.begin());
        unsigned int current_index = GetIndex(current_coord);
        if (Vertices[current_index].IsVisited)
        {
            continue;
        }
        Vertices[current_index].IsVisited = true;
        auto neighbors = GetValidNeighbors(current_coord);
        if(neighbors.size() == 0) continue;
        for (auto neighbor : neighbors)
        {
            int neighbor_index = GetIndex(neighbor);
            if (neighbor.x < Start_Point.x - Max_Deviation || neighbor.x > Start_Point.x - RoadLen +  Max_Deviation || neighbor.y > M || neighbor.y < 0 || neighbor_index > goal_index || neighbor_index < 0) continue;

            if (!Vertices[neighbor_index].IsVisited)
            {


                double next_label = current_weight + Vertices[neighbor_index].Weight;// + GetRoute(neighbor, Start_Point);
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
    return Vertices[GetIndex(c)].Label;
}

double Graph::GetStraightLabel(Coord start, Coord end)
{
    double label = 0;
    Coord current_coord = start;
    unsigned int current_index = GetIndex(current_coord);
    label += Vertices[current_index].Weight;
    while (current_coord.y < end.y) {
        current_coord.y += 1;
        current_index = GetIndex(current_coord);
        double w = Vertices[current_index].Weight;
        if (w < INF) label += w;
        else return 0;
    }
    label += Vertices[GetIndex(end)].Weight;
    return label;
}

