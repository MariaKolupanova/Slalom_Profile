////
//  Graph.cpp
//  Tree


#include "Graph.h"
#include <stack>
#include <fstream>
#include <sstream>
#include <fstream>
Graph::Graph(std::vector<std::vector<Vertex>>& vertices, Coord start_point, Coord goal_point, int n, int m, int road_len, int move_step, int max_deviation, double max_weight, double max_angle) :
        Start_Point(start_point), Goal_Point(goal_point), N(n), M(m), RoadLen(road_len), Move_Step(move_step), Max_Deviation(max_deviation), Max_Weight(max_weight), Max_Angle(max_angle)
{

    Vertices.resize(2 * Max_Deviation * (2 * Move_Step + 1) * M +  (2*Max_Deviation-RoadLen + 1) * (2 * Move_Step + 1) + 1);
    for (auto y = 1; y < M; y++) {
        for (auto x = -Max_Deviation + Start_Point.x; x <= Max_Deviation + Start_Point.x - RoadLen; x++) {
            for (int i = 0; i <= 2*Move_Step; i++) {
                Coord current_vertex = { x,y,i };
                int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * current_vertex.y + (2 * Move_Step + 1) * (current_vertex.x - Start_Point.x + Max_Deviation) + current_vertex.edge;
                for(int idx = std::min(-Move_Step + i,0); idx<= std::max(RoadLen,RoadLen - Move_Step + i); idx++){
                        Vertices[current_index].Weight += vertices[current_vertex.x + idx][current_vertex.y].Weight;
                }

//                    if (Vertices[current_index].Weight != 0)
//                        std::cout <<current_vertex.x<<" "<< current_vertex.y<<" "<<current_vertex.edge<<" " << Vertices[current_index].Weight << std::endl;
            }
        }
    }
    for (int i = 0; i <= RoadLen; i++)
        Vertices[i].Weight += vertices[Start_Point.x + i][0].Weight;

    //Vertices[2 * Max_Deviation * (2 * Move_Step + 1) * 1000 + (2 * Move_Step + 1) * Max_Deviation]= vertices[Goal_Point.x][Goal_Point.y];
}
std::vector<Coord> Graph::GetValidNeighbors(Coord v) {
    std::vector<Coord> neighbours;
    neighbours.clear();
    if (v.y == Goal_Point.y && v.x == Goal_Point.x ) {
        neighbours.push_back({ Goal_Point.x,Goal_Point.y + 1,0 });
        return neighbours;
    }
    if(v.y < Goal_Point.y) {
        for (int i = 0; i <= 2 * Move_Step; i++) {
            double angle = ((Move_Step - v.edge) * (i - Move_Step) + N * N) /
                           (sqrt(pow((v.edge - Move_Step), 2) + pow(N, 2)) *
                            sqrt(pow((i - v.edge), 2) + pow(N, 2)));
            if (abs(angle > 0.97)) {
                neighbours.push_back({v.x - Move_Step + i, v.y + 1, i});
            }
        }
    }
    return neighbours;
}


double Graph::GetRoute(Coord move, Coord dest) {
        return abs(move.x - dest.x)/10;
}
//
//double Graph::GetDistance(Coord start, Coord finish) {
//    Coord current_vertex = start;
//    Coord prev_vertex = Vertices[start.x][start.y].CameFrom;
//    Coord movement = finish - start;
//    Coord end = { INF,INF };
//    if (start - prev_vertex != movement && prev_vertex != end) {
//        Coord diff_cur = movement;
//        Coord diff_prev = start - Vertices[start.x][start.y].CameFrom;
//        double angle = (diff_prev.x * diff_cur.x + diff_prev.y * diff_cur.y) /
//            (sqrt(pow(diff_prev.x, 2) + pow(diff_prev.y, 2)) *
//                sqrt(pow(diff_cur.x, 2) + pow(diff_cur.y, 2)));
//        if (angle > Max_Angle)
//            return  Max_Weight * (1 - angle);
//        else {
//            return INF_weight;
//        }
//    }
//    return 0;
//}



std::vector<Coord> Graph::ConvertGraphToPath(Coord goalPoint)
{
    std::vector<Coord> result;
    result.clear();
    std::stack<Coord> tmp_path;
    unsigned int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * goalPoint.y + (2 * Move_Step + 1) * (goalPoint.x - Start_Point.x + Max_Deviation);
    Coord current_vertex = goalPoint;
    Coord end = { INF,INF };
    //    for (int i = 0; i <= 3; i++) {
    //        for (int j = 0; j <= RoadLen; j++) {
    //            TotCategories[i] += Vertices[current_vertex.x + j][M - 1].Categories[i];
    //        }
    //    }
      //  route_len += pow(current_vertex.x - Vertices[current_vertex.x][current_vertex.y].CameFrom.x, 2) + pow(current_vertex.y - Vertices[current_vertex.x][current_vertex.y].CameFrom.y, 2);
    while (current_vertex.y > 0 && current_vertex !=end)
    {

        tmp_path.push(current_vertex);
        current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * current_vertex.y + (2 * Move_Step + 1) * (current_vertex.x - Start_Point.x + Max_Deviation) + current_vertex.edge;
        current_vertex = Vertices[current_index].CameFrom;
        if (Vertices[current_index].CameFrom != end) {
            int y = current_vertex.y - Vertices[current_index].CameFrom.y;
            int x = current_vertex.x - Vertices[current_index].CameFrom.x;
            route_len += sqrt(abs(x * x) + y * y);
            //std::cout << x*x << " " << y*y <<std::endl;
            // if(GetWeight(Vertices[current_vertex.x][current_vertex.y].CameFrom, current_vertex)!= 0)

//            for (int i = 0; i <= 3; i++) {
//                for (int idx = 0; idx < y; idx++) {
//                    for (int j = std::min(current_vertex.x, Vertices[current_vertex.x][current_vertex.y].CameFrom.x); j <= std::max(current_vertex.x, Vertices[current_vertex.x][current_vertex.y].CameFrom.x) + RoadLen; j++) {
//                        TotCategories[i] += Vertices[j][Vertices[current_vertex.x][current_vertex.y].CameFrom.y + idx].Categories[i];
//                        // if(Vertices[j][Vertices[current_vertex.x][current_vertex.y].CameFrom.y + idx].Weight !=0 ){
//                        //     std::cout << Vertices[j][Vertices[current_vertex.x][current_vertex.y].CameFrom.y + idx].Weight <<" "<<j<<" "<<Vertices[current_vertex.x][current_vertex.y].CameFrom.y + idx<<std::endl;
//                        // }
//
//                    }
//
//                }
//            }
        }
    }
    // for (int i = 0; i <= 3; i++) {
    //     for (int j = 0; j <= RoadLen; j++){
    //         TotCategories[i] += Vertices[current_vertex.x + j][0].Categories[i];
    //         std::cout << Vertices[current_vertex.x + j][0].Weight <<" "<<current_vertex.x + j<<" "<<0<<std::endl;
    //     }
    // }
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
    // Vertices[startPoint.x][startPoint.y].Weight = GetWeight(startPoint, startPoint);
    int start_index = (2 * Move_Step + 1) * Max_Deviation + Move_Step;
    Vertices[start_index].Label = Vertices[start_index].Weight;// (startPoint, startPoint);
    Vertices[start_index].CameFrom = CameFrom;
    unsigned int goal_index = 2 * Max_Deviation * (2 * Move_Step + 1) * M + (2 * Move_Step + 1) * (Goal_Point.x - Start_Point.x + Max_Deviation);
  // std::cout << goal_index << std::endl;
    min_weigth_map.insert({ 0,Start_Point });
    while (!min_weigth_map.empty() && !Vertices[goal_index].IsVisited)
    {
        auto [current_weight, current_coord] = *(min_weigth_map.begin()); //минимальное значение
        min_weigth_map.erase(min_weigth_map.begin());
        unsigned int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * current_coord.y + (2 * Move_Step + 1) * (Start_Point.x - current_coord.x + Max_Deviation) + current_coord.edge;
        if (Vertices[current_index].IsVisited)
        {
            continue;
        }
        Vertices[current_index].IsVisited = true;
        auto neighbors = GetValidNeighbors(current_coord);
        if(neighbors.size() == 0) continue;
        if(current_coord.y == 1000){
            std::cout <<"problem";
        }
        for (auto neighbor : neighbors)
        {
            int neighbor_index = 2 * Max_Deviation * (2 * Move_Step + 1) * neighbor.y + (2 * Move_Step + 1) * (neighbor.x - Start_Point.x + Max_Deviation) + neighbor.edge;
            if (neighbor.x < Start_Point.x - Max_Deviation || neighbor.x > Start_Point.x - RoadLen +  Max_Deviation || neighbor.y > M || neighbor.y < 0 || neighbor_index > goal_index || neighbor_index < 0) continue;
//            if(neighbor.y == 1000){
//                std::cout <<"problem";
//            }
            if (!Vertices[neighbor_index].IsVisited)
            {


                double next_label = current_weight + Vertices[neighbor_index].Weight;// + GetRoute(neighbor, Start_Point);// GetWeight(current_coord, neighbor) + GetDistance(current_coord, neighbor) +GetRoute(neighbor, goalPoint);
                if (next_label < Vertices[neighbor_index].Label)
                {
                    //std::cout << "x: " << neighbor.x << " y: " << neighbor.y << " edge: " << neighbor.edge << std::endl;
                   // std::cout << next_label << " " << neighbor_index << std::endl;
                    Vertices[neighbor_index].Label = next_label;
                    Vertices[neighbor_index].CameFrom = current_coord;
                    min_weigth_map.insert({ next_label, neighbor });
                }

            }
        }
    }
    //    Vertices.back().Label += Vertices.back().Weight;//GetWeight(goalPoint, goalPoint);
        std::cout << Vertices[goal_index].CameFrom.y << std::endl;
    std::cout << Vertices[goal_index].Label << std::endl;

    return ConvertGraphToPath(Vertices[goal_index].CameFrom);
}
double Graph::GetLabel(Coord c)
{
    unsigned int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * c.y + (2 * Move_Step + 1) * (c.x - Start_Point.x + Max_Deviation) + c.edge ;
    return Vertices[current_index].Label;
}
//
//double Graph::GetStraightLabel(Coord start, Coord end)
//{
//    double label = 0;
//    Coord current = start;
//    // label += GetWeight(start, start);
//    while (current.y < end.y) {
//        current.y += 1;
//        //current.x += 1;
//        double w = GetWeight({ current.x ,current.y - 1 }, current);
//        if (w < INF) label += w;
//        else return 0;
//    }
//    label += GetWeight(end, end);
//    return label;
//}
//
