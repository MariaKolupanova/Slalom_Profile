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
    //std::cout << std::endl;
    for (auto y = 1; y < M; y++) {
        for (auto x = -Max_Deviation + startPoint.x; x <= Max_Deviation + startPoint.x - RoadLen; x++) {
              //std::cout << value << std::endl;
                  //Vertices[value] = vertices[startPoint.x - Max_Deviation + x][y] + vertices[startPoint.x - Max_Deviation + x][y - 1];
            for (int i = 0; i <= Move_Step; i++) {
                Coord current_vertex = { x,y,i };
                int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * current_vertex.y + (2 * Move_Step + 1) * (current_vertex.x - startPoint.x + Max_Deviation) + current_vertex.edge;
                if (current_vertex.edge == 0) {
                    for (int idx = 0; idx <= RoadLen; idx++) {
                        if(current_vertex.y > 0)
                            Vertices[current_index].Weight += vertices[current_vertex.x + idx][current_vertex.y - 1].Weight;
                        if(current_vertex.y < 999)
                        Vertices[current_index].Weight += vertices[current_vertex.x + idx][current_vertex.y ].Weight;
                    }
                }
                else {

                    for (int idy = 0; idy < i; idy++) {
                        if (y >= idy)
                            for (int idx = -1; idx <= RoadLen; idx++) {

                                Vertices[current_index ].Weight += vertices[current_vertex.x + idx][current_vertex.y - idy].Weight;

                            }
                    }
                }
                if (Vertices[current_index].Weight != 0)
                    std::cout  << current_vertex.x << " " << 1000-current_vertex.y << " " << current_vertex.edge << " " << Vertices[current_index].Weight << std::endl;
            }
                for (int i = Move_Step + 1; i <= 2 * Move_Step; i++) {
                    
                    Coord current_vertex = { x,y,i };
                    int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * current_vertex.y + (2 * Move_Step + 1) * (current_vertex.x - startPoint.x + Max_Deviation) + current_vertex.edge;
                        for (int idy = 0; idy < i - Move_Step; idy++) {
                            if (y >= idy) {
                                for (int idx = 0; idx <= RoadLen + 1; idx++) {
                                    Vertices[current_index ].Weight += vertices[current_vertex.x + idx][current_vertex.y - idy].Weight;
                                }
                            }
                    }
 
                    if (Vertices[current_index].Weight != 0)
                        std::cout <<current_vertex.x<<" "<<1000- current_vertex.y<<" "<<current_vertex.edge<<" " << Vertices[current_index].Weight << std::endl;
                }
            }
        }

    for (int i = 0; i <= RoadLen; i++)
        Vertices[i].Weight += vertices[startPoint.x + i][0].Weight;
    Vertices[2 * Max_Deviation * (2 * Move_Step + 1) * 1000 + (2 * Move_Step + 1) * Max_Deviation] = vertices[finishPoint.x][finishPoint.y];
}
std::vector<Coord> Graph::GetValidNeighbors(Coord v) {
    std::vector<Coord> neighbours;
    if (v.y == M - 1 && v.x == 100) {
        neighbours.push_back({ 100,M,0 });
        return neighbours;
    }
    if (v.y == 0) {
        neighbours.push_back({ v.x, v.y + 1, 0 });
        for (int i = 1; i < Move_Step; i++) {
            neighbours.push_back({ v.x - 1, v.y + i, i });
        }
        for (int i = Move_Step + 1; i <= 2 * Move_Step; i++) {
            neighbours.push_back({ v.x + 1, v.y + i - Move_Step, i });
        }
    }
    else if (v.edge == 0) {
        neighbours.push_back({ v.x,v.y + 1, 0 });
        if (Move_Step >= 4) {
            for (int i = 4; i <= Move_Step; i++) 
                neighbours.push_back({ v.x - 1, v.y + i, i });
            for(int i = Move_Step + 4; i <= 2*Move_Step;i++)
                neighbours.push_back({ v.x + 1, v.y + i - Move_Step, i  });

        }
    }
    else if (v.edge == 1) {
        neighbours.push_back({ v.x - 1, v.y + 1, 1 });
        neighbours.push_back({ v.x - 1, v.y + 2, 2  });
    }
    else if (v.edge == Move_Step + 1) {
        neighbours.push_back({ v.x + 1, v.y + 1, 1 + Move_Step });
        neighbours.push_back({ v.x + 1, v.y + 2, 2 + Move_Step });
    }
    else if (v.edge == Move_Step) {
        neighbours.push_back({ v.x , v.y + 1, 0 });
        neighbours.push_back({ v.x - 1, v.y + Move_Step,  Move_Step });
        neighbours.push_back({ v.x - 1, v.y + Move_Step - 1, Move_Step - 1 });
        if (Move_Step >= 6) {
            for (int i = 0; i <= Move_Step - 6; i++)
                neighbours.push_back({ v.x + 1, v.y + Move_Step - i, 2*Move_Step - i });
        }
    }
    else if (v.edge == 2 * Move_Step) {
        neighbours.push_back({ v.x , v.y + 1, 0 });
        neighbours.push_back({ v.x + 1, v.y + Move_Step, 2*Move_Step });
        neighbours.push_back({ v.x + 1, v.y + Move_Step - 1, 2*Move_Step - 1 });
        if (Move_Step >= 6) {
            for (int i = 0; i <= Move_Step - 6; i++)
                neighbours.push_back({ v.x - 1, v.y + Move_Step - i,  Move_Step - i });
        }
    }
    else {
        if (v.edge > Move_Step) {
            neighbours.push_back({ v.x + 1, v.y + v.edge - Move_Step, v.edge });
            neighbours.push_back({ v.x + 1, v.y + v.edge - Move_Step + 1, v.edge + 1 });
            neighbours.push_back({ v.x + 1, v.y + v.edge - Move_Step - 1, v.edge - 1 });
        }
        else {
            neighbours.push_back({ v.x - 1, v.y + v.edge , v.edge });
            neighbours.push_back({ v.x - 1, v.y + v.edge + 1, v.edge + 1 });
            neighbours.push_back({ v.x - 1, v.y + v.edge - 1, v.edge - 1 });
        }

    }
    
    return neighbours;
}


//double Graph::GetWeight(Coord start, Coord finish) {
//    double current_weight = 0;
//    if (start != finish) {
//        for (int j = start.y; j < finish.y; j++) {
//            if (Vertices[std::min(start.x, finish.x)][j].Relief_Right) {
//                for (int i = std::min(start.x, finish.x); i <= std::max(start.x, finish.x) + RoadLen; i++) {
//                    if (Vertices[i][j].Relief_Straight && Vertices[i][j].Relief_Right) {
//                        current_weight += Vertices[i][j].Weight;
//                    }
//                    else {
//                        return INF_weight;
//                    }
//                }
//            }
//            else {
//                //std::cout << "Right"<<std::endl;
//                return INF_weight;
//            }
//        }
//    }
//    else {
//        if (Vertices[start.x][start.y].Relief_Right) {
//            for (int i = start.x; i <= start.x + RoadLen; i++) {
//                if (Vertices[i][start.y].Relief_Straight) {
//                    current_weight += Vertices[i][start.y].Weight;
//                }
//                else {
//                    return INF_weight;
//                }
//            }
//        }
//        else {
//            return INF_weight;
//        }
//    }
//    return current_weight;
//}

//double Graph::GetRoute(Coord move, Coord dest) {
//    if (Vertices[move.x][move.y].CameFrom.x != INF)
//        return 10*pow(abs(move.x - Vertices[move.x][move.y].CameFrom.x),2);
//    else {
//        return abs(move.x - dest.x);
//    }
//}
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
    Coord current_vertex = goalPoint;
    unsigned int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * 1000 + (2 * Move_Step + 1) * Max_Deviation;
    Coord end = { INF,INF };
    //    for (int i = 0; i <= 3; i++) {
    //        for (int j = 0; j <= RoadLen; j++) {
    //            TotCategories[i] += Vertices[current_vertex.x + j][M - 1].Categories[i];
    //        }
    //    }
      //  route_len += pow(current_vertex.x - Vertices[current_vertex.x][current_vertex.y].CameFrom.x, 2) + pow(current_vertex.y - Vertices[current_vertex.x][current_vertex.y].CameFrom.y, 2);
    while (current_vertex.y > 0)
    {

        tmp_path.push(current_vertex);
        current_vertex = Vertices[current_index].CameFrom;
        
        current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * current_vertex.y + (2 * Move_Step + 1) * (current_vertex.x - 100 + Max_Deviation) + current_vertex.edge;
        //std::cout <<current_vertex.x<<" "<<current_vertex.y<<" "<<current_vertex.edge<<" "<< Vertices[current_index].Weight << " " << Vertices[current_index].Label << std::endl;
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
        unsigned int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * current_coord.y + (2 * Move_Step + 1) * (100-current_coord.x +Max_Deviation) + current_coord.edge;
        if (Vertices[current_index].IsVisited)
        {
            continue;
        }
        Vertices[current_index].IsVisited = true;
        std::vector<Coord> all_neighbors;
        auto neighbors = GetValidNeighbors(current_coord);
        for (auto neighbor : GetValidNeighbors(current_coord))
        {
            int neighbor_index = 2 * Max_Deviation * (2 * Move_Step + 1) * neighbor.y + (2 * Move_Step + 1) * (neighbor.x - startPoint.x + Max_Deviation) + neighbor.edge;
            if (neighbor.x < startPoint.x - Max_Deviation || neighbor.x > startPoint.x - RoadLen +  Max_Deviation || neighbor.x < startPoint.x  || neighbor.y < 0 || neighbor.x >= /*N - RoadLen*/ /*startPoint.x + Max_Deviation - RoadLen ||*/ neighbor.y > M - 1 || neighbor_index > goal_index || neighbor_index < 0) continue;
            if (!Vertices[neighbor_index].IsVisited)
            {

               
                double next_label = current_weight + Vertices[neighbor_index].Weight;// GetWeight(current_coord, neighbor) + GetDistance(current_coord, neighbor) +GetRoute(neighbor, goalPoint);
                if (next_label < Vertices[neighbor_index].Label)
                {
                   // std::cout << "x: " << neighbor.x << " y: " << neighbor.y << " edge: " << neighbor.edge << std::endl;
                   // std::cout << next_label << " " << neighbor_index << std::endl;
                    Vertices[neighbor_index].Label = next_label;
                    Vertices[neighbor_index].CameFrom = current_coord;
                    min_weigth_map.insert({ next_label, neighbor });
                }

            }
        }
    }
    //    Vertices.back().Label += Vertices.back().Weight;//GetWeight(goalPoint, goalPoint);
    //    std::cout << Vertices.back().Label << std::endl;
    std::cout << Vertices[goal_index].Label << std::endl;;
    return ConvertGraphToPath(Vertices[goal_index].CameFrom);
}
double Graph::GetLabel(Coord c)
{
    unsigned int current_index = 2 * Max_Deviation * (2 * Move_Step + 1) * c.y + (2 * Move_Step + 1) * (c.x - 100 + Max_Deviation) + c.edge ;
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
