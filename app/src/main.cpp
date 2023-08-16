
//  main.cpp
//  Tree
//
//  Created by Maria on 29.05.2023.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include "Graph.h"
#include "shape_reader.h"
//#include "tiff_reader.h"
#include <time.h>
using namespace std;

int main()
{
    std::vector<std::vector<Vertex>> Vertices;
    //read shape
    vector<string> files_shape;
    vector<string> files_tiff;
    for (int i = 36; i <= 36; i++) {
//        string s = "C:\\Users\\kolupanova.ma\\Desktop\\stratasolutions\\forest\\0" + to_string(i) + "\\pt0000" +
//            to_string(i) + "_ground_localMax.shp";
        string s = "/Users/maria/Slalom_Profile/data/0" + to_string(i) + "/pt0000" + to_string(i) + "_ground_localMax.shp";
        files_shape.push_back(s);
        //        s = "C:\\Users\\kolupanova.ma\\Desktop\\stratasolutions\\relief\\0" + to_string(i) + "\\pt0000" +
        //            to_string(i) + "_ground_dem_smooth.tif";
        //        files_tiff.push_back(s);
    }
    shape_reader::ShapeReader sh;
    sh.readModel(files_shape);
    sh.MakeVertices(Vertices);
    // get boundary box for tiff 
    ShapeTiffGridSettings st_model = sh.GetSettings();
    double max_weight = sh.GetMaxWeight();

    float max_angle_vert = 0.3;
    float max_angle_hor = 0.2;

    int N = st_model.cellWidth;
    int M = st_model.cellHeight;

    //write conditions for movement
    int move_step = 5;
    cout << max_weight << endl;
    int road_len = 3;
    //int max_rad = 200;

    // choose start and finish point

    Coord startPoint = { 100, M-1 };
    Coord goalPoint = { startPoint.x, 0,0 };
    int max_deviation = 10;
    Coord routStart = startPoint;
    Coord routFinish = goalPoint;
    //make graph
    Graph g = { Vertices, N, M, int(road_len * st_model.user_grid),int(move_step * st_model.user_grid), int(max_deviation * st_model.user_grid), 0, 0 };
    vector<Coord> path;
    double weight = INF;
    g.find_path_Dijkstra({ startPoint.x, startPoint.y }, goalPoint);
    path = g.find_path_Dijkstra({ startPoint.x, startPoint.y }, goalPoint);
    std::cout << M << std::endl;
    ofstream out;
    out.open("path.txt");
    if (out.is_open()) {
        out << "x_1,x_2, y,edge, label" << endl;
        for (const auto& path_coord : path)
            out << path_coord.x << ", " << path_coord.x + road_len << ", " << path_coord.y << ", "<<path_coord.edge<<", " << g.GetLabel(path_coord) << endl;
    }
    ofstream vertex;
    out.open("vertex.txt");
    if (out.is_open()) {
        for (const auto& path_coord : path)
            out << path_coord.x << ", " << path_coord.x + road_len << ", " << path_coord.y  << endl;
    }
    ofstream out_2;
    out_2.open("weights.txt");
    if (out_2.is_open()) {
        out_2 << "x, y, thck" << endl;
        for (const auto& t : sh.GetTrees())
            out_2 << t.x << ", " << t.y << ", " << t.thck << std::endl;
    }


    return 0;
}