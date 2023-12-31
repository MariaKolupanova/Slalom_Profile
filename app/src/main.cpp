
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
    // read tiff
    //tiff_reader::TiffReader tiff = { Vertices,st_model };
    float max_angle_vert = 0.3;
    float max_angle_hor = 0.2;
    //tiff.readModel(files_tiff,  max_angle_hor, max_angle_vert);
    //vector<Relief> r = tiff.Get_Relief();

    int N = st_model.cellWidth;
    int M = st_model.cellHeight;

    //write conditions for movement
    int move_step = 5;
    int road_len = 3;

    // choose start and finish point

    Coord startPoint = { 860, 0, -1 };
    Coord goalPoint = { startPoint.x, M };
    int max_deviation = 10;
    Coord routStart = startPoint;
    Coord routFinish = goalPoint;
    //make graph
    clock_t tStart = clock();
    Graph g = { Vertices,startPoint, goalPoint, int(st_model.user_grid), M, int(road_len * st_model.user_grid),int(move_step ), int(max_deviation * st_model.user_grid), 0, 0.97 };
    printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
    //find shortest path
    vector<Coord> path;
    double weight = INF;
    path = g.find_path_Dijkstra();
    double Label = g.GetLabel({goalPoint.x,M+1,0});
    //    printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
    double Route_Len = g.GetRouteLen();
    vector<int> Categories = g.GetCategories();
    cout << "Number of destroied trees:" << endl;
    cout << "Category: thgh < 10: " << Categories[0] << endl;
    cout << "Category: 10 <= thgh < 20: " << Categories[1] << endl;
    cout << "Category: 20 <= thgh < 30: " << Categories[2] << endl;
    cout << "Category: 30 <= thgh: " << Categories[3] << endl;

    // write shortest path, trees and relief
    ofstream out;
    out.open("path.txt");
    if (out.is_open()) {
        out << "x_1,x_2, y,edge, label" << endl;
        for (const auto& path_coord : path)
            out << path_coord.x/st_model.user_grid << ", " << path_coord.x/st_model.user_grid + road_len << ", " << path_coord.y << ", "<<path_coord.edge<<", " << g.GetLabel(path_coord) << endl;
    }
    cout << "Current label: " << /*weight*/Label << endl;
    cout << "Straight label: " << g.GetStraightLabel(startPoint, goalPoint) << endl;
    cout << "Route Len: " << Route_Len << "; Start Point: {" << int(routStart.x/st_model.user_grid) << "," << routStart.y << "}; Goal Point : {" << int(routFinish.x/st_model.user_grid) << ", " << routFinish.y << "};" << endl;
    ofstream out_2;
    out_2.open("weights.txt");
    if (out_2.is_open()) {
        out_2 << "x, y, thck" << endl;
        for (const auto& t : sh.GetTrees())
            out_2 << t.x/ st_model.user_grid << ", " << t.y << ", " << t.thck << std::endl;
    }

    return 0;
}