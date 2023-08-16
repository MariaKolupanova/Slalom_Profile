#pragma once
#ifndef SHAPE_TIFF_MODEL_H
#define SHAPE_TIFF_MODEL_H

#include <vector>
#include <iostream>

const auto INF_weight = std::numeric_limits<double>::infinity();
const auto INF = std::numeric_limits<int>::max();

struct Tree {
    double x, y;
    double thck = 0;
    double hght = 0;
    double relief = 0;
};

struct Relief {
    int x, y;
    double hght;
};

struct Coord
{
    int x, y;
    int edge = 0;
    bool operator != (const Coord& c) {
        return !(*this == c);
    }
    bool operator == (const Coord& c) {
        return x == c.x && y == c.y /*&& edge == c.edge*/;
    }
    Coord operator - (const Coord& c) {
        return { x - c.x, y - c.y, edge - c.edge };
    }
    Coord operator + (const Coord& c) {
        return { x + c.x, y + c.y , edge + c.edge };
    }
};

struct Vertex
{
    //Coord Coordinate = { INF,INF, INF };
    Coord CameFrom = { INF,INF };
    bool IsVisited = false;
    double Label = INF;
    double Weight = 0;
    double Height = 0;
    bool Relief_Straight = true;
    bool Relief_Right = true;
    std::vector<int> Categories = { 0,0,0,0 };
    Vertex operator + (const Vertex& v) {
        Vertex sum_v;
        sum_v.Weight = Weight + v.Weight;
        for (int i = 0; i < Categories.size(); i++) {
            sum_v.Categories[i] = Categories[i] + v.Categories[i];
        }
        return sum_v;
    }
};

struct ShapeTiffGridSettings
{
    struct BoundaryBox
    {
        double xMin;
        double xMax;
        double yMin;
        double yMax;
        
    };
    int cellHeight;
    int cellWidth;
    BoundaryBox boundaryBox;
    double user_grid = 1;
};


#endif //SHAPE_TIFF_MODEL_H
