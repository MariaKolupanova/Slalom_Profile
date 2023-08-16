#pragma once
#ifndef SHAPE_READER_H
#define SHAPE_READER_H

#include <string>
#include <vector>
#include <shape_tiff_model.h>

namespace shape_reader
{
    struct ShapeReader
    {
    public:
        ShapeReader() = default;
        ~ShapeReader() = default;

        void readModel(const std::string& filePath);
        void readModel(const std::vector<std::string>& filePaths);
        bool readFile(const std::string& filePath);
        std::vector<Tree> GetTrees() { return res; }
        // void MakeVertices(std::vector<std::vector<Vertex>>& Vertices, int file_num);
        void MakeVertices(std::vector<std::vector<Vertex>>& Vertices);
        // std::vector<std::vector<Vertex>> GetVertices() { return Vertices; }
        ShapeTiffGridSettings GetSettings() { return readSettings; }
        double GetMaxWeight() { return max_weight; }
        double GetWeight(double thck);
    private:
        std::vector<Tree> trees;
        std::vector<Tree> res;
        // std::vector<std::vector<Vertex>> Vertices;
        ShapeTiffGridSettings readSettings;
        double max_weight = 0;
    };

} //namespace shape_reader

#endif //SHAPE_READER_H
