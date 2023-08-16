#include "shape_reader.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <ShapefileReader.hpp>

namespace shape_reader
{
    bool ShapeReader::readFile(const std::string& filePath) {
        std::ifstream fin(filePath);
        if (!fin.is_open()) {
            return false;
        }
        fin.close();
        try
        {
            shp::ShapefileReader shp{ filePath };
            if (shp.getGeometryType() != shp::GeometryType::PointZ) {
                throw std::runtime_error("wrong shape type!\n");
            }
            Tree tr;
            for (auto const& feature : shp) {
                if (feature.getAttributes().contains("thck")) {
                    tr.thck = std::any_cast<double>(feature.getAttributes()["thck"]);
                }
                if (feature.getAttributes().contains("hght")) {
                    tr.thck = std::any_cast<double>(feature.getAttributes()["hght"]);
                }
                auto p = dynamic_cast<shp::Point*>(feature.getGeometry());
                tr.x = p->getX();
                tr.y = p->getY();
                trees.push_back(tr);
            }
        }
        catch (const std::exception& ex)
        {
            std::cout << "!!!!!!! Exception \"" << ex.what() << "\" while reading " << filePath << " !!!!!!!" << std::endl;
            return false;
        }

        return true;
    }
    void ShapeReader::MakeVertices(std::vector<std::vector<Vertex>>& Vertices) {
        auto it_x = std::minmax_element(trees.begin(), trees.end(), [](Tree& lhs, Tree& rhs) { return lhs.x < rhs.x; });
        auto it_y = std::minmax_element(trees.begin(), trees.end(), [](Tree& lhs, Tree& rhs) { return lhs.y < rhs.y; });
        const int N = int(readSettings.user_grid * ceil((it_x.second->x - it_x.first->x)) + 1);
        const int M = ceil(readSettings.user_grid * ceil((it_y.second->y - it_y.first->y)) + 1);
        readSettings = { M,N,{it_x.first->x,it_x.second->x,it_y.first->y,it_y.second->y} };
        Vertices.resize(N, std::vector<Vertex>(M));
        for (auto tr : trees) {
            Coord pos = { int(floor(readSettings.user_grid * (tr.x - it_x.first->x))), int(floor(readSettings.user_grid * (tr.y - it_y.first->y))) };
            Vertices[pos.x][pos.y].Weight = +GetWeight(tr.thck);
            if (tr.thck > 40) Vertices[pos.x][pos.y].Categories[3] += 1;
            else  Vertices[pos.x][pos.y].Categories[floor(tr.thck / 10)] += 1;
            if (Vertices[pos.x][pos.y].Weight > max_weight) max_weight = Vertices[pos.x][pos.y].Weight;
        }
        for (int i = 0; i < Vertices.size(); i++) {
            for (int j = 0; j < Vertices[0].size(); j++) {
                if (Vertices[i][j].Weight != 0) {
                    Tree t = {  static_cast<double>( i), static_cast<double>( j),Vertices[i][j].Weight };
                    res.push_back( t);
                }
            }
        }
    }
    void ShapeReader::readModel(const std::string& filePath) {

        std::cout << "Start read shape: " << filePath << std::endl;
        if (!readFile(filePath)) {
            std::cout << "Can't read shape: " << filePath << std::endl;
            return;
        }
        std::cout << "reading shape file done" << std::endl;
    }
    double ShapeReader::GetWeight(double thck) {
        //if (thck >= 30) return 1000;
        //if (thck >= 20) return 100;
        //if (thck >= 10) return 20;
        return thck;
    }

    void ShapeReader::readModel(const std::vector<std::string>& filePaths) {
        if (filePaths.size() == 0) {
            std::cout << "Can't read shapes. No filePaths to read!" << std::endl;
            return;
        }
        std::cout << "Start reading shapes: [ " << filePaths[0] << " - " << filePaths[filePaths.size() - 1] << " ] " << std::endl;
#pragma omp parallel for
        int readFiles{ 0 };
        for (const auto& filePath : filePaths) {
            if (readFile(filePath))
                ++readFiles;
        }
        std::cout << "reading shape files " << readFiles << "/" << filePaths.size() << " done" << std::endl;
    }

} //namespace shape_reader