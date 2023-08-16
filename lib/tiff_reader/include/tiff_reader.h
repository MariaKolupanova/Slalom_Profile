#pragma once
#ifndef TIFF_READER_H
#define TIFF_READER_H

#include "shape_tiff_model.h"

namespace tiff_reader
{
    struct TiffReader
    {
    public:
        TiffReader(std::vector<std::vector<Vertex>>& Vertices_, const ShapeTiffGridSettings& readSettings_);
        ~TiffReader() = default;

        bool readFile(const char* filePath, double max_angle_hor, double max_angle_vert);
        void readModel(const std::vector<std::string>& filePaths, double max_angle_hor, double max_angle_vert);
        void readModel(const std::string& filePath, double max_angle_hor, double max_angle_vert);
        //std::vector<Relief> Get_Relief() { return cells_coords; }
        std::vector<Relief> GetRelief() {
            for (int i = 0; i < Vertices.size(); i++) {
                for (int j = 0; j < Vertices[0].size(); j++) {
                    if (!Vertices[i][j].Relief_Right && Vertices[i][j].Relief_Straight)  cells_coords.push_back({ i,j,3.0 });
                    else {
                        if (!Vertices[i][j].Relief_Right)  cells_coords.push_back({ i,j,2.0 });
                        else {
                            if (!Vertices[i][j].Relief_Straight)  cells_coords.push_back({ i,j,1.0 });
                        }
                    }
                }
            }
            return  cells_coords;
        }
    private:
        std::vector<std::vector<Vertex>> Vertices;
        std::vector<Relief> cells_coords;     // x_pix_index, y_pix_index, height
        const ShapeTiffGridSettings readSettings;
        int RoadLen = 3;

    };

} //namespace tiff_reader

#endif //TIFF_READER_H

