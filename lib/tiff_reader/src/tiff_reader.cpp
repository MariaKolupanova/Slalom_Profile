#include "tiff_reader.h"

#include "xtiffio.h"  /* for TIFF */
#include "geotiffio.h" /* for GeoTIFF */

#include <vector> 
#include <map> 
//#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <algorithm>

namespace {
    enum { VERSION = 0, MAJOR, MINOR };
}


namespace tiff_reader
{

    TiffReader::TiffReader(std::vector<std::vector<Vertex>>& Vertices_, const ShapeTiffGridSettings& readSettings_) :
        readSettings(readSettings_), Vertices(Vertices_) {}


    bool TiffReader::readFile(const char* filePath, double max_angle_hor, double max_angle_vert) {
        std::ifstream fin(filePath);
        if (!fin.is_open()) {
            return false;
        }
        fin.close();

        try
        {
            auto boundaryBox = readSettings.boundaryBox;

            /* TIFF information */
            TIFF* tif = (TIFF*)0;  /* TIFF-level descriptor */
            GTIF* gtif = (GTIF*)0; /* GeoKey-level descriptor */
            int versions[3];
            int cit_length;
            geocode_t model;    /* all key-codes are of this type */
            char* citation;
            int key_count;

            /* Open TIFF descriptor to read GeoTIFF tags */
            tif = XTIFFOpen(filePath, "r");//DTM_Vorgen_UTM44N.tif 000072_ground_chm.tif
            if (!tif) {
                std::cout << "Can't read tif: " << filePath << std::endl;
                return false;
            };

            /* Open GTIF Key parser; keys will be read at this time. */
            gtif = GTIFNew(tif);
            if (!gtif) {
                std::cout << "Can't read tif: " << filePath << std::endl;
                return false;
            };

            /* Get the GeoTIFF directory info */
            GTIFDirectoryInfo(gtif, versions, &key_count);
            if (versions[MAJOR] > 1)
            {
                printf("this file is too new for me\n");
                return false;
            }

            if (!GTIFKeyGet(gtif, GTModelTypeGeoKey, &model, 0, 1))
            {
                printf("Yikes! no Model Type\n");
                return false;
            }
            unsigned short count;
            double* tiePoints;
            double dx, dy, x_start, y_start;

            if (TIFFGetField(tif, TIFFTAG_GEOPIXELSCALE, &count, &tiePoints) == 1) {
                dx = tiePoints[0];
                dy = tiePoints[1];
                //printf("Tie Points: \n");
                //printf("dx: %f", tiePoints[0]); // dX - размер ячейки по x
                //printf("\t dy: % f\n", tiePoints[1]); // dY - размер ячейки по y
            }

            if (TIFFGetField(tif, TIFFTAG_GEOTIEPOINTS, &count, &tiePoints) == 1) {
                x_start = tiePoints[3];
                y_start = tiePoints[4];
                //printf("Tie Points: \n");
                //printf("Lat: %f", tiePoints[3]); // X - координата верхней левой точки
                //printf("\t Lon: % f\n", tiePoints[4]); // Y - координата верхней левой точки
            }

            int size = 0;
            tagtype_t type;

            /* ASCII keys are variable-length; compute size */
            cit_length = GTIFKeyInfo(gtif, GTCitationGeoKey, &size, &type);
            if (cit_length > 0)
            {
                citation = static_cast<char*>(malloc(size * cit_length));
                if (!citation) {
                    std::cout << "Smth goes wrong in tiff_reader!" << std::endl;
                    return false;
                };

                GTIFKeyGet(gtif, GTCitationGeoKey, citation, 0, cit_length);
                //printf("Citation:%s\n", citation);
            }

            uint32 w, h, tw, th; // w - длина строки в пикселях h - кол-во строк
            size_t npixels;
            float* raster;
            uint16 smpp, bps;
            uint32 tdepth, depth;
            TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
            TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);
            //std::cout << "width_pixels " << w << "   " << "height_pixels " << h << '\n';
            bool swpd = TIFFIsByteSwapped(tif);
            bool tiled = TIFFIsTiled(tif);

            if (tiled) {
                TIFFGetField(tif, TIFFTAG_TILEWIDTH, &tw);
                TIFFGetField(tif, TIFFTAG_TILELENGTH, &th);
            }
            else {
                tw = w;
                th = h;
            }

            TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &smpp);
            TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bps);
            TIFFGetField(tif, TIFFTAG_IMAGEDEPTH, &depth);
            TIFFGetField(tif, TIFFTAG_TILEDEPTH, &tdepth);
            npixels = w * h;

            if (tiled) {
                raster = (float*)_TIFFmalloc(smpp * w * th * sizeof(float));
                //std::cout << "tiled file" << "\n";
            }
            else {
                raster = (float*)_TIFFmalloc(smpp * npixels * sizeof(float));
                //std::cout << "Not tiled file" << "\n";
            }

            // Boundary box of xps file (1210)
            float coord_up = boundaryBox.yMax,
                coord_down = boundaryBox.yMin,
                coord_left = boundaryBox.xMin,
                coord_right = boundaryBox.xMax;
            //std::cout << std::setprecision(10) << "coord_up " << coord_up << "\n";
            //std::cout << std::setprecision(10) << "coord_down " << coord_down << "\n";
            //std::cout << std::setprecision(10) << "coord_left " << coord_left << "\n";
            //std::cout << std::setprecision(10) << "coord_right " << coord_right << "\n" << "\n";

            // Tiff data
            float coord_tiff_up = y_start,
                coord_tiff_down = y_start - h * dy,
                coord_tiff_left = x_start,
                coord_tiff_right = x_start + w * dx;
            //std::cout << std::setprecision(10) << "coord_tiff_up " << coord_tiff_up << "\n";
            //std::cout << std::setprecision(10) << "coord_tff_down " << coord_tiff_down << "\n";
            //std::cout << std::setprecision(10) << "coord_tiff_left " << coord_tiff_left << "\n";
            //std::cout << std::setprecision(10) << "coord_tiff_right " << coord_tiff_right << "\n";

            const float tiff_grid_deltax{ coord_tiff_left - coord_left };
            const float tiff_grid_deltay{ coord_tiff_up - coord_up };
            int blocks_x, blocks_y;

            const int block_xmin = 0;
            const int block_ymin = 0;
            if (raster != NULL) {
                if (!tiled) {
                    auto scan_line = (uint32*)malloc(smpp * w * (sizeof(uint32)));
                    for (int i = 0; i < h; i++) //loading the data into a buffer
                    {
                        TIFFReadScanline(tif, scan_line, i);
                        if (scan_line) {
                            memcpy(&raster[i * smpp * w], scan_line, smpp * w * sizeof(uint32));
                        }
                    }
                    if (readSettings.user_grid >= 1) {
                        const int block_xmax = int(coord_right - coord_left);                          // Возможно не int, а ceil?
                        const int block_ymax = int(coord_up - coord_down);                            // Возможно не int, а ceil?
                        auto userWidth = int(readSettings.user_grid);
                        auto userHeight = int(readSettings.user_grid);
                        std::vector<Relief> cells_coords;
                        for (int i = 0; i < h - 1; i++) {
                            blocks_y = block_ymax - int(-tiff_grid_deltay + dy * i + 0.5 * dy);
                            for (int j = 0; j < w - RoadLen; j++) {
                                blocks_x = int(tiff_grid_deltax + dx * j + 0.5 * dx);
                                if (blocks_x < block_xmax && blocks_y < block_ymax &&
                                    blocks_x > block_xmin && blocks_y > block_ymin) {
                                    const double val{ raster[size_t(i * w + j)] };
                                    const double val_st{ raster[size_t((i + 1) * w + j)] };
                                    const double val_r{ raster[size_t(i * w + j + 1)] };
                                    bool rel_right = true;
                                    // for (int idx = 0; idx < RoadLen; idx++) {
                                      //   const double val_r{ raster[size_t(i * w + j )] };
                                     //    if (rel_right && val_r > 0 && abs(val - val_r) > max_angle)
                                     //        rel_right = false;
                                    // }
                                    if (val > 0) {
                                        cells_coords.push_back({ userWidth * j,int(readSettings.cellHeight - userHeight * i - 1), val });
                                        for (int par_x = 0; par_x < userWidth; par_x++) {
                                            for (int par_y = 0; par_y < userHeight; par_y++) {
                                                if (val_st > 0) Vertices[int(userWidth * j) + par_x][int(readSettings.cellHeight - userHeight * i) + par_y - 1].Relief_Straight = (abs(val - val_st) < max_angle_vert);
                                                if (val_r > 0) Vertices[int(userWidth * j) + par_x][int(readSettings.cellHeight - userHeight * i) + par_y - 1].Relief_Right = (abs(val - val_r) < max_angle_hor);
                                                //Vertices[int(userWidth*j) + par_x][int(readSettings.cellHeight - userHeight * i) + par_y - 1].Relief = val;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else {
                        using Coords = std::tuple<int, int, float>;
                        std::vector<std::vector<Coords>> cells_coords;
                        auto userWidth = int(1 / readSettings.user_grid);
                        auto userHeight = int(1 / readSettings.user_grid);
                        const int block_xmax = int((coord_right - coord_left) / userWidth);                          // Возможно не int, а ceil?
                        const int block_ymax = int((coord_up - coord_down) / userHeight);
                        int gridSize = block_xmax * block_ymax;
                        cells_coords.resize(gridSize);
                        for (int i = 0; i < h; i++) {
                            blocks_y = block_ymax - int((-tiff_grid_deltay + dy * i + 0.5 * dy) / userHeight);
                            for (int j = 0; j < w; j++) {
                                blocks_x = int((tiff_grid_deltax + dx * j + 0.5 * dx) / userWidth);
                                const double val{ raster[size_t(i * w + j)] };
                                if (blocks_x  < block_xmax && blocks_y < block_ymax &&
                                    blocks_x  > block_xmin && blocks_y > block_ymin &&
                                    val > 0) {
                                    cells_coords[blocks_x + blocks_y * block_xmax].push_back(std::make_tuple(i, j, val));
                                }
                            }
                        }


                        for (int cell_num = 0; cell_num < cells_coords.size(); ++cell_num) {
                            const auto& c{ cells_coords[cell_num] };
                            float sum_height = 0;
                            float max_height = 0;
                            float coord_max_height = 0;
                            float min_height = INT_MAX;
                            float coord_min_height = 0;
                            if (c.size() == 0)
                                continue;
                            for (int i = 0; i < c.size(); i++) {
                                const double cur_height{ std::get<2>(c[i]) };
                                sum_height += cur_height;
                                if (cur_height > max_height) {
                                    max_height = cur_height;
                                    coord_max_height = i;
                                }
                                if (cur_height < min_height) {
                                    min_height = cur_height;
                                    coord_min_height = i;
                                }
                            }
                            if ((max_height - min_height) / std::pow((std::pow((std::get<0>(c[coord_max_height]) - std::get<0>(c[coord_min_height])), 2) + std::pow((std::get<1>(c[coord_max_height]) - std::get<1>(c[coord_min_height])), 2)), 0.5) / dx > std::min(max_angle_hor, max_angle_vert)) {
                                Vertices[cell_num % block_xmax][int(std::floor(cell_num / block_xmax))].Relief_Straight = false;
                                Vertices[cell_num % block_xmax][int(std::floor(cell_num / block_xmax))].Relief_Right = false;
                            }
                        }

                    }
                }
                else {
                    return false;
                }
            }

            _TIFFfree(raster);
            /* get rid of the key parser */
            GTIFFree(gtif);

            /* close the TIFF file descriptor */
            XTIFFClose(tif);


        }
        catch (const std::exception& ex)
        {
            std::cout << "!!!!!!! Exception \"" << ex.what() << "\" while reading " << filePath << " !!!!!!!" << std::endl;
            return false;
        }

        return true;
    }

    void TiffReader::readModel(const std::string& filePath, double max_angle_hor, double max_angle_vert) {

        std::cout << "Start read tiff: " << filePath << std::endl;
        if (!readFile(filePath.c_str(), max_angle_hor, max_angle_vert)) {
            std::cout << "Can't read tiff: " << filePath << std::endl;
            return;
        }
        std::cout << "reading tiff file done" << std::endl;

    }
    void TiffReader::readModel(const std::vector<std::string>& filePaths, double max_angle_hor, double max_angle_vert) {

        if (filePaths.size() == 0) {
            std::cout << "Can't read tiffs. No filePaths to read!" << std::endl;
            return;
        }
        std::cout << "Start reading tiffs: [ " << filePaths[0] << " - " << filePaths[filePaths.size() - 1] << " ] " << std::endl;
#pragma omp parallel for
        int readFiles{ 0 };
        for (const auto& filePath : filePaths) {
            if (readFile(filePath.c_str(), max_angle_hor, max_angle_vert))
                ++readFiles;
        }
        std::cout << "reading tiff files " << readFiles << "/" << filePaths.size() << " done" << std::endl;

    }

} //namespace tiff_reader
