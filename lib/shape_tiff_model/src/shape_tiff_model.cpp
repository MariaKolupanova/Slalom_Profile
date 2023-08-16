//#include "shape_tiff_model.h"
//
//namespace shape_tiff_model
//{
//    void ShapeTiffModel::clear() {
//        category_points.clear();
//    }
//    
//    ShapeTiffModel::ShapeTiffModel(const ShapeTiffGridSettings& readSettings) :
//        boundaryBox(readSettings.boundaryBox),
//        userWidth(readSettings.cellWidth),
//        userHeight(readSettings.cellHeight) {}
//
//    void ShapeTiffModel::addCategoryPoint(const int x, const int y) {
//        category_points.push_back({ x,y });
//    }
//
//    const ShapeTiffGridSettings::BoundaryBox& ShapeTiffModel::getBoundaryBox() const {
//        return boundaryBox;
//    }
//
//    float ShapeTiffModel::getUserWidth() const {
//        return userWidth;
//    }
//
//    float ShapeTiffModel::getUserHeight() const {
//        return userHeight;
//    }
//
//    const std::vector<std::pair<int, int>>& ShapeTiffModel::getCategoryPoints() const {
//        return category_points;
//    }
//
//    int ShapeTiffModel::getGridSize() const {
//        return int((boundaryBox.xMax - boundaryBox.xMin) / userWidth) * int((boundaryBox.yMax - boundaryBox.yMin) / userHeight);;
//    }
//
//} //namespace shape_tiff_model
