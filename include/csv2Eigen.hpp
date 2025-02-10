#ifndef __csv2Eigen__
#define __csv2Eigen__

#include <Eigen/Dense>
#include <vector>
#include <fstream>

using namespace Eigen;

template<typename M>
M load_csv (const std::string & path, unsigned int row_beg = 0, unsigned col_beg = 0) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    unsigned int rows = 0;
    unsigned int cols = 0;
    while (std::getline(indata, line)) {
        if (rows<row_beg){
            rows++;
            continue;
        }
        std::stringstream lineStream(line);
        std::string cell;
        cols = 0;
        while (std::getline(lineStream, cell, ',')) {
            if (cols < col_beg){
                cols++;
                continue;
            }
            values.push_back(std::stod(cell));
            cols++;
        }
        rows++;
    }

    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows-row_beg, values.size()/(rows-row_beg));
}

#endif //__csv2Eigen__