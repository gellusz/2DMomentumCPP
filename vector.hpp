
/**
 * \file: vector.hpp
 *
 */

#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <fstream>
#include <iostream>
#include "memtrace.h"
#include "gtest_lite.h"

/**
 * Vector == Koordinatapontok
 */
class Vector{
    double posX, posY;
public:
    Vector(double x, double y): posX(x), posY(y) {}
    /// Getter
    double getX() const {return posX;}
    /// Getter
    double getY() const {return posY;}
    /// Ezzel helyezheto arrebb a vector pozicioja
    /// @param x - x-koordinata
    /// @param y - y-koordinata
    void add(double x, double y) {posX += x; posY += y;}
    ~Vector() {}
};

#endif // VECTOR_HPP

