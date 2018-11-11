
/**
 * \file: hitbox.hpp
 *
 */

#ifndef HITBOX_HPP
#define HITBOX_HPP

#include "vector.hpp"
//#include "memtrace.h"

/**
 * Hitbox == nevleges kiterjedes
 */
class Hitbox{
    Vector position;
    double width, height;
public:
    /// Konstruktor
    /// @param x - koordinata
    /// @param y - koordinata
    /// @param w - szelesseg
    /// @param h - magassag
    Hitbox(double x, double y, double w, double h): position(x,y), width(w), height(h) {}
    /// Mozgato fuggveny
    /// @param X - x-tg menti eltolas
    /// @param Y - y-tg menti eltolas
    void move(double X, double Y) {
        position.add(X,Y);
    }
    /// Getter
    Vector getPos() const {return position;}
    /// Getter
    double getWidth() const {return width;}
    /// Getter
    double getHeight() const {return height;}
    ~Hitbox(){}
};
#endif // HITBOX_HPP

