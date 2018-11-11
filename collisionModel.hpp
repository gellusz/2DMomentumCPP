
/**
 * \file: collisionModel.hpp
 *
 */

#ifndef COLLISIONMODEL_HPP
#define COLLISIONMODEL_HPP

#include "hitbox.hpp"
//#include "memtrace.h"

///


///

/**
 * CollisionModel == utkozesmodell
 */
class CollisionModel{
protected:
    std::string name;
    constexpr static double gravity = -9.81;
    constexpr static double TIME = 0.1;
    Hitbox hitbox;
    double flexibility;
    size_t id;
public:
    /// Konstruktor
    /// @param x - koordinata
    /// @param y - koordinata
    /// @param w - szelesseg
    /// @param h - magassag
    /// @param flex - rugalmassagi egyutthato
    /// @param id - azonosito
    /// @param n - nev
    CollisionModel(double x=0, double y=0, double w = 1, double h = 1, double flex=0,size_t id=0,std::string n = "UnNamedBase"):
         name(n), hitbox(x,y,w,h), flexibility(flex),id(id){}
    ///
    double getFlexibility() const {return flexibility;}
    CollisionModel& operator=(const CollisionModel& rhs){std::cout<<"CMcONST\n";
    return *this;}
    bool operator==(const CollisionModel &rhs){
        return id==rhs.id;
    }
    std::string getName() {return name;}

    Vector getPosLEdge() const {Vector a((hitbox.getPos().getX()-hitbox.getWidth()),hitbox.getPos().getY()); return a;}
    Vector getPosREdge() const {Vector a(((hitbox.getPos().getX())+hitbox.getWidth()),hitbox.getPos().getY()); return a;}
    Vector getPosUEdge() const {Vector a(hitbox.getPos().getX(),(hitbox.getPos().getY()+hitbox.getHeight())); return a;}
    Vector getPosBEdge() const {Vector a(hitbox.getPos().getX(),(hitbox.getPos().getY()-hitbox.getHeight())); return a;}
    virtual bool isSlope() const = 0;
    virtual int getRGr() const =0;
    virtual int getLGr() const =0;
    virtual double getFriction() const=0;
    virtual double getVx() const =0;
    virtual double getVy() const =0;
    virtual double getAx() const =0;
    virtual double getAy() const =0;
    virtual double getMass() const=0;
    virtual double getHeight(const double coordX) const=0;
    virtual void action()=0;
    virtual void setVx(double v1X)=0;
    virtual void setVy(double v1Y)=0;
    virtual void setAx(double a1X)=0;
    virtual void setAy(double a1Y)=0;
    virtual void addvX(double v1X)=0;
    virtual void addvY(double v1Y)=0;
    virtual void setLeftImpact(CollisionModel* c) =0;
    virtual void setRightImpact(CollisionModel* c) =0;
    virtual void setTopImpact(CollisionModel* c) =0;
    virtual void setBottomImpact(CollisionModel* c) =0;
    virtual CollisionModel* getLeftImpact() const=0;
    virtual CollisionModel* getRightImpact() const=0;
    virtual CollisionModel* getTopImpact() const=0;
    virtual CollisionModel* getBottomImpact() const=0;
    virtual ~CollisionModel() {}
};

class PersistentCM : public CollisionModel {
    double friction;
    int leftGrad, rightGrad;
    bool slope;
public:
    /// Konstruktor
    /// @param n - nev
    /// @param x - koordinata
    /// @param y - koordinata
    /// @param w - szelesseg
    /// @param h - magassag
    /// @param flex - rugalmassagi mertek [0;1]
    /// @param frict - surlodasi egyutthato  [0;1]
    /// @param lGr - bal also sarok szoge fokban [1;90]
    /// @param rGr - jobb also sarok szoge fokban [1;90]
    /// @param sl - lejto-e
    /// @param id - azonosito
    /// @param n - nev

    PersistentCM(double x=0, double y=0, double w = 1, double h = 1, double flex = 0, double frict = 0.5, int lGr = 90, int rGr = 90, bool sl=false,size_t id = 0,std::string n = "UnNamedPersistent"):
         CollisionModel(x,y,w,h,flex,id,n), friction(frict),leftGrad(lGr), rightGrad(rGr),slope(sl){}
    /// Getter
    virtual bool isSlope() const {return slope;}
    /// Getter
    virtual double getFriction() const {return friction;}
    /// Getter
    virtual int getRGr() const {return rightGrad;}
    /// Getter
    virtual int getLGr() const {return leftGrad;}
    virtual double getVx() const {return 0;}
    virtual double getVy() const {return 0;}
    virtual double getAx() const {return 0;}
    virtual double getAy() const {return 0;}
    virtual double getMass() const {return 0;}
    virtual void action() {}
    virtual void setVx(double v1X) {}
    virtual void setVy(double v1Y) {}
    virtual void setAx(double a1X) {}
    virtual void setAy(double a1Y) {}
    virtual void addvX(double v1X) {}
    virtual void addvY(double v1Y) {}
    virtual void setLeftImpact(CollisionModel* c) {}
    virtual void setRightImpact(CollisionModel* c) {}
    virtual void setTopImpact(CollisionModel* c) {}
    virtual void setBottomImpact(CollisionModel* c) {}
    virtual CollisionModel* getLeftImpact() const {return NULL;}
    virtual CollisionModel* getRightImpact() const {return NULL;}
    virtual CollisionModel* getTopImpact() const {return NULL;}
    virtual CollisionModel* getBottomImpact() const {return NULL;}
    /// Adott x-koordinatahoz megadja a megfelelo felso hatarat a persistentCM-nek slope eseten
    /// @param coordX - egy movableCM x-koordinataja
    /// @return h - egy y-koordinata
    virtual double getHeight(const double coordX) const {
        double d,h;
        if(leftGrad == 90){
            d = getPosREdge().getX()-coordX;
            h = d*(std::tan(rightGrad));
        }else if(rightGrad == 90){
            d = coordX-getPosLEdge().getX();
            h = d*(std::tan(leftGrad));
        }
        return h;
    }
};


class MovableCM : public CollisionModel {
    bool controlled;
    double mass;
    double vX, vY;
    double aX, aY;
    double X, Y;
    CollisionModel *leftImpact;
    CollisionModel *rightImpact;
    CollisionModel *topImpact;
    CollisionModel *bottomImpact;
public:
    /// Konstruktor
    /// @param x - koordinata
    /// @param y - koordinata
    /// @param w - szelesseg
    /// @param h - magassag
    /// @param flex - rugalmassagi mertek [0;1]
    /// @param m - tomeg
    /// @param id - azonosito
    /// @param n -nev
    MovableCM(double x=0, double y=0, double w = 1, double h = 1, double flex = 0, double m=1,size_t id = 0,std::string n = "UnNamedMovable") :
        CollisionModel(x,y,w,h,flex,id,n), mass(m), vX(0), vY(0), aX(0), aY(0), X(0), Y(0) {}
    bool isControlled() const {return controlled;}
    /// Getter
    virtual double getVx() const {return vX;}
    /// Getter
    virtual double getVy() const {return vY;}
    /// Getter
    virtual double getAx() const {return aX;}
    /// Getter
    virtual double getAy() const {return aY;}
    /// Egy utkoztetett CM x-tg menti sebessegenek novelesere alkalmas
    /// @param v1X - novekmeny ertek
    virtual void addvX(double v1X){vX = v1X;}
    /// Egy utkoztetett CM y-tg menti sebessegenek novelesere alkalmas
    /// @param v1Y - novekmeny ertek
    virtual void addvY(double v1Y){vY = v1Y;}
    /// Egy utkoztetett CM x-tg menti sebessegenek beallitasara alkalmas
    /// @param v1X - beallitando ertek
    virtual void setVx(double v1X){vX = v1X;}
    /// Egy utkoztetett CM y-tg menti sebessegenek beallitasara alkalmas
    /// @param v1Y - beallitando ertek
    virtual void setVy(double v1Y){vY = v1Y;}
    /// Egy utkoztetett CM x-tg menti gyorsulasanak beallitasara alkalmas
    /// @param a1X - beallitando ertek
    virtual void setAx(double a1X){aX = a1X;}
    /// Egy utkoztetett CM y-tg menti gyorsulasanak beallitasara alkalmas
    /// @param a1Y - beallitando ertek
    virtual void setAy(double a1Y){aY = a1Y;}
    /// Getter    virtual CollisionModel* getLeftImpact() const {return leftImpact; }
    /// Getter
    virtual CollisionModel* getRightImpact() const {return rightImpact; }
    /// Getter    virtual CollisionModel* getTopImpact() const {return topImpact; }
    /// Getter
    virtual CollisionModel* getBottomImpact() const {return bottomImpact; }
    /// Setter
    virtual void setLeftImpact(CollisionModel* c) {leftImpact = c; }
    /// Setter    virtual void setRightImpact(CollisionModel* c) {rightImpact = c; }
    /// Setter
    virtual void setTopImpact(CollisionModel* c) {topImpact = c; }
    /// Setter
    virtual void setBottomImpact(CollisionModel* c) {bottomImpact = c; }
    /// Getter
    virtual double getMass() const {return mass;}

    /// Ez a fuggveny szamolja a kulonbozo esetekben az x es y tengelyek menti a gyorsulast
    void calculateAcceleration(){

        if(topImpact != NULL){
            aY = (-(vY*topImpact->getFlexibility())-vY)/TIME;
            if(vX > ((-gravity)*topImpact->getFriction())*TIME){
                aX = -((-gravity)*topImpact->getFriction());
            }else if(vX < -((-gravity)*topImpact->getFriction())*TIME){
                aX = ((-gravity)*topImpact->getFriction());
            }else{
                aX= 0;
            }
        }
        if(leftImpact != NULL && !(leftImpact->isSlope())){
            aX = (-(vX*leftImpact->getFlexibility())-vX)/TIME;
        }
        if(rightImpact!= NULL && !(rightImpact->isSlope())){
            aX = (-(vX*rightImpact->getFlexibility())-vX)/TIME;
        }
        if(bottomImpact == NULL){
            aY = gravity;
        }else{
            if(!(bottomImpact->isSlope())){
                aY = (-(vY*bottomImpact->getFlexibility())-vY)/TIME;
                if(vX > 0){
                    aX = -((-gravity)*bottomImpact->getFriction());
                }else if(vX < 0){
                    aX = ((-gravity)*bottomImpact->getFriction());
                }else{
                    aX= 0;
                }
            }else{
                if(bottomImpact->getRGr() == 90){
                    aX = gravity*(std::sin(bottomImpact->getLGr()))*(std::cos(bottomImpact->getLGr()));
                    aX += gravity*(std::cos(bottomImpact->getLGr()))*bottomImpact->getFriction();
                }else if(bottomImpact->getLGr() == 90){
                    aX = (-gravity)*(std::sin(bottomImpact->getRGr()))*(std::cos(bottomImpact->getRGr()));
                    aX += (-gravity)*(std::cos(bottomImpact->getRGr()))*bottomImpact->getFriction();
                }
                aY = gravity;
            }
        }
    }

    /// Ez a fuggveny szamolja az utkozesek soran az impulzusmegmaradas torvenye segitsegevel
    /// az utkozo testek utkozesebol eredo sebesseget - Ha az egyenlet egy resze nem megfelelo akkor hiba van a m...omentumban -> kivetel
    /// @param cm1 - az a CM ami az adott pillanatban es iteracioban aktiv es hatast gyakorol a masikra
    /// @param cm2 - az a CM amire az adott pillanatban es iteracioban ki akarjuk szamolni az eredo sebesseget
    double impulseVFormulaX(CollisionModel* cm1, CollisionModel* cm2){
        double massRate ,a,b,c,d, vTmp, v1=cm1->getVx(),v2=cm2->getVx();
            massRate = cm2->getMass()/cm1->getMass();
            a = ((-2)*massRate)-1;
            b = 2*(v1+(massRate*v2));
            c = (v2)*(v2)-2*(v2*v1);
            d = b*b-4*a*c;
            if(d<0)throw "glitch in the mat.... impulse calculation";
            if((v1>0)&&(v2>=0)){
                vTmp = ((-b)-(std::sqrt(d)))/(2*a);
            }else if((v1<0)&&(v2<=0)){
                vTmp = ((-b)+(std::sqrt(d)))/(2*a);
            }
            return vTmp;
    }
    /// Ez a fuggveny szamolja ki a kulonbozo esetekre az x es y tengelyek menti sebesseget
    void calculateVelocity(){
        double vTmp;

        try{
        if(topImpact != NULL){
            if(topImpact->getMass()>0 && vX !=0){
                vTmp = impulseVFormulaX(this,topImpact);
                vX = impulseVFormulaX(topImpact,this);
                aX = 0;
                topImpact->setVx(vTmp);
            }
        }
        if(leftImpact != NULL ){
            if(leftImpact->getMass()>0 && vX !=0){
                vTmp = impulseVFormulaX(this,leftImpact);
                vX = impulseVFormulaX(leftImpact,this);
                aX = 0;
                leftImpact->setVx(vTmp);
            }
        }
        if(rightImpact!= NULL){
            if( rightImpact->getMass()>0 && vX !=0){
                vTmp = impulseVFormulaX(this,rightImpact);
                vX = impulseVFormulaX(rightImpact,this);
                aX = 0;
                rightImpact->setVx(vTmp);
            }
        }
        if(bottomImpact != NULL){
            if(!(bottomImpact->isSlope())){
                if( bottomImpact->getMass()>0 && vX !=0){
                    vTmp = impulseVFormulaX(this,bottomImpact);
                    vX = impulseVFormulaX(bottomImpact,this);
                    aX = 0;
                    bottomImpact->setVx(vTmp);
                }else{
                    if((vX<(-gravity)*bottomImpact->getFriction()*TIME)&&(vX>(-(-gravity)*bottomImpact->getFriction()*TIME))){
                        vX = 0;
                        aX = 0;
                }
                }
            }
        }
        }catch(const char * e){
            std::cout<<e<<std::endl;
        }
        if(vX<100&&vX>(-100))
            vX+= aX*TIME;
        if(vY<100&&vY>(-100))
            vY+= aY*TIME;
    }
    /// Elmozudulas szamitasa x es y tengelyek menten
    void calculateMove(){
        X=(vX*TIME)+((aX/2)*(TIME*TIME));
        Y=(vY*TIME)+((aY/2)*(TIME*TIME));
    }
    /// Ez a fuggveny keszteti barmilyen akciora az adott CM-t
    void action() {

        this->calculateAcceleration();
        this->calculateVelocity();
        this->calculateMove();
        this->hitbox.move(X,Y);
    }
    /// Getter
    virtual bool isSlope() const {return false;}
    /// Getter
    virtual double getFriction() const {return 0;}
    /// Getter
    virtual int getRGr() const { return 90; }
    /// Getter
    virtual int getLGr() const { return 90; }
    /// Getter
    virtual double getHeight(const double coordX) const{return getPosUEdge().getY();}
};

#endif // COLLISIONMODEL_HPP


