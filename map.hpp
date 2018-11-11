
/**
 * \file: map.hpp
 *
 */

#ifndef MAP_HPP
#define MAP_HPP

#include "collisionModel.hpp"
//#include "memtrace.h"



enum {ZOOM = 30};

/**
 * Map == Virtualis ter/ Koordinatarendszer
 */
class Map{
    const static size_t maxSize = 100;
    CollisionModel *element[maxSize];
    size_t numElements;
    int width, height;
    Map& operator=(const Map& rhs);
    Map(const Map&);
public:
    /// Konstruktor
    Map() : numElements(0),width(0),height(0) {}
    /// A Maprol az elementet szeretnenk visszakapni ha tombkent kezeljuk
    CollisionModel* operator[](int idx){
        return element[idx];
    }
    /// Getter
    size_t size() {return numElements;}
    /// Getter
    size_t capacity() {return maxSize;}
    /// Uj element felvetele - ha mar nem fer, kivetelt dob
    /// @param P - memoriaban lefoglalando CollisionModel
    void add(CollisionModel *P){
        if((numElements + 1) > maxSize){
            delete P;
            throw std::out_of_range("Map size OutOfRange");
        }else{
        element[numElements] = P;
        numElements++;
        }
    }
    /// A fajlbol valo olvasas soran kapott karakterbol es adatokbol eloallitja a megfelelo elementet es elhelyezi a Map-on/ban
    /// @param c - kapott karakter
    /// @param cnt - azonos karakterek sorozatanak aktualis hossza = szelesseg
    /// @param row - hanyadik sornal tart az olvasas-> height-row+1= y pozicio
    void builder(const char &c,size_t &cnt, const int &row){
        size_t cntt = cnt;
        if(cnt == 0)
            cntt = 1;
        switch(c){
            case 'x':
                add(new MovableCM(width*ZOOM,(height-row+1)*ZOOM,1*ZOOM/2,1*ZOOM/2,0,1,numElements));
                break;
            case '-':
                add(new PersistentCM(((2*width-cntt)/2)*ZOOM,(height-row+1)*ZOOM,cntt*ZOOM/2,1*ZOOM/2,0.2,0.01,90,90,false,numElements));
                break;
            case '\\':
                add(new PersistentCM(((2*width-cntt)/2)*ZOOM,(height-row+1)*ZOOM,cntt*ZOOM/2,1*ZOOM/2,0,0.5,90,45,true,numElements));
                break;
            case '/':
                add(new PersistentCM(((2*width-cntt)/2)*ZOOM,(height-row+1)*ZOOM,cntt*ZOOM/2,1*ZOOM/2,0,0.5,45,90,true,numElements));
                break;
        }
    }
    /// A fajlbol valo olvasas soran kapott karakterek sorrendjebol es kulonbozosegebol fakado logika
    /// @param a - kapott karakter
    /// @param cmp - az a karakter amely elozoleg lett beolvasva / osszehasonlitando karakter
    /// @param cnt - azonos karakterek sorozatanak aktualis hossza
    /// @param rows -  hanyadik sornal tart az olvasas
    void reader(const char &a,const char &cmp,size_t &cnt,int &rows){
        bool isElement = (cmp=='\\' || cmp=='/' || cmp=='x' || cmp=='-');
        if(a==cmp){
            ++cnt;
            ++width;
        }else{
            switch(a){
                    case '\n':
                        if(isElement && height>=rows){
                            builder(cmp,cnt,rows);
                        }
                        cnt = 0;
                        ++rows;
                        width = 0;
                        break;
                    default:
                        if(isElement){
                            builder(cmp,cnt,rows);
                            cnt = 0;
                        }else if(cmp == '.'){
                            cnt = 0;
                        }
                        ++width;
                        break;
                }
        }
    }
    /// Ez olvassa be file-bol az elementeket es tolti be oket a megfelelo helyre
    void load() {
        std::cout<<"Load()"<<std::endl;
        char c0= '.',c1= ',';
        size_t nth_samechar = 0;
        int rows = 0;
        std::ifstream readf;
        readf.open("map.txt");
        if(readf.is_open(), std::ifstream::in){
        std::cout<<"Reading"<<std::endl;
            readf>>height;
            std::cout<<height<<std::endl;
            while(!readf.eof()){
                readf>> std::noskipws >>c0;
                reader(c0,c1,nth_samechar,rows);
                readf>> std::noskipws >>c1;
                reader(c1,c0,nth_samechar,rows);
            }
        }
        readf.close();
        std::cout<<numElements<<std::endl;
    }
    /// Szimulacio minden egyes elementre, egy iteracio = TIME-nak felel meg
    void simulate() {
        for(size_t i = 0;i<numElements;i++){
            setBottomSensor(element[i]);
            setTopSensor(element[i]);
            setLeftSensor(element[i]);
            setRightSensor(element[i]);
            element[i]->action();
        }
    }

    /// Az utkozesmodellek szeleinek egymashoz viszonyitott y-tengely menti viszonyulasat vizsgalja, igazzal ter vissza,
    /// ha szelek pontjainak unioja nem ures halmaz
    /// @param a - statikus utkozesmodell
    /// @param b - mozgasban levo utkozesmodell
    bool axisYCmp(CollisionModel* a, CollisionModel* b){
        return((a->getPosUEdge().getY() >= b->getPosBEdge().getY() && a->getPosBEdge().getY() <= b->getPosBEdge().getY())
                    || ((a->getPosUEdge().getY() >= b->getPosUEdge().getY() && a->getPosBEdge().getY() <= b->getPosUEdge().getY())));
    }
    /// Az utkozesmodellek szeleinek egymashoz viszonyitott x-tengely menti viszonyulasat vizsgalja, igazzal ter vissza,
    /// ha szelek pontjainak unioja nem ures halmaz
    /// @param a - statikus utkozesmodell
    /// @param b - mozgasban levo utkozesmodell
    bool axisXCmp(CollisionModel* a, CollisionModel* b){
        return(((a->getPosLEdge().getX() <= b->getPosREdge().getX()) && (a->getPosREdge().getX() >= b->getPosREdge().getX()))
                    || ((a->getPosLEdge().getX() <= b->getPosLEdge().getX() && a->getPosREdge().getX() >= b->getPosLEdge().getX())));
    }
    /// Balrol valo utkozest vizsgalja, igazzal ter vissza, ha az oldalso szelek tavolsaga megfelelo es egymashoz kepest megfelelo magassagban vannak
    /// @param a - statikus utkozesmodell
    /// @param b - mozgasban levo utkozesmodell
    bool leftCmp(CollisionModel* a, CollisionModel* b){
        return ((a->getPosREdge().getX() >= b->getPosLEdge().getX() && a->getPosLEdge().getX() < b->getPosLEdge().getX())
                   && axisYCmp(a,b));
    }
    /// Jobbrol valo utkozest vizsgalja, igazzal ter vissza, ha a oldalso szelek tavolsaga megfelelo es egymashoz kepest megfelelo magassagban vannak
    /// @param a - statikus utkozesmodell
    /// @param b - mozgasban levo utkozesmodell
    bool rightCmp(CollisionModel* a, CollisionModel* b){
        return ((a->getPosLEdge().getX() <= b->getPosREdge().getX() && a->getPosREdge().getX() > b->getPosREdge().getX())
                   && axisYCmp(a,b));
    }
    /// Felulrol valo utkozest vizsgalja, igazzal ter vissza, ha az also-felso szelek tavolsaga megfelelo es egymashoz kepest megfelelo szelessegben vannak
    /// @param a - statikus utkozesmodell
    /// @param b - mozgasban levo utkozesmodell
    bool topCmp(CollisionModel* a, CollisionModel* b){
        return ((a->getPosBEdge().getY() <= b->getPosUEdge().getY() && a->getPosUEdge().getY() > b->getPosUEdge().getY())
                   && axisXCmp(a,b));
    }
    /// Alulrol valo utkozest vizsgalja, igazzal ter vissza, ha az also-felso szelek tavolsaga megfelelo es egymashoz kepest megfelelo szelessegben vannak
    /// @param a - statikus utkozesmodell
    /// @param b - mozgasban levo utkozesmodell
    bool bottomCmp(CollisionModel* a, CollisionModel* b){
        return ((a->getPosUEdge().getY() >= b->getPosBEdge().getY() && a->getPosBEdge().getY() < b->getPosBEdge().getY())
                   && axisXCmp(a,b));
    }
    /// Megkeresi a mapon azt a modellt amelyikkel a parameterkent megadott utkozesmodell balrol talalkozik
    /// @param cm - utkozesmodell
    void setLeftSensor(CollisionModel* cm) {
        bool set = false;
        for(size_t i = 0; i<size();i++){
            if(leftCmp(element[i],cm)&&(!(element[i]==cm->getBottomImpact()))){
                cm->setLeftImpact(element[i]);
                set = true;
            }
        }
        if(!set)
            cm->setLeftImpact(nullptr);
    }
    /// Megkeresi a mapon azt a modellt amelyikkel a parameterkent megadott utkozesmodell jobbrol talalkozik
    /// @param cm - utkozesmodell
    void setRightSensor(CollisionModel* cm) {
            bool set = false;
            for(size_t i = 0; i<size();i++){
                if(rightCmp(element[i],cm)&&(!(element[i]==cm->getBottomImpact()))){
                    cm->setRightImpact(element[i]);
                    set = true;
                }
            }
        if(!set)
            cm->setRightImpact(nullptr);
    }
    /// Megkeresi a mapon azt a modellt amelyikkel a parameterkent megadott utkozesmodell felulrol talalkozik
    /// @param cm - utkozesmodell
    void setTopSensor(CollisionModel* cm) {
        bool set = false;
            for(size_t i = 0; i<size();i++){
                if(topCmp(element[i],cm)&&(!(element[i]==cm->getLeftImpact() || element[i]==cm->getRightImpact()))){
                    cm->setTopImpact(element[i]);
                    set = true;
                }
            }
        if(!set)
            cm->setTopImpact(nullptr);
    }
    /// Megkeresi a mapon azt a modellt amelyikkel a parameterkent megadott utkozesmodell alulrol talalkozik
    /// @param cm - utkozesmodell
    void setBottomSensor(CollisionModel* cm) {
            bool set = false;
            for(size_t i = 0; i<size();i++){
                if(bottomCmp(element[i],cm)&&(!(element[i]==cm->getLeftImpact() || element[i]==cm->getRightImpact()))){
                    cm->setBottomImpact(element[i]);
                    set = true;
                }
            }
        if(!set)
            cm->setBottomImpact(nullptr);
    }
    /// Map-on szereplo osszes element torlese
    void clear() {
        for(size_t i = 0; i<numElements; i++){
            delete element[i];
        }
        numElements = 0;
    }
    ~Map() {
        for(size_t i = 0; i<numElements; i++){
            delete element[i];
        }
    }
};

#endif // MAP_HPP
