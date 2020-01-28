#ifndef FDSTARPLANNER_H
#define FDSTARPLANNER_H

#include <climits>
#include <vector>
#include <algorithm>
#include <string>

class FDStarPlanner {
public:
	
    FDStarPlanner(bool fullInit = false);	
    ~FDStarPlanner();

    /* some simple helper classes */    
    struct Position {
        int x, y;
        Position(int x, int y): x(x), y(y) { }
        Position(): x(0), y(0) { }
        bool operator==(const Position &other) const { return (x == other.x) && (y == other.y); }
    };
    
    struct Rect {
        int x, y;
        unsigned width, height;
        Rect(int x, int y, unsigned width, unsigned height): x(x), y(y), width(width), height(height) { }
        Rect(): x(0), y(0), width(0), height(0) { }
        bool isEmpty() const { return (width == 0 || height == 0); }
        
        bool contains(const Rect &other) const {
            return (other.x >= x && other.y >= y && (other.x + other.width <= x + width) && (other.y + other.height <= y + height));
        }       
        bool contains(const Position &pos) const {
            return (pos.x >= x && pos.y >= y && pos.x < (int)(x + width) && pos.y < (int)(y + height));
        }
    };

    template <typename T = unsigned char>
    struct Map {
        const T *data;
        unsigned width;
        unsigned height;        
        int cellDelta;
        int rowDelta;        
        T obstacleValue;
        Map(const T *data, unsigned width, unsigned height, const T &obstacleValue): 
            data(data), width(width), height(height), cellDelta(1), rowDelta(0), obstacleValue(obstacleValue) { }
        Rect rect() const { return Rect(0, 0, width, height); }
        
        bool isEmpty() const { return (width == 0 && height == 0); }
    };
    
    /* public interface to the planner */
    template <typename T>
    void setMap(const Map<T> &map) {
        _start = Position(-1, -1);
        _goal = Position(-1, -1);
        _path.clear();
        _errorMsg.clear();
        mapWidth = map.width;
        mapHeight = map.height;
        if(map.width > 0 && map.height > 0) {
            initMap(map, std::vector<Rect>());
            accumulatedInputUpdates = NewMap;
        } else freeData();    
    }        
    template <typename T>
    void updateMap(const Map<T> &map, const std::vector<Rect> &updateRegions) {
        if(mapWidth == map.width && mapHeight == map.height) {
            if(updateRegions.empty()) return;
            Rect mapRect = map.rect();
            for(unsigned i = 0; i < updateRegions.size(); i++) {
                if(!mapRect.contains(updateRegions.at(i))) return;
            }

            initMap(map, updateRegions);
            if(!(accumulatedInputUpdates & NewMap)) accumulatedInputUpdates |= UpdatedMap;
            callPlanner();
        } else setMap(map);	        
    }
    
    void setStart(const Position &start);    
    void setGoal(const Position &goal);
    
    typedef std::vector<Position> Path;
    
    const Path &path() const { return _path; }
    const std::string &errorMsg() const { return _errorMsg; }
#ifdef __linux__
    unsigned calcTimeMs() const { return _lastCalcTime; }
#endif

private:
    bool _fullInit;

    unsigned mapWidth, mapHeight;    
    Position _start, _goal;

    Path _path; 
    std::string _errorMsg;

    template <typename T>
    void initMap(const Map<T> &map, const std::vector<Rect> &updateRegions) {
        if(updateRegions.empty()) {
            freeData();
            cells = new (std::nothrow) Cell[mapWidth * mapHeight];
            openHeap = new (std::nothrow) Cell *[mapWidth * mapHeight + 1];
            openListLength = 0;

            if(!cells || !openHeap) return;

            Cell *pCell = cells;
            const T *pCost = map.data;
            for(int y = 0; y < mapHeight; y++) {			
                for(int x = 0; x < mapWidth; x++) {
                    pCell->x = x;
                    pCell->y = y;
                    pCell->backPtr = NULL;
                    pCell->pFocus = NULL;
                    pCell->list = List_New;
                    pCell->heapIndex = 0;
                    pCell->h_cost = pCell->f_cost = pCell->fB_cost = 0;
                    pCell->blocked = (*pCost == map.obstacleValue);				
                    pCost += map.cellDelta;
                    pCell++;
                }			
                pCost += map.rowDelta;
            }	
        } else {
            // It's a map update: incorporate cost changes
            int w = mapWidth;
            int h = mapHeight;
            for(unsigned i = 0; i < updateRegions.size(); i++) {
                const Rect &region = updateRegions.at(i);
                const T *pCost = map.data + (region.y * (w * map.cellDelta + map.rowDelta)) + region.x * map.cellDelta;
                int rowDelta = map.rowDelta + (w - region.width) * map.cellDelta;
                for(int y = region.y; y < (region.y + region.height); y++) {
                    Cell *pCell = cells + w * y + region.x;
                    for(int i = 0; i < region.width; i++) {
                        bool newBlocked = (*pCost == map.obstacleValue);
                        if(newBlocked != pCell->blocked) {
                            pCell->blocked = newBlocked;
                            // add the changed cell itself to the OPEN list
                            if(pCell->list == List_Closed) insert(*pCell, pCell->h_cost);

                            if(!pCell->blocked) {
                                // if a cell has been unblocked, add all neighbors to the open list.
                                // Note: In the paper, only arc cost changes are mentioned, but we are working on cells, i.e. if
                                // chaning a cell's cost this influences all arcs from this cell to its neighbors.
                                for(int iy = pCell->y - 1; iy <= pCell->y + 1; iy++) {
                                    if((unsigned)iy >= (unsigned)h) continue;
                                    for(int ix = pCell->x - 1; ix <= pCell->x + 1; ix++) {
                                        if((unsigned)ix >= (unsigned)w) continue;
                                        Cell *pNeighbor = cells + iy * w + ix;
                                        if(pNeighbor->list == List_Closed) insert(*pNeighbor, pNeighbor->h_cost);
                                    }
                                }
                            }					
                        }
                        pCost += map.cellDelta;
                        pCell++;			
                    }
                    pCost += rowDelta;
                }
            }
        }        
    }
    
    void callPlanner();
    
    enum InputUpdate {
		NoInputUpdates = 0,
		UpdatedStart = 1,
		UpdatedGoal = 2,
		UpdatedMap = 4,
		NewMap = 8
	};
    int accumulatedInputUpdates;

    void doCalculatePath(int updates);
    
	enum ListType {
		List_New,
		List_Open,
		List_Closed,
	};
	struct Cell {
		Cell *backPtr;
		unsigned short x, y;
		// TODO: merge List and blocked into one 32 bit word
		ListType list;
		bool blocked;
		int heapIndex;		
		Cell *pFocus;
		unsigned h_cost, k_cost;
		unsigned f_cost, fB_cost;
		
		inline bool operator<(const Cell &other) {
			if(fB_cost == other.fB_cost) {
				if(f_cost == other.f_cost) return k_cost < other.k_cost;
				else return f_cost < other.f_cost;
			} else return fB_cost < other.fB_cost;
		}
		inline bool operator>=(const Cell &other) { return ! (*this < other); }
	};
	inline unsigned dist(const Cell &c1, const Cell &c2) {
		unsigned dx = abs(c1.x - c2.x);
		unsigned dy = abs(c1.y - c2.y);
		unsigned dMin = std::min(dx, dy);
		unsigned dMax = std::max(dx, dy);
		return 7 * dMin + 5 * (dMax - dMin);
	}
	struct Cost {
		inline Cost(): c1(UINT_MAX), c2(UINT_MAX) { }
		inline Cost(unsigned c1, unsigned c2): c1(c1), c2(c2) { }
		inline bool operator<(const Cost &other) {
			if (c1 == other.c1) return c2 < other.c2;
			else return c1 < other.c1;
		}
		inline bool operator<=(const Cost &other) {
			if(c1 == other.c1) return c2 <= other.c2;
			else return c1 < other.c1;
		}		
		inline bool isNoVal() const { return (c1 == UINT_MAX && c2 == UINT_MAX); }
		unsigned c1, c2;
	};
	
	Cell *cells;	
	Cell **openHeap;
	unsigned openListLength;
	
	Cell *pRobot;
	unsigned d_curr;
	
	void freeData();
	
	Cell *getMinState();
	Cost getMinVal();
    
	inline Cost getCost(const Cell &cell) { return Cost(cell.h_cost + dist(cell, *pRobot), cell.h_cost); }
	Cost processState();
	void insert(Cell &cell, unsigned h_cost);    
    void heapUp(Cell &cell);
    void heapDown(Cell &cell);
#ifdef __linux__    
    unsigned _lastCalcTime;
#endif
};

#endif // FDSTARPLANNER_H
