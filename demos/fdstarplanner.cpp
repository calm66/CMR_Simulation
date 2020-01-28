#include "fdstarplanner.h"
#include <new>
#ifdef __linux__
#include <sys/time.h>
#endif


#define OBSTACLE_COST	2000000000U

FDStarPlanner::FDStarPlanner(bool fullInit):
	_fullInit(fullInit),
    mapWidth(0), mapHeight(0), _start(-1, -1), _goal(-1, -1),
    cells(NULL), openHeap(NULL), openListLength(0)
#ifdef __linux__    
    , _lastCalcTime(0)
#endif
{

}

FDStarPlanner::~FDStarPlanner() {
	freeData();
}
	
void FDStarPlanner::freeData() {
	if(cells) {
		delete[] cells;
		cells = NULL;
	}
	if(openHeap) {
		delete[] openHeap;
		openHeap = NULL;
	}
}

void FDStarPlanner::setStart(const Position &start) {
    if(start.x == _start.x && start.y == _start.y) return;
    if(start.x < 0 || start.y < 0) {
		_start = Position(-1, -1);
		_path.clear();
		return;
	}
	if(start.x < mapWidth && start.y < mapHeight) {
		_start = start;
		accumulatedInputUpdates |= UpdatedStart;
		callPlanner();		
	}
}
void FDStarPlanner::setGoal(const Position &goal) {
    if(goal.x == _goal.x && goal.y == _goal.y) return;
    if(goal.x < 0 || goal.y < 0) {
        _goal = Position(-1, -1);
        _path.clear();
        return;
    }
    if(goal.x < mapWidth && goal.y < mapHeight) {
        _goal = goal;
        accumulatedInputUpdates |= UpdatedGoal;
        callPlanner();
    }
}

void FDStarPlanner::callPlanner() {
    if(mapWidth > 0 && mapHeight > 0 && _start.x >= 0 && _start.y >= 0 && _goal.x >= 0 && _goal.y >= 0) {
        _errorMsg.clear();
		_path.clear();
#ifdef __linux__
        struct timeval before, after;
        gettimeofday(&before, NULL);
#endif
        doCalculatePath(accumulatedInputUpdates);
#ifdef __linux__
        gettimeofday(&after, NULL);
		_lastCalcTime = (unsigned long long)(after.tv_sec - before.tv_sec) * 1000 + (after.tv_usec - before.tv_usec) / 1000;
#endif        
        accumulatedInputUpdates = 0;
		
		if(_path.empty() && _errorMsg.empty()) _errorMsg = "No Path set";
	}	
}

void FDStarPlanner::doCalculatePath(int updates) {
    // printf("doCalculatePath(0x%02X)\n", updates);
    
    if(!cells || !openHeap) {
		_errorMsg = "Planner memory allocation error";
		return;
	}
	
	Cell *pStart = cells + _start.y * mapWidth + _start.x;
	Cell *pGoal = cells + _goal.y * mapWidth + _goal.x;
    
	// check validity of start & goal
	if(pStart->blocked) {
		_errorMsg = "Start position blocked";
		return;
	} else if(pGoal->blocked) {
		_errorMsg = "Goal position blocked";
		return;
	}

	// if reusing knowledge from previous calls is not possible, (re-)initialize planner state
	if(updates & ~(UpdatedStart | UpdatedMap)) {
		for(Cell *pCell = cells; pCell < (cells + mapWidth * mapHeight); pCell++) {
			pCell->list = List_New;
			pCell->backPtr = NULL;
			pCell->heapIndex = 0;
			pCell->h_cost = 0;
		}

		pRobot = pStart;
		d_curr = 0;
		
		openListLength = 1;
		pGoal->fB_cost = pGoal->f_cost = pGoal->h_cost = pGoal->k_cost = 0;
		pGoal->list = List_Open;
		pGoal->heapIndex = 1;
		openHeap[1] = pGoal;
	}
	
	bool success = true;
    if(updates & (NewMap | UpdatedGoal)) {
        while(true) {
            Cost val = processState();
            if(!_fullInit && pStart->list == List_Closed) break;
            if(openListLength == 0) break;
            if(val.c2 >= OBSTACLE_COST) break; // no error handling here since pStart is checked for valid costs after the loop
        }
        if(success && (pStart->list != List_Closed || pStart->h_cost >= OBSTACLE_COST)) {
            _errorMsg = "No Path found";
            success = false;
        }

    } else {			
        if(pStart != pRobot) {
            d_curr += dist(*pStart, *pRobot) + 1;			
            pRobot = pStart;	
        }		

        Cost val = getMinVal();
        if(pStart->list == List_New || val < getCost(*pStart)) {
            while(openListLength > 0) {
                val = processState();

                if(pStart->list != List_New && getCost(*pStart) <= val) break;

                if(val.c2 >= OBSTACLE_COST) {
                    _errorMsg = "No Path found";
                    success = false;
                    break;
                }
            }
        }

        if(success && (pStart->list == List_New || pStart->h_cost >= OBSTACLE_COST)) {
            _errorMsg = "No Path found";
            success = false;
        }
    }
	
	// prepare debug layers
	const Cell *pCell = cells;
	
    Path p;
    
	if(success) {
		// follow the backpointers to construct the path
		Cell *pCell = pStart;
		
		int pathLength = 0;	
		while(true) {
			pathLength++;
			if(pCell->blocked) {
				// sanity check: path planned through blocked area
				_errorMsg = "Path blocked";
				success = false;
				break;
			}
			if(pCell == pGoal) break;

			pCell = pCell->backPtr;
			if(!pCell) {
				// internal error: backpointers do not form a sequence
				_errorMsg = "NULL pointer in backpointer sequence";
				success = false;
				break;
			}
			if(pathLength > 1000000) {
				// sanity check: probably loop in backpointer sequence
				_errorMsg = "Path too long";
				success = false;
				break;
			}
		}
		if(success) {
			// path is valid -> store it			
			p.resize(pathLength);
			pathLength = 0;
			pCell = pStart;
			while(true) {		
				p[pathLength++] = Position(pCell->x, pCell->y);				
				if(pCell == pGoal) break;
				pCell = pCell->backPtr;
			}
		}
	}

	_path = p;
}

FDStarPlanner::Cell *FDStarPlanner::getMinState() {
	if(openListLength > 0) {
		while(true) {
			Cell *pMin = openHeap[1];
			if(pMin->pFocus == pRobot) return pMin;
			
			// correct f[B]_cost and reposition in the heap
			pMin->f_cost = pMin->k_cost + dist(*pMin, *pRobot);
			pMin->fB_cost = pMin->f_cost + d_curr;
			pMin->pFocus = pRobot;
			heapUp(*pMin);
			heapDown(*pMin);		
		}
	}
	return NULL;
}

FDStarPlanner::Cost FDStarPlanner::getMinVal() {
	Cell *pMin = getMinState();
	if(pMin) return Cost(pMin->f_cost, pMin->k_cost);
	else return Cost();
}

FDStarPlanner::Cost FDStarPlanner::processState() {
    Cell *pMin = getMinState();
	// error if open list is empty
	if(!pMin) return Cost();
	
	// remove first entry from the open list
	pMin->list = List_Closed;
	pMin->heapIndex = 0;	
	if(--openListLength){
		openHeap[1] = openHeap[openListLength + 1]; //move the last item in the heap up to slot #1
		openHeap[1]->heapIndex = 1;
		heapDown(*openHeap[1]);
	}	
	Cost val(pMin->f_cost, pMin->k_cost);
	unsigned k_val = pMin->k_cost;
	
	Cell *pNeighbors[8];
	unsigned c_cost[8];
	unsigned numNeighbors = 0;
	for(int y = pMin->y - 1; y <= pMin->y + 1; y++) {
		for(int x = pMin->x - 1; x <= pMin->x + 1; x++) {
			if(y == pMin->y && x == pMin->x) continue;
			if((unsigned)y >= (unsigned)mapHeight || (unsigned)x >= (unsigned)mapWidth) continue;			
			Cell *pNeighbor = cells + y * mapWidth + x;
			pNeighbors[numNeighbors] = pNeighbor;
			if(pNeighbor->blocked || pMin->blocked) c_cost[numNeighbors] = OBSTACLE_COST;
			else c_cost[numNeighbors] = (x != pMin->x && y != pMin->y) ? 7 : 5;
			numNeighbors++;
		}		
	}
	
	if(k_val < pMin->h_cost) {
		for(unsigned i = 0; i < numNeighbors; i++) {
			Cell *pNeighbor = pNeighbors[i];
			if(pNeighbor->list == List_New) continue;
			if(getCost(*pNeighbor) <= val) { // neighbor with h=o (optimal cost)
				unsigned newHCost = c_cost[i];
				if(newHCost < OBSTACLE_COST) newHCost += pNeighbor->h_cost;
				if(pMin->h_cost > newHCost) {
					pMin->h_cost = newHCost;
					pMin->backPtr = pNeighbor;
				}
			}
		}		
	}
	
	if(k_val == pMin->h_cost) {
		for(unsigned i = 0; i < numNeighbors; i++) {
			Cell *pNeighbor = pNeighbors[i];
			unsigned neighborHCost = c_cost[i];
			if(neighborHCost < OBSTACLE_COST) neighborHCost += pMin->h_cost;			
			if(pNeighbor->list == List_New || (pNeighbor->h_cost > neighborHCost) ||
			   (pNeighbor->backPtr == pMin && pNeighbor->h_cost != neighborHCost)) {				   
				pNeighbor->backPtr = pMin;
				insert(*pNeighbor, neighborHCost);				
			}			
		}
	} else {
		for(unsigned i = 0; i < numNeighbors; i++) {
			Cell *pNeighbor = pNeighbors[i];
			unsigned neighborHCost = c_cost[i];
			if(neighborHCost < OBSTACLE_COST) neighborHCost += pMin->h_cost;
			
			if(pNeighbor->list == List_New ||
			   (pNeighbor->backPtr == pMin && pNeighbor->h_cost != neighborHCost)) {
				pNeighbor->backPtr = pMin;
				insert(*pNeighbor, neighborHCost);				
			} else if(pNeighbor->backPtr != pMin) {
				if(pNeighbor->h_cost > neighborHCost) {
					insert(*pMin, pMin->h_cost);										
				} else {
					unsigned hCost = c_cost[i];
					if(hCost < OBSTACLE_COST) hCost += pNeighbor->h_cost;
					if(pMin->h_cost > hCost && pNeighbor->list == List_Closed && val < getCost(*pNeighbor)) {
						insert(*pNeighbor, pNeighbor->h_cost);
					}
				}
			}
		}
	}

	return getMinVal();
}

// move element to the beginning of the heap (lower key values) as far as possible
void FDStarPlanner::heapUp(Cell &cell) {
	if(cell.list != List_Open) return;
	unsigned idx = cell.heapIndex;
	while(idx != 1) {
		unsigned parentIdx = idx >> 1;
		if(*openHeap[idx] < *openHeap[parentIdx]) {
			Cell *temp = openHeap[parentIdx];
			openHeap[parentIdx] = openHeap[idx];
			openHeap[idx] = temp;
			openHeap[parentIdx]->heapIndex = parentIdx;
			openHeap[idx]->heapIndex = idx;
			idx = parentIdx;
		} else break;
	}
}

// move element away from the beginning of the heap (to higher key values) as far as possible
void FDStarPlanner::heapDown(Cell &cell) {
	if(cell.list != List_Open) return;
		
	unsigned idx = cell.heapIndex;
	// Repeat the following until the item sinks to its proper spot in the heap.
	while(1){
		unsigned origIdx = idx;
		unsigned idxChild = idx << 1;		
		if(idxChild <= openListLength) {
			if(*openHeap[idx] >= *openHeap[idxChild]) idx = idxChild;
		}	
		idxChild++;
		if(idxChild <= openListLength) {
			if(*openHeap[idx] >= *openHeap[idxChild]) idx = idxChild;
		}
		
		if(origIdx != idx) {
			Cell *temp = openHeap[idx];
			openHeap[idx] = openHeap[origIdx];
			openHeap[origIdx] = temp;
			openHeap[idx]->heapIndex = idx;
			openHeap[origIdx]->heapIndex = origIdx;
		} else break;
	}
}

void FDStarPlanner::insert(Cell &cell, unsigned h_cost) {
	if(cell.list == List_Open) {
		if(h_cost < cell.k_cost) cell.k_cost = h_cost;
		cell.f_cost = cell.k_cost + dist(cell, *pRobot);
		cell.fB_cost = cell.f_cost + d_curr;
		heapUp(cell);
		heapDown(cell);
	} else {
		if(cell.list == List_New) cell.k_cost = h_cost;
		else cell.k_cost = std::min(cell.h_cost, h_cost);		
		cell.f_cost = cell.k_cost + dist(cell, *pRobot); 
		cell.fB_cost = cell.f_cost + d_curr;
		// insert element
		int idx = ++openListLength;
		openHeap[idx] = &cell;
		cell.heapIndex = idx;
		cell.list = List_Open;
		heapUp(cell);
	}
	cell.h_cost = h_cost;	
	cell.pFocus = pRobot;	
}
