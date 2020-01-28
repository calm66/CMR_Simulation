//$ mex -I../tools/mex/include mex_fdstarplanner.cpp fdstarplanner.cpp

/* Usage
 * 
 * [path, cost] = mex_fdstarplanner(map, freeSpaceValue, startPos, goalPos, updateRect)
 * Inputs:
 * - map: NxM array of uint8 map (cost) data 
 * - freeSpaceValue: scalar, cost value for map data that represents free (walkable) space
 * - startPos: 2-element vector with the robot's start location (x, y) in map-cell coordinates (zero-based)
 * - goalPos: 2-element vector with the goal position (x, y) in map-cell coordinates (zero-based)
 * - updateRect (optional): 4-element vector (left, top, width, height) indicating the bounding rect for map cost changes
 * 
 * outputs:
 * - path: Nx2 matrix of path points, one (x,y) entry per row. Empty if no path exist
 * - cost: accumulated path cost (inf if path is empty)
 */

#include "mex.h"
#include "matrix.h"
#include "fdstarplanner.h"

#define printf mexPrintf

#include <mex/object_manager.hpp>
#include <memory>

enum MethodIds {
    MethodId_Update
};

class Manager: public mex::object_manager<FDStarPlanner, MethodIds> {
public:
    Manager() {        
        addMethod("update", MethodId_Update);
    }
    
    FDStarPlanner *create(const mxArray *mxOpts) {
        std::unique_ptr<FDStarPlanner> p(new FDStarPlanner());
        /* using the unique_ptr here makes this function exception-safe, i. e. in case an error is thrown during update(...), 
         * the planner instance will not leak memory
         */        
        if (mxOpts) update(*p, mxOpts);
        return p.release();
	}

    void invoke(FDStarPlanner &p, MethodIds methodId, const mxArray *mxOpts, int nlhs, mxArray *plhs[]) {                
        if (methodId == MethodId_Update) {
            if (!mxOpts) mexErrMsgTxt("Update method not allowed without arguments");
            update(p, mxOpts);
        }
        
        if (nlhs > 0) {
            const FDStarPlanner::Path &path = p.path();
    
            mxArray *mxPath = mxCreateDoubleMatrix(path.size(), 2, mxREAL);
            double *pPath = mxGetPr(mxPath);
            for(unsigned i = 0; i < path.size(); i++) {
                pPath[i] = path[i].x;
                pPath[i + path.size()] = path[i].y;
            }
            plhs[0] = mxPath;
            
            if (nlhs > 1) plhs[1] = mxCreateString(p.errorMsg().c_str());            
        }
    }
    
    void update(FDStarPlanner &p, const mxArray *mxOpts) {
        const mxArray *mxMap = mxGetField(mxOpts, 0, "map");
        const mxArray *mxObstacleValue = mxGetField(mxOpts, 0, "obstacleValue");
        const mxArray *mxRegions = mxGetField(mxOpts, 0, "updateRegions");
        
        if (mxObstacleValue && !mxMap) mexWarnMsgTxt("Argument \"obstacleValue\" ignored since no \"map\" was given");
        if (mxRegions && !mxMap) mexWarnMsgTxt("Argument \"updateRegions\" ignored since no \"map\" was given");
            
        if (mxMap) {
            if (mxGetNumberOfDimensions(mxMap) != 2 || mxIsEmpty(mxMap) ||
                (!mxIsUint8(mxMap) && !mxIsLogical(mxMap)) ||
                mxIsComplex(mxMap)) mexErrMsgTxt("Invalid format for argument \"map\": expected nonempty uint8 or logical matrix!");
            
            unsigned obstacleValue = 0;
            if (mxObstacleValue) {                
                if (mxIsComplex(mxObstacleValue) || !mex::is_scalar(mxObstacleValue) ||
                    !(mxIsNumeric(mxObstacleValue) || mxIsLogical(mxObstacleValue)))
                    mexErrMsgTxt("Invalid format for argument \"obstacleValue\": expected a noncomplex numeric scalar value");
        
                obstacleValue = (unsigned)mxGetScalar(mxObstacleValue);
            }   
            
            std::vector<FDStarPlanner::Rect> regions;
            if (mxRegions && mxGetM(mxRegions) > 0) {
                if (mxGetNumberOfDimensions(mxRegions) != 2 || mxGetN(mxRegions) != 4 ||
                    mxIsComplex(mxRegions) || !mxIsDouble(mxRegions))
                    mexErrMsgTxt("Invalid format for argument \"updateRegions\": expected a real-valued Mx4 matrix with one region [x, y, w, h] per row.");
                unsigned nRects = mxGetM(mxRegions);
                const double *pRect = mxGetPr(mxRegions);
                for(unsigned i = 0; i < nRects; i++) {   
                    regions[i] = FDStarPlanner::Rect((unsigned)pRect[i], (unsigned)pRect[i + nRects], (unsigned)pRect[i + 2 * nRects], (unsigned)pRect[i + 3 * nRects]);
                }
            }
                                
            if (mxIsUint8(mxMap)) setMap<unsigned char>(p, mxMap, obstacleValue, regions);
            else if (mxIsLogical(mxMap)) setMap<mxLogical>(p, mxMap, obstacleValue, regions);
        }
        
        if (const mxArray *mxStart = mxGetField(mxOpts, 0, "start")) {
            if (mxGetNumberOfElements(mxStart) != 2 || !mxIsDouble(mxStart) || mxIsComplex(mxStart))
                mexErrMsgTxt("Invalid format for argument \"start\": expected a noncomplex 2-element double vector!");
            const double *pStart = mxGetPr(mxStart);
            p.setStart(FDStarPlanner::Position((int)pStart[0], (int)pStart[1]));
        }
        if (const mxArray *mxGoal = mxGetField(mxOpts, 0, "goal")) {
            if (mxGetNumberOfElements(mxGoal) != 2 || !mxIsDouble(mxGoal) || mxIsComplex(mxGoal)) 
                mexErrMsgTxt("Invalid format for argument \"goal\": expected a noncomplex 2-element double vector!");
            const double *pGoal = mxGetPr(mxGoal);
            p.setGoal(FDStarPlanner::Position((int)pGoal[0], (int)pGoal[1]));
        }
	}
            
    template <typename T> 
    void setMap(FDStarPlanner &p, const mxArray *mxMap, T obstacleValue, const std::vector<FDStarPlanner::Rect> &regions) {
        FDStarPlanner::Map<T> map((const T *)mxGetData(mxMap), mxGetN(mxMap), mxGetM(mxMap), obstacleValue);
        map.cellDelta = map.height;
        map.rowDelta = 1 - map.height * map.width;
        if (regions.empty()) p.setMap(map);
        else p.updateMap(map, regions);
    }
};

MEX_DECLARE_OBJECT_MANAGER(Manager)
