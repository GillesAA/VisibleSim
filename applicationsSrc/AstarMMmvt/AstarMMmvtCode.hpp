/**
 * @file AstarMMmvt.hpp
 * Generated by VisibleSim BlockCode Generator
 * https://services-stgi.pu-pm.univ-fcomte.fr/visiblesim/generator.php#
 * @author yourName
 * @date 2000-12-02                                                                     
 **/

 #ifndef AstarMMmvt_H_
 #define AstarMMmvt_H_
 
 #include "robots/catoms3D/catoms3DSimulator.h"
 #include "robots/catoms3D/catoms3DWorld.h"
 #include "robots/catoms3D/catoms3DBlockCode.h"
 #include "robots/catoms3D/catoms3DMotionEngine.h"
 #include "math/cell3DPosition.h"
 #include <map>
 #include <vector>
 #include <string>
 #include <queue>
 #include <unordered_set>
 #include <unordered_map>
 #include <cmath>
 #include <algorithm>
 #include <fstream>

 #include "AstarMessages.hpp"


 enum States {STATIONARY, FREE, MOVING, BRIDGE};
 
 using namespace Catoms3D;
 using std::string;
 
 class AstarMMmvt : public Catoms3DBlockCode {
 protected:
 

 
 public:
 
     bool isfree= false;
     bool isReturning = false;
     P2PNetworkInterface* parent = nullptr;
     int nbWaitedAnswers = 0;
     std::map<Cell3DPosition, std::vector<std::pair<Cell3DPosition, Cell3DPosition>>> graphEdges;
     std::map<int, std::vector<int>> graphConnectors;
     std::vector<std::pair<Cell3DPosition, Cell3DPosition>> discoveredPath;
     std::vector<std::pair<Cell3DPosition, Cell3DPosition>> inPath;
     Cell3DPosition currentTarget = Cell3DPosition(20, 19, 0);
     States moduleState = STATIONARY;
     bool moduleLightState = true;
     std::queue<P2PNetworkInterface*> trafficQ;
     Cell3DPosition inPos;

     static std::queue<std::array<int, 3>> targetQueue;
     Scheduler *scheduler;
     World *world;
     Lattice *lattice;
     Catoms3DBlock *module = nullptr;
     Catoms3DBlock *pivot = nullptr;
     Catoms3DBlock *prevpivot = nullptr;
     Cell3DPosition nextPosition;

     // Constructor and destructor
     AstarMMmvt(Catoms3DBlock *host);
     ~AstarMMmvt() override {};
 
     // Main startup function
     void startup() override;


     // Graph Functions
     void mergeGraphEdges(std::map<Cell3DPosition, std::vector<std::pair<Cell3DPosition, Cell3DPosition>>>& targetGraph,
        const std::map<Cell3DPosition, std::vector<std::pair<Cell3DPosition, Cell3DPosition>>>& sourceGraph);
 
     double heuristic(const Cell3DPosition &a, const Cell3DPosition &b);
 
     std::vector<std::pair<Cell3DPosition, Cell3DPosition>> a_star(const std::map<Cell3DPosition, std::vector<std::pair<Cell3DPosition, Cell3DPosition>>>& graphEdges,
        Cell3DPosition start, Cell3DPosition goal);
 
     // Event handlers
     void onBlockSelected() override;
     void processLocalEvent(std::shared_ptr<Event> pev) override;
     void onMotionEnd() override;
 
     // GUI interface
     string onInterfaceDraw() override;


     void parseUserBlockElements(TiXmlElement *config);

 
     void parseUserElements(TiXmlDocument *config) override;
 
     // Command-line arguments
     bool parseUserCommandLineArgument(int &argc, char **argv[]) override;
 
     // Additional helper methods
     void onAssertTriggered();

     //MOTION COORDINATION
     bool greenLightIsOn = true;
     bool moduleAwaitingGo = false;
     Cell3DPosition awaitingModulePos = Cell3DPosition(-1, -1, -1);
     P2PNetworkInterface *awaitingModuleProbeItf = NULL;
     Cell3DPosition actuationTargetPos;
     Cell3DPosition stepTargetPos;
     Catoms3DBlock *stepPivot;
     bool notFindingPivot = false;
    
     inline static Time getRoundDuration() {
        Time duration = 0;

        // Simulate actual motion of a catom
        for (int i = 0; i < 2 * Catoms3DRotation::nbRotationSteps; i++) {
            duration += Catoms3DRotation::getNextRotationEventDelay();
        }

        return duration;
    }
 
     bool rotating = false;

     bool isAdjacentToPosition(const Cell3DPosition &pos) const;

     Catoms3DBlock *findTargetLightAmongNeighbors(const Cell3DPosition &targetPos, const Cell3DPosition &srcPos) const;

     Catoms3DBlock *findTargetLightAroundTarget(const Cell3DPosition &targetPos, const Cell3DPosition &finalPos) const;

     void probeGreenLight();

     Catoms3DBlock *customFindMotionPivot(const Catoms3DBlock *m, const Cell3DPosition &tPos, RotationLinkType faceReq);

     vector<Catoms3DBlock *> findNextProbingPoints(const Cell3DPosition &targetPos, const Cell3DPosition &pivotPos);

#define SET_GREEN_LIGHT(x) setGreenLight(x, __LINE__)
    /**
     * Changes the light state of a pivot and take the appriopriate action
     */
    void setGreenLight(bool onoff, int _line_);


    int sendHMessage(HandleableMessage *msg,P2PNetworkInterface *dest,Time t0,Time dt);

    int sendHMessageToAllNeighbors(HandleableMessage *msg,Time t0,Time dt, int nexcept, ...);

     /*****************************************************************************/
     /** Needed to associate code to module                                      **/
     static BlockCode* buildNewBlockCode(BuildingBlock *host) {
         return (new AstarMMmvt((Catoms3DBlock*)host));
     }
     /*****************************************************************************/
 };
 
 #endif 
 
 