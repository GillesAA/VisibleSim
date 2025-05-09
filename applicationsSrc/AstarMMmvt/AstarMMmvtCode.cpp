#include "AstarMMmvtCode.hpp"

#include "motion/teleportationEvents.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"


using namespace Catoms3D;

std::queue<std::array<int, 3>> AstarMMmvt::targetQueue;

AstarMMmvt::AstarMMmvt(Catoms3DBlock *host): Catoms3DBlockCode(host) {
    if (not host) return;
    module = static_cast<Catoms3DBlock*>(hostBlock);
}

void AstarMMmvt::startup() {
    // console << "start\n";
    if (isfree) {
        moduleState = FREE;
        if (!AstarMMmvt::targetQueue.empty()) {
            currentTarget = Cell3DPosition(AstarMMmvt::targetQueue.front()[0], AstarMMmvt::targetQueue.front()[1], AstarMMmvt::targetQueue.front()[2]);
            console << currentTarget << "\n";
            AstarMMmvt::targetQueue.pop();
        }
        module->setColor(RED);
        Cell3DPosition currentPosition = module->position;
        for(auto &pos: module->getAllMotions()) {
           graphEdges[currentPosition].push_back(pos.first);
       }
       nbWaitedAnswers = sendHMessageToAllNeighbors(new GraphBuildMessage(currentPosition),10000,1000,0);
    } else {
        hostBlock->setColor(LIGHTGREY);
    }
}

// Function to merge two graph edge maps
void AstarMMmvt::mergeGraphEdges(std::map<Cell3DPosition, std::vector<Cell3DPosition>>& targetGraph, 
    const std::map<Cell3DPosition, std::vector<Cell3DPosition>>& sourceGraph) {

    for (const auto& sourceEdge : sourceGraph) {
        const Cell3DPosition& sourcePos = sourceEdge.first;
        const std::vector<Cell3DPosition>& sourceConnections = sourceEdge.second;

        // Check if this position already exists in the target graph
        if (targetGraph.find(sourcePos) != targetGraph.end()) {
            // Position exists in both graphs, merge the vectors
            std::vector<Cell3DPosition>& targetConnections = targetGraph[sourcePos];

            // For each connection in the source graph
            for (const Cell3DPosition& connection : sourceConnections) {
                // Check if this connection already exists in the target
                if (std::find(targetConnections.begin(), targetConnections.end(), connection) 
                == targetConnections.end()) {
                    targetConnections.push_back(connection);
                }
            }
        } else {
        // Position only exists in source graph, copy it to target
        targetGraph[sourcePos] = sourceConnections;
        }
    }
}

// Heuristic function (Euclidean distance)
double AstarMMmvt::heuristic(const Cell3DPosition& a, const Cell3DPosition& b) {
    return sqrt(pow(a.pt[0] - b.pt[0], 2) + pow(a.pt[1] - b.pt[1] , 2) + pow(a.pt[2]  - b.pt[2] , 2));
}

std::vector<Cell3DPosition> AstarMMmvt::a_star(const std::map<Cell3DPosition, std::vector<Cell3DPosition>>& graphEdges, Cell3DPosition start, Cell3DPosition goal){
    // Comparator for priority queue (min-heap)
    struct Compare {
        bool operator()(const std::pair<int, Cell3DPosition>& a, const std::pair<int, Cell3DPosition>& b) {
            return a.first > b.first;  // Min-heap: lower cost has higher priority
        }
    };
    std::priority_queue<std::pair<int, Cell3DPosition>, std::vector<std::pair<int, Cell3DPosition>>, Compare> frontier;
    frontier.push({0, start});

    // A* Maps for tracking the shortest path and costs
    std::unordered_map<Cell3DPosition, Cell3DPosition, Cell3DPosition::Hash> came_from;
    std::unordered_map<Cell3DPosition, int, Cell3DPosition::Hash> cost_so_far;
    came_from[start] = start;
    cost_so_far[start] = 0;
    
    Cell3DPosition closest = start;
    int closest_h = heuristic(goal, start);  // Initial heuristic

    while (!frontier.empty()) {
        Cell3DPosition current = frontier.top().second;
        frontier.pop();

        int current_h = heuristic(goal, current);
        if (current_h < closest_h) {
            closest_h = current_h;
            closest = current;
        }

        if (current == goal) {
            break;  // Goal reached
        }

        if (graphEdges.find(current) == graphEdges.end()) {
            console << "Node not found: " << current << "\n";
            break;
        }

        for (const Cell3DPosition& next : graphEdges.at(current)) {
            int new_cost = cost_so_far[current] + 1;  // Edge cost = 1

            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                int priority = new_cost + heuristic(goal, next);
                frontier.push({priority, next});
                came_from[next] = current;
            }
        }
    }

    // Decide whether to reconstruct from goal or closest reachable point
    Cell3DPosition endpoint;
    if (came_from.find(goal) != came_from.end()) {
        endpoint = goal;
        moduleState = MOVING;
    } else {
        console << "Goal not reachable, finding path to closest reachable point.\n";
        endpoint = closest;
        moduleState = BRIDGE;
    }

    // Reconstruct path
    std::vector<Cell3DPosition> path;
    for (Cell3DPosition current = endpoint; current != start; current = came_from[current]) {
        path.push_back(current);
        console << "Position in final path: " << current << "\n";
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;

}

void AstarMMmvt::onMotionEnd() {
    console << " has reached its destination\n";
    if(moduleState == MOVING or moduleState == BRIDGE) {
        module->setColor(ORANGE);
        if (!discoveredPath.empty()){
            Cell3DPosition nextPosition;
            while (!discoveredPath.empty() && discoveredPath.front() == module->position) {
                discoveredPath.erase(discoveredPath.begin());
            }
            if (!discoveredPath.empty()) {
                nextPosition = discoveredPath.front();
                for (const Cell3DPosition& pos : discoveredPath) {
                    console << " " << pos;  
                }
                console << "\n";
                console << "Connected Positions: ";
                for(auto &pos: module->getAllMotions()){
                    console << pos.first << " ";
                }
                console << "\n";
            }
            console << "Current Position: " << module -> position << "\n";
            console << "Next Position: " << nextPosition << "\n";
            discoveredPath.erase(discoveredPath.begin()); // Remove it from the path
            getScheduler()->schedule(new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, nextPosition, 
            RotationLinkType::Any, false));
        }
    }
}

void AstarMMmvt::processLocalEvent(EventPtr pev) {
    MessagePtr message;
    stringstream info;

    switch (pev->eventType) {
        case EVENT_RECEIVE_MESSAGE: {
            message = (std::static_pointer_cast<NetworkInterfaceReceiveEvent>(pev))->message;

            if (message->isMessageHandleable()) {
                std::shared_ptr<HandleableMessage> hMsg =
                    (std::static_pointer_cast<HandleableMessage>(message));

                console << " received " << hMsg->getName() << " from "
                        << message->sourceInterface->hostBlock->blockId
                        << " at " << getScheduler()->now() << "\n";
                hMsg->handle(this);
            }
        }
        case EVENT_ADD_NEIGHBOR:{
            console << "ADDED NEIGHBOR \n";
        }break;
        case EVENT_REMOVE_NEIGHBOR:{
            console << "REMOVED NEIGHBOR \n";
            module->setColor(CYAN);
        }break;
        default:
            break;

    }
    Catoms3DBlockCode::processLocalEvent(pev);
}



void AstarMMmvt::onBlockSelected() {
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
}

void AstarMMmvt::onAssertTriggered() {
    console << " has triggered an assert\n";
}

void AstarMMmvt::parseUserBlockElements(TiXmlElement *config) {
    const char *attr = config->Attribute("isfree");
    isfree = (attr?Simulator::extractBoolFromString(attr):false);
}

void AstarMMmvt::parseUserElements(TiXmlDocument *config) {

    TiXmlElement *worldElement = config->RootElement();
    if(worldElement == NULL) return;
    TiXmlElement* targetList = worldElement->FirstChildElement()->NextSiblingElement()->NextSiblingElement()->NextSiblingElement();
    if(!targetList) {
        cerr << "No target positions provided" << endl;
        return;
    }
    TiXmlElement* target = targetList->FirstChildElement();
    TiXmlElement* cell = target->FirstChildElement();
    while(cell){
        string pos = cell->Attribute("position");
        std::array<int, 3> coord;
        std::stringstream ss(pos);
        std::string val;
        int i = 0;
        
        while (std::getline(ss, val, ',') && i < 3) {
            coord[i++] = std::stoi(val);
        }
        targetQueue.push(coord);
        cell = cell->NextSiblingElement("cell");
    }

}

bool AstarMMmvt::parseUserCommandLineArgument(int &argc, char **argv[]) {
    if ((argc > 0) && ((*argv)[0][0] == '-')) {
        switch((*argv)[0][1]) {
            case 'b': {
                cout << "-b option provided" << endl;
                return true;
            } break;

            case '-': {
                std::string varg = std::string((*argv)[0] + 2);
                if (varg == std::string("foo")) {
                    int fooArg;
                    try {
                        fooArg = stoi((*argv)[1]);
                        argc--;
                        (*argv)++;
                    } catch(std::logic_error&) {
                        std::stringstream err;
                        err << "foo must be an integer." << endl;
                        throw CLIParsingError(err.str());
                    }
                    cout << "--foo option provided with value: " << fooArg << endl;
                } else return false;
                return true;
            }

            default: cerr << "Unrecognized command line argument: " << (*argv)[0] << endl;
        }
    }
    return false;
}

std::string AstarMMmvt::onInterfaceDraw() {
    std::stringstream trace;
    trace << "Nb of modules " << BaseSimulator::getWorld()->getNbBlocks()  << "\n" << "Module State: " << moduleState << "\n";
    return trace.str();
}

int AstarMMmvt::sendHMessage(HandleableMessage *msg,P2PNetworkInterface *dest,Time t0,Time dt) {
    return BlockCode::sendMessage(msg, dest, t0, dt);
}

int AstarMMmvt::sendHMessageToAllNeighbors(HandleableMessage *msg, Time t0, Time dt, int nexcept, ...) {
    return BlockCode::sendMessageToAllNeighbors(msg, t0, dt, nexcept);
}

// /************************************************************************
//  ************************* MOTION COORDINATION **************************
//  ***********************************************************************/

//  bool AstarMMmvt::isAdjacentToPosition(const Cell3DPosition& pos) const {
//     return lattice->cellsAreAdjacent(module->position, pos);
// }

// void AstarMMmvt::probeGreenLight() {
//     Cell3DPosition nextPos;

//     Catoms3DBlock *pivot = customFindMotionPivot(module, stepTargetPos, Any);
//     stepPivot = pivot;

//     if (!pivot) {
//         notFindingPivot = true;
//         getScheduler()->schedule(new InterruptionEvent(
//             getScheduler()->now() + getRoundDuration(),
//             module, IT_MODE_FINDING_PIVOT));
//         scheduler->trace("Reattempt finding pivot", module->blockId, PINK);
//         return;
//     }

//     notFindingPivot = false;

//     sendMessage(new ProbePivotLightStateMessage(module->position, stepTargetPos,
//                                                 MeshComponent::Any),
//                 module->getInterface(pivot->position), MSG_DELAY, 0);
// }

// void AstarMMmvt::setGreenLight(bool onoff, int _line_) {
//     stringstream info;
//     info << " light turned ";

//     if (onoff) {
//         info << "green: ";
//         greenLightIsOn = true;
//         module->setColor(GREEN);

//         // Resume flow if needed
//         if (moduleAwaitingGo) {
//             bool nextToModule = isAdjacentToPosition(awaitingModulePos);

//             P2PNetworkInterface* itf = nextToModule ?
//                 module->getInterface(awaitingModulePos) :
//                 // Move the message up the branch
//                 awaitingModuleProbeItf;

//             VS_ASSERT(itf and itf->isConnected());
//             sendMessage(new GreenLightIsOnMessage(module->position, awaitingModulePos),
//                         itf, MSG_DELAY, 0);
//             moduleAwaitingGo = false;
//             awaitingModuleProbeItf = NULL;
//         }
//     } else {
//         info << "red: ";
//         greenLightIsOn = false;
//         module->setColor(RED);
//     }

//     info << _line_;
//     getScheduler()->trace(info.str(),module->blockId, onoff ? GREEN : RED);
// }
