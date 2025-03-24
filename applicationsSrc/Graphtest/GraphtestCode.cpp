#include "GraphtestCode.hpp"

#include "motion/teleportationEvents.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"


using namespace Catoms3D;

map<Cell3DPosition, vector<Cell3DPosition>> Graphtest::cells;
vector<Cell3DPosition> Graphtest::visited;
vector<Cell3DPosition> Graphtest::teleportedPositions;

Graphtest::Graphtest(Catoms3DBlock *host): Catoms3DBlockCode(host) {
    if (not host) return;

    // Registers a callback (myBroadcastFunc) to the message of type R
    addMessageEventFunc2(BROADCAST_MSG_ID,
                        std::bind(&Graphtest::myBroadcastFunc,this,
                                    std::placeholders::_1, std::placeholders::_2));
    // Registers a callback (myAcknowledgeFunc) to the message of type K
    addMessageEventFunc2(GRAPHBUILD_MSG_ID,
                        std::bind(&Graphtest::myGraphBuildFunc,this,
                                    std::placeholders::_1, std::placeholders::_2));
    /*
    addMessageEventFunc2(BACK_MSG_ID,
        std::bind(&Graphtest::handleBackMessage, this,
                  std::placeholders::_1, std::placeholders::_2));
*/
    module = static_cast<Catoms3DBlock*>(hostBlock);
}

void Graphtest::startup() {
    // console << "start\n";
    if (module->blockId == 1) {
        module->setColor(RED);
        distance = 0;
        //floodDistance();
        Cell3DPosition currentPosition = module->position;
        cells[currentPosition] = vector<Cell3DPosition>();
        // console << "Printing cells map:\n";
        // for (const auto &entry : cells) {
        //     // entry.first is the key (module->position)
        //     // entry.second is the vector of Cell3DPosition
        //     console << "Position: " << entry.first << " -> ";
        //     console << "Connected Positions: ";
        //     for (const auto &cellPos : entry.second) {
        //         console << cellPos << " "; // Assuming operator<< is defined for Cell3DPosition
        //     }
        //     console << "\n";
        // }
        teleportedPositions.push_back(currentPosition);
        for(auto &pos: module->getAllMotions()) {
           cells[currentPosition].push_back(pos.first);
            visited.push_back(pos.first);
            parentMap[pos.first] = currentPosition;
       }
       sendMessageToAllNeighbors("flood msg", new MessageOf<Cell3DPosition>(BROADCAST_MSG_ID, currentPosition),10000,1000,0);
        // getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, *cells[currentPosition].begin()));
        console << "First position in cells[currentPosition]: " << *cells[currentPosition].begin() << "\n";
    } else {
        distance = -1;
        hostBlock->setColor(LIGHTGREY);
    }
}

void Graphtest::myBroadcastFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender){
    MessageOf<Cell3DPosition> *msg = static_cast<MessageOf<Cell3DPosition>*>(_msg.get());
    Cell3DPosition target = *msg->getData();
    if (parent==nullptr){
        Cell3DPosition currentPosition = module->position;
        parent = sender;
        module->setColor(PURPLE);
        auto freeNeighbors = module->getAllFreeNeighborPos();
        for(const auto& [pos, connectorID] : freeNeighbors) {
            console << "Free position: " << pos << " via connector: " << (int)connectorID << "\n";
            for(const auto& [pos1, connectorIDto] : freeNeighbors){
                const Catoms3DMotionRulesLink *linkH = Catoms3DMotionEngine::findPivotConnectorLink(module, connectorID, connectorIDto, HexaFace);
                const Catoms3DMotionRulesLink *linkO = Catoms3DMotionEngine::findPivotConnectorLink(module, connectorID, connectorIDto, OctaFace);
                if (linkH!=nullptr || linkO!=nullptr){
                    graphEdges[pos].push_back(pos1);
                }
            }
        }
        for(int i=0; i<module->getNbInterfaces(); i++) {
            if(module->getInterface(i)->isConnected() and module->getInterface(i) != parent) {
                sendMessage("flood msg",new MessageOf<Cell3DPosition>(BROADCAST_MSG_ID, currentPosition),module->getInterface(i),1000,100);
                nbWaitedAnswers++;
            }
        }
        //This if case only applies if a module is at the end of a line and its only connection is its parent
        if(nbWaitedAnswers==0){
            console << "Graph edges:" << "\n";
            for (const auto& edge : graphEdges) {
                const Cell3DPosition& source = edge.first;
                const std::vector<Cell3DPosition>& destinations = edge.second;
                
                console << "From "<< source << " to: ";
                
                for (const Cell3DPosition& dest : destinations) {
                    console << dest << " ";
                }
                console << "\n";
            }
            sendMessage("ack2parent",new MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>>(GRAPHBUILD_MSG_ID, graphEdges),parent,1000,100);
        }
    }
    else {
        sendMessage("ack2parent",new MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>>(GRAPHBUILD_MSG_ID, graphEdges),parent,1000,100);
        console << "Graph edges:" << "\n";
        for (const auto& edge : graphEdges) {
            const Cell3DPosition& source = edge.first;
            const std::vector<Cell3DPosition>& destinations = edge.second;
            
            console << "From "<< source << " to: ";
            
            for (const Cell3DPosition& dest : destinations) {
                console << dest << " ";
            }
            console << "\n";
        }
    }
}

void Graphtest::myGraphBuildFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender){
    MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>> *msg = static_cast<MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>>*>(_msg.get());
    std::map<Cell3DPosition, std::vector<Cell3DPosition>> prevGraph = *msg->getData();
    mergeGraphEdges(graphEdges, prevGraph);
    nbWaitedAnswers--;
    console << "rec. Ack(" << nbWaitedAnswers << ") from " << sender->getConnectedBlockId() << "\n";
    if(nbWaitedAnswers<=1){
        if(parent==nullptr){
            console << "Graph edges:" << "\n";
            for (const auto& edge : graphEdges) {
                const Cell3DPosition& source = edge.first;
                const std::vector<Cell3DPosition>& destinations = edge.second;
                
                console << "From "<< source << " to: ";
                
                for (const Cell3DPosition& dest : destinations) {
                    console << dest << " ";
                }
                console << "\n";
            }
        }
        else{
            sendMessage("ack2parent",new MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>>(GRAPHBUILD_MSG_ID, graphEdges),parent,1000,100);
        }
    }
}

// Function to merge two graph edge maps
void Graphtest::mergeGraphEdges(std::map<Cell3DPosition, std::vector<Cell3DPosition>>& targetGraph, 
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
                    // Connection doesn't exist, add it
                    targetConnections.push_back(connection);
                }
            }
        } else {
        // Position only exists in source graph, copy it to target
        targetGraph[sourcePos] = sourceConnections;
        }
    }
}
/*
 void Graphtest::handleSampleMessage(std::shared_ptr<Message> _msg,
                                               P2PNetworkInterface* sender) {
    MessageOf<int>* msg = static_cast<MessageOf<int>*>(_msg.get());
    int d = *msg->getData() + 1;

    console << "Block " << module->blockId << " received distance = " << d
            << " from " << sender->getConnectedBlockId() << "\n";

    if (distance == -1 || d < distance) {
        distance = d;
        parent = sender;
        console << "Block " << module->blockId << " adopts distance = " << distance << "\n";
        floodDistance();
    } else {
        // No improvement, send back
        sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, distance), sender, 100, 200);
    }#include "cell3DPosition.h"
}

void Graphtest::handleBackMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    MessageOf<int> *msg = static_cast<MessageOf<int> *>(_msg.get());
    nbWaitedAnswers--;

    if (nbWaitedAnswers == 0) {
        if (parent) {
            // Not leader, propagate back
            sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, distance), parent, 100, 200);
        } else {
            // Leader: gradient is stable
            console << "Gradient construction complete. All distances are stable.\n";

            // Example: run A* from leader (1) to some goal node (e.g., 5)
            int goalId = 33;
            std::vector<int> path = runAStar(1, goalId);
            if (!path.empty()) {
                console << "A* Path from 1 to " << goalId << ": ";
                for (int id : path) console << id << " ";
                console << "\n";
            } else {
                console << "No path found from 1 to " << goalId << "\n";
            }
        }
    }
}

void Graphtest::floodDistance() {
    nbWaitedAnswers = 0;
    for (int i = 0; i < module->getNbInterfaces(); i++) {
        P2PNetworkInterface* neighborInterface = module->getInterface(i);
        if (neighborInterface && neighborInterface->connectedInterface && neighborInterface != parent) {
            sendMessage("Flood", new MessageOf<int>(SAMPLE_MSG_ID, distance), neighborInterface, 100, 100);
            nbWaitedAnswers++;
        }
    }
    if (nbWaitedAnswers == 0 && parent != nullptr) {
        sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, distance), parent, 100, 200);
    }
}

*/
void Graphtest::onMotionEnd() {
    // console << " has reached its destination\n";
}

void Graphtest::processLocalEvent(EventPtr pev) {
    Catoms3DBlockCode::processLocalEvent(pev);

    switch (pev->eventType) {
        case EVENT_TELEPORTATION_END:
            // Return journey handling
            if (isReturning) {
                if (!discoveredPath.empty()) {
                    // Get the next position in the return path
                    Cell3DPosition nextPosition = discoveredPath.back();
                    discoveredPath.pop_back(); // Remove it from the path

                    // console << "Returning: Teleporting to " << nextPosition << "\n";

                    // Schedule teleportation to the next position
                    getScheduler()->schedule(
                        new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition)
                    );
                } else {
                    // Return journey complete
                    // console << "Return journey complete. Back at the initial position.\n";
                    isReturning = false; // Reset the flag
                }
            }
            // Discovery phase handling
            else if (!visited.empty() && module->position != Cell3DPosition(15, 0, 1)) {
                // Get the next position to explore
                Cell3DPosition nextPosition = visited.back();
                visited.pop_back();
                teleportedPositions.push_back(nextPosition);

                // Track parent-child relationship
                if (parentMap.find(nextPosition) == parentMap.end()) {
                    parentMap[nextPosition] = module->position;
                }

                // Discover adjacent positions
                for (auto &pos : module->getAllMotions()) {
                    cells[module->position].push_back(pos.first);
                    if (std::find(visited.begin(), visited.end(), pos.first) == visited.end() &&
                        std::find(teleportedPositions.begin(), teleportedPositions.end(), pos.first) == teleportedPositions.end()) {
                        visited.push_back(pos.first);
                        parentMap[pos.first] = module->position; // Track parent
                    }
                }

                // Schedule the next teleportation
                getScheduler()->schedule(
                    new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition)
                );
            }
            // Goal reached handling
            else if (module->position == Cell3DPosition(15, 0, 1)) {
                // console << "Goal reached. Reconstructing optimal path...\n";
                isReturning = true; // Set return flag

                // Backtrack from goal to start
                discoveredPath.clear();
                Cell3DPosition current = module->position;

                while (parentMap.find(current) != parentMap.end()) {
                    discoveredPath.push_back(current);
                    current = parentMap[current];
                }
                discoveredPath.push_back(current); // Add the start position

                std::reverse(discoveredPath.begin(), discoveredPath.end()); // Reverse to start-to-goal order

                // Output the optimal path
                console << "Optimal path from start to goal:\n";
                for (auto &pos : discoveredPath) {
                    console << pos << "\n";
                }

                // Start the return journey
                if (!discoveredPath.empty()) {
                    Cell3DPosition nextPosition = discoveredPath.back();
                    discoveredPath.pop_back();

                    // console << "Starting return journey: Teleporting to " << nextPosition << "\n";
                    getScheduler()->schedule(
                        new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition)
                    );
                }
            }
            break;

        default:
            break;
    }
}



void Graphtest::onBlockSelected() {
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
}

void Graphtest::onAssertTriggered() {
    console << " has triggered an assert\n";
}

bool Graphtest::parseUserCommandLineArgument(int &argc, char **argv[]) {
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

std::string Graphtest::onInterfaceDraw() {
    std::stringstream trace;
    trace << "Distance: " << distance;
    return trace.str();
}

/*
 std::vector<int> Graphtest::runAStar(int startId, int goalId) {
    auto *world = (Catoms3DWorld*)BaseSimulator::getWorld();
    if (!world->getBlockById(goalId)) {
        console << "Goal block not found\n";
        return {};
    }

    std::unordered_map<int, NodeRecord> records;
    auto cmp = [&records](int lhs, int rhs) {
        return records[lhs].fScore > records[rhs].fScore;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> openSet(cmp);

    records[startId] = {0.0, heuristic(startId, goalId), -1};
    openSet.push(startId);

    std::unordered_set<int> closedSet;

    while (!openSet.empty()) {
        int current = openSet.top();
        openSet.pop();

        if (current == goalId) {
            return reconstructPath(records, current);
        }

        closedSet.insert(current);

        Catoms3DBlock *currentModule = world->getBlockById(current);
        if (!currentModule) continue;

        for (int i = 0; i < currentModule->getNbInterfaces(); i++) {
            P2PNetworkInterface* neighborInterface = currentModule->getInterface(i);
            if (!neighborInterface || !neighborInterface->connectedInterface) continue;
            int neighborId = neighborInterface->connectedInterface->hostBlock->blockId;

            if (closedSet.find(neighborId) != closedSet.end()) continue;

            double tentative_gScore = records[current].gScore + 1.0;

            if (records.find(neighborId) == records.end() || tentative_gScore < records[neighborId].gScore) {
                records[neighborId].cameFrom = current;
                records[neighborId].gScore = tentative_gScore;
                records[neighborId].fScore = tentative_gScore + heuristic(neighborId, goalId);
                openSet.push(neighborId);
            }
        }
    }

    return {};
}

std::vector<int> Graphtest::reconstructPath(std::unordered_map<int, NodeRecord> &records, int current) {
    std::vector<int> path;
    while (current != -1) {
        path.push_back(current);
        current = records[current].cameFrom;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

double Graphtest::heuristic(int currentId, int goalId) {
    auto *world = (Catoms3DWorld*)BaseSimulator::getWorld();
    Catoms3DBlock *cBlock = world->getBlockById(currentId);
    Catoms3DBlock *gBlock = world->getBlockById(goalId);

    if (!cBlock || !gBlock) return 0.0;

    Graphtest *cCode = dynamic_cast<Graphtest*>(cBlock->blockCode);
    Graphtest *gCode = dynamic_cast<Graphtest*>(gBlock->blockCode);

    if (!cCode || !gCode) return 0.0;

    int dC = cCode->distance;
    int dG = gCode->distance;
    if (dC == -1 || dG == -1) return 0.0;

    // Simple heuristic: absolute difference in their distances from the leader
    return std::fabs((double)dC - (double)dG);
}
*/
