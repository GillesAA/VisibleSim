#include "AstarMMmvtCode.hpp"

#include "motion/teleportationEvents.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"


using namespace Catoms3D;

std::queue<std::array<int, 3>> AstarMMmvt::targetQueue;

AstarMMmvt::AstarMMmvt(Catoms3DBlock *host): Catoms3DBlockCode(host) {
    if (not host) return;

    // Registers a callback (myBroadcastFunc) to the message of type R
    addMessageEventFunc2(GRAPHBUILD_MSG_ID,
                        std::bind(&AstarMMmvt::myGraphBuildFunc,this,
                                    std::placeholders::_1, std::placeholders::_2));
    // Registers a callback (myAcknowledgeFunc) to the message of type K
    addMessageEventFunc2(GRAPHMERGE_MSG_ID,
                        std::bind(&AstarMMmvt::myGraphMergeFunc,this,
                                    std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(PLS_MSG_ID,
                        std::bind(&AstarMMmvt::myPLSFunc, this,
                                    std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(GLO_MSG_ID,
                        std::bind(&AstarMMmvt::myGLOFunc, this,
                                    std::placeholders::_1, std::placeholders::_2));

    module = static_cast<Catoms3DBlock*>(hostBlock);
}

void AstarMMmvt::startup() {
    // console << "start\n";
    if (isfree) {
        moduleState = MOVING;
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
       nbWaitedAnswers = sendMessageToAllNeighbors("flood msg", new MessageOf<Cell3DPosition>(GRAPHBUILD_MSG_ID, currentPosition),10000,1000,0);
    } else {
        hostBlock->setColor(LIGHTGREY);
    }
}

void AstarMMmvt::myGraphBuildFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender){
    MessageOf<Cell3DPosition> *msg = static_cast<MessageOf<Cell3DPosition>*>(_msg.get());
    // Cell3DPosition target = *msg->getData();
    Catoms3DWorld *world = Catoms3D::getWorld();
    if (parent==nullptr){
        Cell3DPosition currentPosition = module->position;
        parent = sender;
        module->setColor(PURPLE);
        auto freeNeighbors = module->getAllFreeNeighborPos();
        for(const auto& [pos, connectorID] : freeNeighbors) {
            graphEdges[pos];
            int connectorintID = static_cast<int>(connectorID);
            console << "Connector: " << connectorintID << ", " << pos << " is connected to : " << "\n";
            for(const auto& [pos1, connectorIDto] : freeNeighbors){
                const Catoms3DMotionRulesLink *linkH = Catoms3DMotionEngine::findPivotConnectorLink(module, connectorID, connectorIDto, HexaFace);
                const Catoms3DMotionRulesLink *linkO = Catoms3DMotionEngine::findPivotConnectorLink(module, connectorID, connectorIDto, OctaFace);
                if ((linkH != nullptr || linkO != nullptr) && !Catoms3DMotionEngine::isBetweenOppositeOrDiagonalBlocks(world->lattice, pos1)){
                    int connectorintIDto = static_cast<int>(connectorIDto);
                    console << "(" << connectorintIDto << ", " << pos1 << ")\n";
                    graphConnectors[connectorID].push_back(connectorIDto);
                    graphEdges[pos].push_back(pos1);
                }
            }
            console << "\n";
        }
        for(int i=0; i<module->getNbInterfaces(); i++) {
            if(module->getInterface(i)->isConnected() and module->getInterface(i) != parent) {
                sendMessage("flood msg",new MessageOf<Cell3DPosition>(GRAPHBUILD_MSG_ID, currentPosition),module->getInterface(i),1000,100);
                nbWaitedAnswers++;
            }
        }
        //This if case only applies if a module is at the end of a line and its only connection is its parent
        if(nbWaitedAnswers==0){
            sendMessage("ack2parent",new MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>>(GRAPHMERGE_MSG_ID, graphEdges),parent,1000,100);
        }
    }
    //Some of the modules that have received a flood will still receive a flood if the module isn't their parent this allows to remove them from the waited answers
    else if(nbWaitedAnswers>0){
        nbWaitedAnswers--;
        console << "nbWait: " << nbWaitedAnswers << "\n";
        if(nbWaitedAnswers == 0) {
            sendMessage("ack2parent", new MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>>(
                GRAPHMERGE_MSG_ID, graphEdges), parent, 1000, 100);
        }
    }
    else{
        sendMessage("ack2parent",new MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>>(GRAPHMERGE_MSG_ID, graphEdges),parent,1000,100);
    }
}

void AstarMMmvt::myGraphMergeFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender){
    MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>> *msg = static_cast<MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>>*>(_msg.get());
    std::map<Cell3DPosition, std::vector<Cell3DPosition>> prevGraph = *msg->getData();
    mergeGraphEdges(graphEdges, prevGraph);
    nbWaitedAnswers--;
    console << "rec. Ack(" << nbWaitedAnswers << ") from " << sender->getConnectedBlockId() << "\n";
    if(nbWaitedAnswers==0){
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

            discoveredPath = a_star(graphEdges, module->position, currentTarget);

            std::ofstream outFile("graph_edges.txt");  // Open a file for writing

            if (outFile.is_open()) {
                outFile << "Graph edges:" << "\n";
                for (const auto& edge : graphEdges) {
                    const Cell3DPosition& source = edge.first;
                    const std::vector<Cell3DPosition>& destinations = edge.second;

                    outFile << "From " << source << " to: ";

                    for (const Cell3DPosition& dest : destinations) {
                        outFile << dest << " ";
                    }
                    outFile << "\n";
                }
                if (discoveredPath.empty()) {
                    outFile << "Path is empty.\n";
                } else {
                    outFile << "Path:";
                    for (const Cell3DPosition& pos : discoveredPath) {
                        outFile << " " << pos;  // No leading comma
                    }
                    outFile << "\n";
                }
                outFile.close();  // Always close the file
            } else {
                std::cerr << "Unable to open file for writing.\n";
            }
            
            if (discoveredPath.empty()) {
                console << "Path is empty.\n";
            } else {
                console << "Path:";
                for (const Cell3DPosition& pos : discoveredPath) {
                    console << " " << pos;  // No leading comma
                }
                console << "\n";
            }

            discoveredPath.erase(discoveredPath.begin());
            getScheduler()->schedule(new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, discoveredPath.front(), 
            RotationLinkType::Any, false));
        }
        else{
            sendMessage("ack2parent",new MessageOf<std::map<Cell3DPosition, std::vector<Cell3DPosition>>>(GRAPHMERGE_MSG_ID, graphEdges),parent,1000,100);
        }
    }
}

void AstarMMmvt::myPLSFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender) {
    MessageOf<Cell3DPosition> *msg = static_cast<MessageOf<Cell3DPosition>*>(_msg.get());
    Cell3DPosition nextPosition = *msg->getData();
    console << "MESSAGE RECEIVED\n";
    sendMessage("ack2parent",new MessageOf<Cell3DPosition>(GLO_MSG_ID, nextPosition),sender,1000,100);
}

void AstarMMmvt::myGLOFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender) {
    MessageOf<Cell3DPosition> *msg = static_cast<MessageOf<Cell3DPosition>*>(_msg.get());
    Cell3DPosition nextPosition = *msg->getData();
    console << "MESSAGE RECEIVED\n";
    getScheduler()->schedule(new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, nextPosition, 
    RotationLinkType::Any, false));
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
   
    switch (pev->eventType) {
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
    trace << "Nb of modules " << BaseSimulator::getWorld()->getNbBlocks()  << "\n";
    return trace.str();
}
