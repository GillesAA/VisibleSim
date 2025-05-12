
#include "AstarMessages.hpp"
#include "AstarMMmvtCode.hpp"


void GraphBuildMessage::handle(BaseSimulator::BlockCode* bc) {
    AstarMMmvt& mabc = *static_cast<AstarMMmvt*>(bc);
    Catoms3DBlock* module = mabc.module;
    Catoms3DWorld* world = Catoms3D::getWorld();
    P2PNetworkInterface* sender = module->getInterface(senderPos);
    // auto* sender = mabc->module->getInterfaceTo(senderPos);
    
    if (mabc.parent == nullptr) {
        Cell3DPosition currentPosition = module->position;
        mabc.parent = sender;
        module->setColor(PURPLE);
        
        auto freeNeighbors = module->getAllFreeNeighborPos();
        for (const auto& [pos, connectorID] : freeNeighbors) {
            mabc.graphEdges[pos];  // Initialize the list for this position
            int connectorintID = static_cast<int>(connectorID);
            mabc.console << "Connector: " << connectorintID << ", " << pos << " is connected to: \n";
            
            for (const auto& [pos1, connectorIDto] : freeNeighbors) {
                const Catoms3DMotionRulesLink* linkH = Catoms3DMotionEngine::findPivotConnectorLink(module, connectorID, connectorIDto, HexaFace);
                const Catoms3DMotionRulesLink* linkO = Catoms3DMotionEngine::findPivotConnectorLink(module, connectorID, connectorIDto, OctaFace);
                
                if ((linkH != nullptr || linkO != nullptr) && !Catoms3DMotionEngine::isBetweenOppositeOrDiagonalBlocks(world->lattice, pos1)) {
                    int connectorintIDto = static_cast<int>(connectorIDto);
                    mabc.console << "(" << connectorintIDto << ", " << pos1 << ")\n";
                    mabc.graphConnectors[connectorID].push_back(connectorIDto);
                    mabc.graphEdges[pos].push_back(pos1);
                }
            }
            mabc.console << "\n";
        }
        
        for (int i = 0; i < module->getNbInterfaces(); i++) {
            auto* iface = module->getInterface(i);
            if (iface->isConnected() && iface != mabc.parent) {
                mabc.sendHMessage(
                    new GraphBuildMessage(currentPosition),
                    iface, 1000, 100
                );
                mabc.nbWaitedAnswers++;
            }
        }
        
        // This if case only applies if a module is at the end of a line and its only connection is its parent
        if (mabc.nbWaitedAnswers == 0) {
            mabc.sendHMessage(
                new GraphMergeMessage(mabc.graphEdges),
                mabc.parent, 1000, 100
            );
        }
    }
    // Some of the modules that have received a flood will still receive a flood if the module isn't their parent
    // this allows to remove them from the waited answers
    else if (mabc.nbWaitedAnswers > 0) {
        mabc.nbWaitedAnswers--;
        mabc.console << "nbWait: " << mabc.nbWaitedAnswers << "\n";
        if (mabc.nbWaitedAnswers == 0) {
            mabc.sendHMessage(
                new GraphMergeMessage(mabc.graphEdges),
                mabc.parent, 1000, 100
            );
        }
    }
    else {
        mabc.sendHMessage(
            new GraphMergeMessage(mabc.graphEdges),
            mabc.parent, 1000, 100
        );
    }
}

void GraphMergeMessage::handle(BaseSimulator::BlockCode* bc) {
    AstarMMmvt& mabc = *static_cast<AstarMMmvt*>(bc);

    Catoms3DBlock* module = mabc.module;
    mabc.mergeGraphEdges(mabc.graphEdges, graphEdgesData);
    mabc.nbWaitedAnswers--;
    mabc.console << "rec. Ack(" << mabc.nbWaitedAnswers << ") from unknown sender\n"; // or retrieve sender separately if needed

    if (mabc.nbWaitedAnswers == 0) {
        if (mabc.parent == nullptr) {
            mabc.console << "Graph edges:\n";
            for (const auto& edge : mabc.graphEdges) {
                const Cell3DPosition& source = edge.first;
                mabc.console << "From " << source << " to: ";
                for (const Cell3DPosition& dest : edge.second) {
                    mabc.console << dest << " ";
                }
                mabc.console << "\n";
            }

            mabc.discoveredPath = mabc.a_star(mabc.graphEdges, mabc.module->position, mabc.currentTarget);

            std::ofstream outFile("graph_edges.txt");
            if (outFile.is_open()) {
                outFile << "Graph edges:\n";
                for (const auto& edge : mabc.graphEdges) {
                    outFile << "From " << edge.first << " to: ";
                    for (const auto& dest : edge.second) {
                        outFile << dest << " ";
                    }
                    outFile << "\n";
                }

                if (mabc.discoveredPath.empty()) {
                    outFile << "Path is empty.\n";
                } else {
                    outFile << "Path:";
                    for (const Cell3DPosition& pos : mabc.discoveredPath) {
                        outFile << " " << pos;
                    }
                    outFile << "\n";
                }
                outFile.close();
            }

            if (!mabc.discoveredPath.empty()) {
                mabc.discoveredPath.erase(mabc.discoveredPath.begin());
                getScheduler()->schedule(new Catoms3DRotationStartEvent(
                    getScheduler()->now() + 1000, mabc.module, mabc.discoveredPath.front(),
                    RotationLinkType::Any, false));
            } else {
                mabc.console << "Path is empty.\n";
            }
        } else {
            mabc.sendHMessage(new GraphMergeMessage(mabc.graphEdges),
                mabc.parent, 1000, 100);
        }
    }
}


// void GLOMessage::handle(BaseSimulator::BlockCode *bc) {
//     AstarMMmvt& mabc = *static_cast<AstarMMmvt*>(bc);

//     // If this catom is the destination, stop routing.
//     if (mabc.module->position == dstPos) {
//         // Optionally, handle locally or assert
//         VS_ASSERT(false && "route() called at destination");
//         return;
//     }

//     // Attempt to send directly to the destination if it's a neighbor
//     P2PNetworkInterface* dstItf = mabc.module->getInterface(dstPos);
//     if (dstItf && dstItf->isConnected()) {
//         mabc.sendHMessage(this->clone(), dstItf, MSG_DELAY, 0);
//         return;
//     }

//     // If direct send fails, try sending to any connected neighbor (broadcast-like)
//     for (int i = 0; i < mabc.module->getNbInterfaces(); ++i) {
//         P2PNetworkInterface* itf = mabc.module->getInterface(i);
//         if (itf && itf->isConnected()) {
//             mabc.sendHMessage(this->clone(), itf, MSG_DELAY, 0);
//         }
//     }

//     // You may add logic to prevent infinite forwarding or track visited nodes if needed
// }