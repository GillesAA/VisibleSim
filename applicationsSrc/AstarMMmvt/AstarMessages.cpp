
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
        module->setColor(GREEN);
        
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
                    mabc.graphEdges[pos].emplace_back(pos1, currentPosition);
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

// static bool areSamePivotPositions(Catoms3DBlock* a, Catoms3DBlock* b) {
//     if (!a || !b) {
//         std::cout << "Null pivot detected! a=" << a << ", b=" << b << "\n";
//         return false;
//     }
//     return a && b && (a->position == b->position);
// }


void GraphMergeMessage::handle(BaseSimulator::BlockCode* bc) {
    AstarMMmvt& mabc = *static_cast<AstarMMmvt*>(bc);

    mabc.mergeGraphEdges(mabc.graphEdges, graphEdgesData);
    mabc.nbWaitedAnswers--;
    mabc.console << "rec. Ack(" << mabc.nbWaitedAnswers << ") from unknown sender\n"; // or retrieve sender separately if needed

    if (mabc.nbWaitedAnswers == 0) {
        if (mabc.parent == nullptr) {
            mabc.console << "Graph edges:\n";
            for (const auto& edge : mabc.graphEdges) {
                const Cell3DPosition& source = edge.first;
                mabc.console << "From " << source << " to: ";
                for (const auto& [dest, pivot] : edge.second) {
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
                    for (const auto& [dest, pivot] : edge.second) {
                        outFile << dest << " ";
                    }
                    outFile << "\n";
                }

                if (mabc.discoveredPath.empty()) {
                    outFile << "Path is empty.\n";
                } else {
                    outFile << "Path:";
                    for (const auto& [pos, pivot] : mabc.discoveredPath) {
                        outFile << " " << pos;
                    }
                    outFile << "\n";
                }
                outFile.close();
            }

            if (!mabc.discoveredPath.empty()) {
                mabc.discoveredPath.erase(mabc.discoveredPath.begin());
                mabc.nextPosition = mabc.discoveredPath.front().first;
                mabc.pivot = mabc.customFindMotionPivot(mabc.module, mabc.nextPosition, Any);
                mabc.console << "Pivot: " << mabc.pivot->position << "\n";
                if(mabc.pivot == mabc.prevpivot){
                    getScheduler()->schedule(new Catoms3DRotationStartEvent(
                        getScheduler()->now() + 1000, mabc.module, mabc.pivot, mabc.nextPosition,
                        RotationLinkType::Any, false));
                }          
                else if (mabc.prevpivot == nullptr) {
                    cout << "PLSSENDING\n";
                    mabc.sendHMessage(new PLSMessage(mabc.nextPosition, mabc.module->position), mabc.module->getInterface(mabc.pivot->position), 1000, 100);
                }
                else {
                    cout << "FTRSENDING\n";
                    mabc.sendHMessage(new FTRMessage(), mabc.module->getInterface(mabc.prevpivot->position), 1000, 100);
                    mabc.sendHMessage(new PLSMessage(mabc.nextPosition, mabc.module->position), mabc.module->getInterface(mabc.pivot->position), 1000, 100);
                }
            } else {
                mabc.console << "Path is empty.\n";
            }
        } else {
            mabc.sendHMessage(new GraphMergeMessage(mabc.graphEdges),
                mabc.parent, 1000, 100);
        }
    }
}


void PLSMessage::handle(BaseSimulator::BlockCode *bc) {
    AstarMMmvt& mabc = *static_cast<AstarMMmvt*>(bc);
    P2PNetworkInterface* sender = mabc.module->getInterface(senderPos);
    mabc.console << "PLS\n";

    if (mabc.moduleLightState) {
        if(mabc.trafficQ.empty()) {
            mabc.sendHMessage(new GLOMessage(), sender, 1000, 100);
            mabc.moduleLightState = false;
            mabc.module->setColor(RED);
        }
        else {
            mabc.trafficQ.push(sender);
            mabc.sendHMessage(new GLOMessage(), mabc.trafficQ.front(), 1000, 100);
            mabc.trafficQ.pop();
            mabc.moduleLightState = false;
            mabc.module->setColor(RED);
        }
    }
    else {
        mabc.trafficQ.push(sender);
    }
}

void GLOMessage::handle(BaseSimulator::BlockCode *bc) {
    AstarMMmvt& mabc = *static_cast<AstarMMmvt*>(bc);
    mabc.prevpivot = mabc.pivot;
    getScheduler()->schedule(new Catoms3DRotationStartEvent(
        getScheduler()->now() + 1000, mabc.module, mabc.pivot, mabc.nextPosition,
        RotationLinkType::Any, false));
    // // If this catom is the destination, stop routing.
    // if (mabc.module->position == dstPos) {
    //     // Optionally, handle locally or assert
    //     VS_ASSERT(false && "route() called at destination");
    //     return;
    // }

    // // Attempt to send directly to the destination if it's a neighbor
    // P2PNetworkInterface* dstItf = mabc.module->getInterface(dstPos);
    // if (dstItf && dstItf->isConnected()) {
    //     mabc.sendHMessage(this->clone(), dstItf, MSG_DELAY, 0);
    //     return;
    // }

    // // If direct send fails, try sending to any connected neighbor (broadcast-like)
    // for (int i = 0; i < mabc.module->getNbInterfaces(); ++i) {
    //     P2PNetworkInterface* itf = mabc.module->getInterface(i);
    //     if (itf && itf->isConnected()) {
    //         mabc.sendHMessage(this->clone(), itf, MSG_DELAY, 0);
    //     }
    // }

    // You may add logic to prevent infinite forwarding or track visited nodes if needed
}

void FTRMessage::handle(BaseSimulator::BlockCode *bc) {
    AstarMMmvt& mabc = *static_cast<AstarMMmvt*>(bc);
    mabc.console << "FTR\n";
    mabc.module->setColor(GREEN);
}