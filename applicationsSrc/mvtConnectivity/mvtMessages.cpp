#include "mvtMessages.hpp"
#include "mvtConnectivityCode.hpp"


void FPCheckMessage::handle(BaseSimulator::BlockCode* bc) {
    mvtConnectivity& mabc = *static_cast<mvtConnectivity*>(bc);
    Catoms3DBlock* module = mabc.module;
    Catoms3DWorld* world = Catoms3D::getWorld();
    P2PNetworkInterface* sender = module->getInterface(senderPos);
    vector<Cell3DPosition> result;
    for (const auto& pair : mabc.graphEdges) {
        const Cell3DPosition& key = pair.first;

        auto it = neighGraph.find(key);
        if (it != neighGraph.end()) {
            if (!pair.second.empty()) {
                result.push_back(key);
                mabc.console << "For the catom at: " << senderPos << " The pos: " << key << " is in common\n";
            }
        }
    }
    if (result.empty()) {
        mabc.console << "Catom: " << sourceInterface->hostBlock->blockId << " Not connected\n";
        mabc.module->setColor(RED);
    }
}