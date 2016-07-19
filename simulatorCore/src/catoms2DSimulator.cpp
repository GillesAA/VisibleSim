/*
 * catoms2DSimulator.cpp
 *
 *  Created on: 12 janvier 2014
 *      Author: Benoît
 */

#include <iostream>
#include <string.h>

#include "catoms2DSimulator.h"
#include "trace.h"
#include "utils.h"

using namespace std;
using namespace BaseSimulator::utils;

namespace Catoms2D {

void Catoms2DSimulator::help() {
	cerr << "VisibleSim: " << endl;
	cerr << "Catoms2D" << endl;
	exit(EXIT_SUCCESS);
}

Catoms2DSimulator::Catoms2DSimulator(int argc, char *argv[],
									 Catoms2DBlockCode *(*bcb)(Catoms2DBlock*))
	: BaseSimulator::Simulator(argc, argv, (BlockCodeBuilder)bcb) {
	OUTPUT << "\033[1;34m" << "Catoms2DSimulator constructor" << "\033[0m" << endl;
}

Catoms2DSimulator::~Catoms2DSimulator() {
	OUTPUT << "\033[1;34m" << "Catoms2DSimulator destructor" << "\033[0m" <<endl;
}

void Catoms2DSimulator::createSimulator(int argc, char *argv[],
										Catoms2DBlockCode *(*catoms2DBlockCodeBuildingFunction)
										(Catoms2DBlock*)) {
	simulator =  new Catoms2DSimulator(argc, argv, catoms2DBlockCodeBuildingFunction);
	simulator->parseConfiguration(argc, argv);
	simulator->startSimulation();
}

void Catoms2DSimulator::loadWorld(const Cell3DPosition &gridSize, const Vector3D &gridScale,
								  int argc, char *argv[]) {
    world = new Catoms2DWorld(gridSize, gridScale, argc,argv);
	
	if (GlutContext::GUIisEnabled)
		world->loadTextures("../../simulatorCore/catoms2DTextures");
	
	World::setWorld(world);
}

void Catoms2DSimulator::loadBlock(TiXmlElement *blockElt, int blockId,
								  BlockCode *(*buildingBlockCodeBuildingFunction)(BuildingBlock*),
								  const Cell3DPosition &pos, const Color &color, bool master) {

	// Any additional configuration file parsing exclusive to this type of block should be performed
	//  here, using the blockElt TiXmlElement.

	// @todo: parse angle orientation

	// Finally, add block to the world
	((Catoms2DWorld*)world)->addBlock(blockId, buildingBlockCodeBuildingFunction, pos, color, 0, master);
}

} // catoms2D namespace
