/*
 * catoms2DBlockCode.h
 *
 *  Created on: 12 janvier 2014
 *      Author: Benoît
 */

#ifndef CATOMS2DBLOCKCODE_H_
#define CATOMS2DBLOCKCODE_H_

#include "base/blockCode.h"
#include "robots/catoms2D/catoms2DBlock.h"
#include "comm/network.h"

namespace Catoms2D {

class Catoms2DBlock;

class Catoms2DBlockCode : public BaseSimulator::BlockCode {
public:

    Catoms2DBlockCode(Catoms2DBlock *host);
    virtual ~Catoms2DBlockCode();

    virtual void processLocalEvent(EventPtr pev) override;
};

}

#endif /* CATOMS2DBLOCKCODE_H_ */
