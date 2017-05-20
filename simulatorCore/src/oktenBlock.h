/*!
 * \file oktenBlock.h
 * \brief okten Block
 * \date 05/03/2015
 * \author Benoît Piranda
 */

#ifndef OKTENBLOCK_H_
#define OKTENBLOCK_H_

#include <stdexcept>

#include "buildingBlock.h"
#include "oktenBlockCode.h"
#include "oktenGlBlock.h"
#include "cell3DPosition.h"
#include "lattice.h"
#include "utils.h"

using namespace BaseSimulator::utils;

//! \namespace Okten
namespace Okten {

/**
    \brief list of rotations around x,y,z axis applied to the initial catom to obtain orientation of indice orientationCode

    tabOrientationAngles[oc] : rotations done to transform initial catom to oc oriented catom

*/
const float tabOrientationAngles[12][3] = { {0,0,0}, {180.0f,0.0f,-90.0f}, {-90.0f,45.0f,-45.0f},
											{90.0f,45.0f,-135.0f}, {-90.0f,45.0f,135.0f}, {90.0f,45.0f,45.0f},
											{0,0,180.0f}, {180.0f,0,90.0f}, {90.0f,-45.0f,135.0f},
											{-90.0f,-45.0f,45.0f}, {90.0f,-45.0f,-45.0f}, {-90.0f,-45.0f,-135.0f} };

/**
    \brief list of connector positions (x,y,z) in catom local coordinates

    tabConnectorPositions[i] : coordinates of connector #i

*/
const float tabConnectorPositions[12][3] = { {1,0,0}, {0,1,0}, {0.5,0.5,M_SQRT2_2},
											 {-0.5,0.5,M_SQRT2_2},{-0.5,-0.5,M_SQRT2_2},{0.5,-0.5,M_SQRT2_2},
											 {-1,0,0}, {0,-1,0}, {-0.5,-0.5,-M_SQRT2_2},
											 {0.5,-0.5,-M_SQRT2_2},{0.5,0.5,-M_SQRT2_2},{-0.5,0.5,-M_SQRT2_2}};


class OktenBlockCode;

/*! \class OktenBlock
    \brief Special treatement and data for 3D quasi-spherical robot
*/
class OktenBlock : public BaseSimulator::BuildingBlock {
public :
	short orientationCode; //!< number of the connector that is along the x axis.
public:
/**
   \brief Constructor
   \param bId: id of the block
   \param bcd : code block function
*/
	OktenBlock(int bId, BlockCodeBuilder bcb);
	~OktenBlock();

	inline virtual OktenGlBlock* getGlBlock() { return static_cast<OktenGlBlock*>(ptrGlBlock); };
	inline void setGlBlock(OktenGlBlock*ptr) { ptrGlBlock=ptr;};
/**
   \brief Get the interface from the neighbor position in the grid
   \param pos: position of the cell (if in the grid)
   \return return interface if it exists one connected, NULL otherwise */
    inline P2PNetworkInterface *getInterface(SCLattice::Direction d) { return P2PNetworkInterfaces[d]; }
/**
   \brief Get the interface from the interface id
   \param id: interface number
   \return return interface if it exists one connected, NULL otherwise */
	inline P2PNetworkInterface *getInterface(int id) const { return P2PNetworkInterfaces[id]; };
/**
   \brief Get the position of the gridcell in the direction of the given connector
   \param connectorId: id of connector (0..5)
   \param pos: position of the cell (if in the grid)
   \return return true if the cell is in the grid, false otherwise. */
	bool getNeighborPos(short connectorId,Cell3DPosition &pos) const;
/**
   \brief Get the direction id for the corresponding interface
   \param p2p: pointer to the interface
   \return return value [0..5] of the direction according SCLattice::Direction. */
	int getDirection(P2PNetworkInterface*p2p);
/**
   \brief Set the length of connector
   \param connectorId: id of connector (0..5)
   \param length: between 0 (closed) and 1.0 (activated). */
	void setConnectorLength(short connectorId,float length);
/**
   \brief Get the orientation code from the transformation matrix of the catom
   \param mat: homogeneous transformation matrix
   \return return orientation code. */
	static short getOrientationFromMatrix(const Matrix &mat);
/**
   \brief Get the transformation matrix of the catom from its position in the grid and its orientation code
   \param pos: position of the cell constaining the catom
   \param code: orientation code (number of the connector aligned with x axis)
   \return return homogeneous transformation matrix. */
    static Matrix getMatrixFromPositionAndOrientation(const Cell3DPosition &pos,short code);
/**
   \brief Set the catom in the grid according to a cell position and an orientation code
   \param pos: position of the cell constaining the catom
   \param code: orientation code (number of the connector aligned with x axis)*/
	void setPositionAndOrientation(const Cell3DPosition &pos,short code);

	// MeldInterpreter
	/**
	 * @copydoc BuildingBlock::addNeighbor
	 */
	virtual void addNeighbor(P2PNetworkInterface *ni, BuildingBlock* target);
	/**
	 * @copydoc BuildingBlock::removeNeighbor
	 */
	virtual void removeNeighbor(P2PNetworkInterface *ni);

};

std::ostream& operator<<(std::ostream &stream, OktenBlock const& bb);

}

#endif /* OKTENBLOCK_H_ */
