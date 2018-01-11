#include "Block.hpp"

namespace control
{

class SuperBlock : public Block
{
public:
	friend class Block;
// methods
	SuperBlock(SuperBlock *parent, const char *name) :
		Block(parent, name),
		_children()
	{
	}
//	virtual ~SuperBlock();
//	virtual void setDt(float dt);
//	virtual void updateParams();
//	virtual void updateSubscriptions();
//	virtual void updatePublications();
//
//protected:
//// methods
//	List<Block *> &getChildren();
//	void updateChildParams();
//	void updateChildSubscriptions();
//	void updateChildPublications();
//// attributes
//	List<Block *> _children;
};

};
