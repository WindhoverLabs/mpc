#include "SuperBlock.hpp"


//SuperBlock::SuperBlock(SuperBlock *parent, const char *name) :
//	Block(parent, name),
//	_children()
//{
//}
//
//
//
//~SuperBlock::SuperBlock()
//{
//};
//
//
//
//void SuperBlock::setDt(float dt)
//{
//}
//
//
//
//void SuperBlock::updateParams()
//{
//	Block::updateParams();
//
//	if (getChildren().getHead() != NULL)
//	{
//		updateChildParams();
//	}
//}
//
//
//
//void SuperBlock::updateSubscriptions()
//{
//	Block::updateSubscriptions();
//
//	if (getChildren().getHead() != NULL) { updateChildSubscriptions(); }
//}
//
//
//
//void SuperBlock::updatePublications()
//{
//	Block::updatePublications();
//
//	if (getChildren().getHead() != NULL)
//	{
//		updateChildPublications();
//	}
//}
//
//
//
//List<Block *> &SuperBlock::getChildren()
//{
//	return _children;
//}
//
//
//
//void SuperBlock::updateChildParams()
//{
//}
//
//
//
//void SuperBlock::updateChildSubscriptions()
//{
//
//}
//
//
//
//void SuperBlock::updateChildPublications()
//{
//
//}
