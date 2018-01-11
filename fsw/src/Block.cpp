#include "Block.hpp"



namespace control
{

Block::Block(SuperBlock *parent, const char *name) :
			_name(name),
			_parent(parent),
			_dt(0),
			_subscriptions(),
			_params()
{
	if (getParent() != nullptr)
	{
		getParent()->getChildren().add(this);
	}
}



//void Block::getName(char *name, size_t n)
//{
//	if (getParent() == nullptr) {
//		strncpy(buf, _name, n);
//		// ensure string is terminated
//		buf[n - 1] = '\0';
//
//	} else {
//		char parentName[blockNameLengthMax];
//		getParent()->getName(parentName, n);
//
//		if (!strcmp(_name, "")) {
//			strncpy(buf, parentName, n);
//			// ensure string is terminated
//			buf[n - 1] = '\0';
//
//		} else {
//			snprintf(buf, n, "%s_%s", parentName, _name);
//		}
//	}
//}
//
//
//
//~Block::Block()
//{
//
//}
//
//
//
//void Block::updateParams()
//{
//	BlockParamBase *param = getParams().getHead();
//	int count = 0;
//
//	while (param != nullptr) {
//		if (count++ > maxParamsPerBlock) {
//			char name[blockNameLengthMax];
//			getName(name, blockNameLengthMax);
//			printf("exceeded max params for block: %s\n", name);
//			break;
//		}
//
//		//printf("updating param: %s\n", param->getName());
//		param->update();
//		param = param->getSibling();
//	}
//}
//
//
//
//void Block::updateSubscriptions()
//{
//	uORB::SubscriptionNode *sub = getSubscriptions().getHead();
//	int count = 0;
//
//	while (sub != nullptr) {
//		if (count++ > maxSubscriptionsPerBlock) {
//			char name[blockNameLengthMax];
//			getName(name, blockNameLengthMax);
//			printf("exceeded max subscriptions for block: %s\n", name);
//			break;
//		}
//
//		sub->update();
//		sub = sub->getSibling();
//	}
//}
//
//
//
//void Block::updatePublications()
//{
//	uORB::PublicationNode *pub = getPublications().getHead();
//	int count = 0;
//
//	while (pub != nullptr) {
//		if (count++ > maxPublicationsPerBlock) {
//			char name[blockNameLengthMax];
//			getName(name, blockNameLengthMax);
//			printf("exceeded max publications for block: %s\n", name);
//			break;
//		}
//
//		pub->update();
//		pub = pub->getSibling();
//	}
//}
//
//
//
//void Block::setDt(float dt)
//{
//	Block::setDt(dt);
//	Block *child = getChildren().getHead();
//	int count = 0;
//
//	while (child != nullptr) {
//		if (count++ > maxChildrenPerBlock) {
//			char name[blockNameLengthMax];
//			getName(name, blockNameLengthMax);
//			printf("exceeded max children for block: %s\n", name);
//			break;
//		}
//
//		child->setDt(dt);
//		child = child->getSibling();
//	}
//}
//
//
//
//float Block::getDt()
//{
//	return _dt;
//}
//
//
//
//SuperBlock *Block::getParent()
//{
//	return _parent;
//}
//
//
//
//List<uORB::SubscriptionNode *> &Block::getSubscriptions()
//{
//	return _subscriptions;
//}
//
//
//
//List<uORB::PublicationNode *> &Block::getPublications()
//{
//	return _publications;
//}
//
//
//
//List<BlockParamBase *> &Block::getParams()
//{
//	return _params;
//}
//
//
//
//Block::Block(const control::Block &)
//{
//
//}
//
//
//
//Block Block::operator=(const control::Block &)
//{
//
//}

}
