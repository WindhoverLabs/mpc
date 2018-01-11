


namespace control
{

/**
 */
class Block
{
public:
	friend class BlockParamBase;
//// methods
//	Block(SuperBlock *parent, const char *name);
//	void getName(char *name, size_t n);
//	virtual ~Block();
//	virtual void updateParams();
//	virtual void updateSubscriptions();
//	virtual void updatePublications();
//	virtual void setDt(float dt);
//// accessors
//	float getDt();
//protected:
//// accessors
//	SuperBlock *getParent();
//	List<uORB::SubscriptionNode *> &getSubscriptions();
//	List<uORB::PublicationNode *> &getPublications();
//	List<BlockParamBase *> &getParams();
//// attributes
//	const char *_name;
//	SuperBlock *_parent;
//	float _dt;
//	List<uORB::SubscriptionNode *> _subscriptions;
//	List<uORB::PublicationNode *> _publications;
//	List<BlockParamBase *> _params;

private:
	/* this class has pointer data members and should not be copied (private constructor) */
	Block(const control::Block &);
	Block operator=(const control::Block &);
};

}
