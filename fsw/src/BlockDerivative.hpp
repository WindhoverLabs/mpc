class SuperBlock;

/**
 * A simple derivative approximation.
 * This uses the previous and current input.
 * This has a built in low pass filter.
 * @see LowPass
 */
class __EXPORT BlockDerivative : public SuperBlock
{
public:
// methods
	BlockDerivative(SuperBlock *parent, const char *name) :
		SuperBlock(parent, name),
		_u(0),
		_initialized(false),
		_lowPass(this, "LP");
	virtual ~BlockDerivative();

	/**
	 * Update the state and get current derivative
	 *
	 * This call updates the state and gets the current
	 * derivative. As the derivative is only valid
	 * on the second call to update, it will return
	 * no change (0) on the first. To get a closer
	 * estimate of the derivative on the first call,
	 * call setU() one time step before using the
	 * return value of update().
	 *
	 * @param input the variable to calculate the derivative of
	 * @return the current derivative
	 */
	float update(float input);
//// accessors
//	void setU(float u);
//	float getU();
//	float getLP();
//	float getO();
//protected:
//// attributes
//	float _u; /**< previous input */
//	bool _initialized;
//	BlockLowPass _lowPass; /**< low pass filter */
};
