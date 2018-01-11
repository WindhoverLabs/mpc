
/**
 * @file Expo.hpp
 *
 * So called exponential curve function implementation.
 * It is essentially a linear combination between a linear and a cubic function.
 * It's used in the range [-1,1]
 */

#ifndef EXPO_HPP
#define EXPO_HPP

namespace math
{

// Type-safe signum function
int signf(float val);
const float expof(const float &value, const float &e);
const float deadzonef(const float &value, const float &dz);
const float expof_deadzone(const float &value, const float &e, const float &dz);

}

#endif
