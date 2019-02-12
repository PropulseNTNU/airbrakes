#include "interpolation.h"
#include "Arduino.h"
// the planed optimal path precalculated. It will be represented as a array off height(index) -> velocity pairs
const float reference_velocity[5] = {100,75,60,55,52};

float getReferenceVelocity(float height){
  unsigned int x0 = floor(height);
  unsigned int x1 = x0 + 1;
  float y0 = reference_velocity[x0];
  float y1 = reference_velocity[x1];
  return ((y0 * (x1 - height)) + (y1 * (height - x0 ))) / (x1 - x0);
}
