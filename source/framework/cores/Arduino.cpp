#include "arduino.h"
#include "stdlib.h"


int32_t random(int32_t min, int32_t max)
{
  return min + rand() % (max - min);
}

