#include "main.h"

struct sPos {
  float leftLst;
  float rightLst;
  float backLst;

  float a;
  float x;
  float y;
} position;

pros::ADIEncoder lEncoder (1, 2, true);
pros::ADIEncoder rEncoder (3, 4, true);
pros::ADIEncoder bEncoder (5, 6, false);
