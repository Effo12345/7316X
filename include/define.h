#include "main.h"

#ifndef DEFINE_H
#define DEFINE_H

extern struct sPos {
  float leftLst;
  float rightLst;
  float backLst;

  float a;
  float x;
  float y;
} position;

extern pros::ADIEncoder lEncoder;
extern pros::ADIEncoder rEncoder;
extern pros::ADIEncoder bEncoder;


#endif
