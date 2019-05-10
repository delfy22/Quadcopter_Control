#pragma once
#include "Arduino.h"

class EMA {
public:
  EMA(float a): alpha(a), s_x(0), s_y(0), s_z(0), olds_x(0), olds_y(0), olds_z(0) {} 
  
  float getSx();
  float getSy();
  float getSz();
  
  float getOldSx();
  float getOldSy();
  float getOldSz();

  void setSx (float new_val);
  void setSy (float new_val);
  void setSz (float new_val);
  
  void calcSx (float new_val);
  void calcSy (float new_val);
  void calcSz (float new_val);

  void updateOldValues ();

private:
  float alpha;
  float s_x;
  float s_y;
  float s_z;
  float olds_x;
  float olds_y;
  float olds_z;
};
