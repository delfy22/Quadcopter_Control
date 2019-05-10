#include "EMA.h"


float EMA::getSx(){
  return s_x;
}

float EMA::getSy(){
  return s_y;
}

float EMA::getSz(){
  return s_z;
}

float EMA::getOldSx(){
  return olds_x;
}

float EMA::getOldSy(){
  return olds_y;
}

float EMA::getOldSz(){
  return olds_z;
}

void EMA::setSx (float new_val) {
  s_x = new_val;
}

void EMA::setSy (float new_val) {
  s_y = new_val;
}

void EMA::setSz (float new_val) {
  s_z = new_val;
}

void EMA::calcSx (float new_val) {
  s_x = alpha*new_val + (1 - alpha)*s_x; 
}

void EMA::calcSy (float new_val) {
  s_y = alpha*new_val + (1 - alpha)*s_y; 
}

void EMA::calcSz (float new_val) {
  s_z = alpha*new_val + (1 - alpha)*s_z; 
}

void EMA::updateOldValues () {
  olds_x = s_x;
  olds_y = s_y;
  olds_z = s_z;
}
