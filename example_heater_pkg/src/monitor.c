#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "monitor_types.h"
#include "monitor.h"

static float temperature_cpy;

static bool heaton_guard(void) {
  return (temperature_cpy) < ((float)(18.0f));
}

static float heaton_arg0(void) {
  return temperature_cpy;
}

static bool heatoff_guard(void) {
  return (temperature_cpy) > ((float)(21.0f));
}

static float heatoff_arg0(void) {
  return temperature_cpy;
}

void step(void) {
  float heaton_arg_temp0;
  float heatoff_arg_temp0;
  (temperature_cpy) = (temperature);
  if ((heaton_guard)()) {
    {(heaton_arg_temp0) = ((heaton_arg0)());
     (heaton)((heaton_arg_temp0));}
  };
  if ((heatoff_guard)()) {
    {(heatoff_arg_temp0) = ((heatoff_arg0)());
     (heatoff)((heatoff_arg_temp0));}
  };
}
