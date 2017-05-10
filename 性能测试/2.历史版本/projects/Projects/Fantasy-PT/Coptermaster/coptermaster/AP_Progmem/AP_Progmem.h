
#ifndef __AP__H__
#define __AP__H__

#include "../AP_HAL/AP_HAL_Boards.h"
#if defined(__AVR__) 
#include "AP__AVR.h"
#else
//#include "AP__Identity.h"
#endif

#define _STRING(_v, _s)  static const char _v[]  = _s

#endif // __AP__H__

