/*
 *   Copyright (c) 2006 CNRS-LAAS 
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Author: Florent Lamiraux (LAAS-CNRS)
 *
 */

#ifndef JRL_DEPRECATED_H
#define JRL_DEPRECATED_H

// This macro allow a generic deprecation of the methods
// It is based on the code found at
// http://stackoverflow.com/questions/295120/c-mark-as-deprecated/295154
#ifdef __GNUC__
#define JRLDEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define JRLDEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement JRLDEPRECATED for this compiler")
#define JRLDEPRECATED(func) func
#endif
//

#endif
