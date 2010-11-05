/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 * Florent Lamiraux
 *
 * CNRS/AIST
 *
 * This file is part of abstract-robot-dynamics.
 * abstract-robot-dynamics is free software: you can redistribute it
 * and/or modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * abstract-robot-dynamics is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with abstract-robot-dynamics.  If not, see
 * <http://www.gnu.org/licenses/>.
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
