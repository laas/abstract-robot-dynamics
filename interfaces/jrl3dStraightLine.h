/*
 *   Copyright (c) 2006 CNRS-LAAS 
 *
 *   Research carried out within the scope of the Associated
 *   International Laboratory: Joint Japanese-French Robotics
 *   Laboratory (JRL)
 *
 *   Developed by Florent Lamiraux (LAAS-CNRS)
 *
 */

#ifndef JRL_3D_STRAIGHT_LINE
#define JRL_3D_STRAIGHT_LINE

/**
   \brief A straight in 3-dimensional space.
*/

class Cjrl3dStraightLine {
public:
  /** 
      \brief Constructor by point and vector.
  */
  Cjrl3dStraightLine(const vector3d& inPoint, const vector3d& inVector) : 
    attPoint(inPoint3d), attVector(inVector) {};

  /**
     \brief Get Point.
  */
  const vector3d& point() {return attPoint;};

  /**
     \brief Get Vector.
  */
  const vector3d& vector() {return attVector;};

private:

  /**
     \brief Point belonging to the line.
  */
  vector3d attPoint;

  /**
     \brief Vector giving the direction of the straight line.
  */
  vector3d attVector;
};

#endif
