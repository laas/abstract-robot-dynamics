// Copyright 2006, 2007, 2008, 2009, 2010, Florent Lamiraux, CNRS/AIST
//
// This file is part of abstract-robot-dynamics.
// abstract-robot-dynamics is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// abstract-robot-dynamics is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// abstract-robot-dynamics.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef ABSTRACT_ROBOT_DYNAMICS_HAND_HH
# define ABSTRACT_ROBOT_DYNAMICS_HAND_HH
# include <vector>

# include <abstract-robot-dynamics/fwd.hh>
# include <abstract-robot-dynamics/joint.hh>


/// \brief This class represents a robot hand.
///
/// A hand has a central point referenced in the wrist joint frame and
/// three axis.
class CjrlHand
{
public:
  /// \brief Destructor.
  virtual ~CjrlHand() {}

  /// \brief Get the wrist joint to which the hand is attached.
  virtual JrlJointPtr associatedWrist() const = 0;

  /// \brief Get the wrist joint to which the hand is attached
  virtual void setAssociatedWrist(JrlJointPtr inJoint ) = 0;

  /// \brief Get the center of the hand
  ///
  /// \retval outCenter Center of the hand in the frame of the wrist.
  virtual void getCenter(vector3d& outCenter) const = 0;

  /// \brief Set the center of the hand
  ///
  /// \param inCenter Center of the hand in the frame of the wrist.
  virtual void setCenter(const vector3d& inCenter) = 0;

  /// \brief Get thumb axis when had is in open position
  ///
  /// \retval outThumbAxis Axis of the thumb in wrist frame in open position
  virtual void getThumbAxis(vector3d& outThumbAxis) const = 0;

  /// \brief Set thumb axis in wrist frame when had is in open position
  ///
  /// \param inThumbAxis Axis of the thumb in wrist frame in open position
  virtual void setThumbAxis(const vector3d& inThumbAxis) = 0;

  /// \brief Get forefinger axis.
  ///
  /// \retval outForeFingerAxis axis of the forefinger in wrist frame
  /// in open position.
  virtual void getForeFingerAxis(vector3d& outForeFingerAxis) const = 0;

  /// \brief Set forefinger axis.
  ///
  /// \param inForeFingerAxis axis of the forefinger in wrist frame
  ///                         in open position.
  virtual void setForeFingerAxis(const vector3d& inForeFingerAxis) = 0;

  /// \brief Get palm normal
  ///
  /// \retval outPalmNormal normal to the palm in the frame of the wrist.
  virtual void getPalmNormal(vector3d& outPalmNormal) const = 0;

  /// \brief Set palm normal
  ///
  /// \param inPalmNormal normal to the palm in the frame of the wrist.
  virtual void setPalmNormal(const vector3d& inPalmNormal) = 0;
};

#endif //! ABSTRACT_ROBOT_DYNAMICS_HAND_HH
