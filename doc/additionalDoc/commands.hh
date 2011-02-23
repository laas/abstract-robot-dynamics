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

/**
   \page abstractRobotDynamics_commands Recommended commands in implementations

   In this page, we provide a list of commands that we recommend
   implementations to support in order to keep some compatibility between different implementations:
   See CjrlDynamicRobot paragraph "Control of the implementation" for details.
   
   \htmlonly
      <div class="memitem">
	<div class="memproto">
	  <table>
	    <tr>
	      <td><h4>Name</h4></td>
	      <td>&nbsp;</td>
	      <td><h4>Value</h4></td>
	      <td>&nbsp;</td>
	      <td><h4>Description</h4></td>
	    </tr>
	    <tr>
	      <td>"ComputeZMP"</td>
	      <td>&nbsp;</td>
	      <td>"true"</td>
	      <td>&nbsp;</td>
	      <td>Require computation of ZMP.</td>
	    </tr>
	    <tr>
	      <td>"ComputeZMP"</td>
	      <td>&nbsp;</td>
	      <td>"false"</td>
	      <td>&nbsp;</td>
	      <td>Do not require computation of ZMP.</td>
	    </tr>
	    <tr><td>&nbsp;</td>
	    </tr>
	    <tr>
	      <td>"ComputeVelocity"</td>
	      <td>&nbsp;</td>
	      <td>"true"</td>
	      <td>&nbsp;</td>
	      <td>Require computation of velocities of bodies.</td>
	    </tr>
	    <tr>
	      <td>"ComputeVelocity"</td>
	      <td>&nbsp;</td>
	      <td>"false"</td>
	      <td>&nbsp;</td>
	      <td>Do not require computation of velocities of bodies.</td>
	    </tr>
	    <tr><td>&nbsp;</td>
	    </tr>
	    <tr>
	      <td>"ComputeCoM"</td>
	      <td>&nbsp;</td>
	      <td>"true"</td>
	      <td>&nbsp;</td>
	      <td>Require computation of robot center of mass.</td>
	    </tr>
	    <tr>
	      <td>"ComputeCoM"</td>
	      <td>&nbsp;</td>
	      <td>"false"</td>
	      <td>&nbsp;</td>
	      <td>Do not require computation of robot center of mass.</td>
	    </tr>
	    <tr><td>&nbsp;</td>
	    </tr>
	    <tr>
	      <td>"ComputeAccelerationCoM"</td>
	      <td>&nbsp;</td>
	      <td>"true"</td>
	      <td>&nbsp;</td>
	      <td>Require computation of robot center of mass acceleration.</td>
	    </tr>
	    <tr>
	      <td>"ComputeAccelerationCoM"</td>
	      <td>&nbsp;</td>
	      <td>"false"</td>
	      <td>&nbsp;</td>
	      <td>Do not require computation of robot center of mass acceleration.</td>
	    </tr>
	    <tr><td>&nbsp;</td>
	    </tr>
	    <tr>
	      <td>"ComputeMomentum"</td>
	      <td>&nbsp;</td>
	      <td>"true"</td>
	      <td>&nbsp;</td>
	      <td>require computation of global momentum and momentum of each body.</td>
	    </tr>
	    <tr>
	      <td>"ComputeMomentum"</td>
	      <td>&nbsp;</td>
	      <td>"false"</td>
	      <td>&nbsp;</td>
	      <td>Do not require computation of global momentum and momentum of each body.</td>
	    </tr>
	    <tr><td>&nbsp;</td>
	    </tr>
	    <tr>
	      <td>"ComputeBackwardDynamics"</td>
	      <td>&nbsp;</td>
	      <td>"true"</td>
	      <td>&nbsp;</td>
	      <td>Require computation of forces and torques on each joint.</td>
	    </tr>
	    <tr>
	      <td>"ComputeBackwardDynamics"</td>
	      <td>&nbsp;</td>
	      <td>"false"</td>
	      <td>&nbsp;</td>
	      <td>Do not require computation of forces and torques on each joint.</td>
	    </tr>
	  </table>
	</div>
      </div>
   \endhtmlonly

*/
