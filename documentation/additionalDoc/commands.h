/**
   \page abstractRobotDynamics_commands Recommended commands in implementations

   In this page, we provide a list of commands that we recommend
   implementations to support in order to keep some compatibility.
   
   \htmlonly
      <table width=100% cellpadding=5 cellspacing=0>
	<tr><td align=center color:#0066CC>
	    <h4>Name</h4>
	  </td>
	  <td align=center>
	    <h4>Value</h4>
	  </td>
	  <td align=center>
	    <h4>Description</h4>
	  </td>
	</tr>
	<tr><td width="30">
	    "ComputeZMP"
	  </td>
	  <td>
	    "true"
	  </td>
	  <td>
	    Requires computation of ZMP when calling CjrlDynamicRobot::computeForwardKinematics().
	  </td>
	</tr>
	<tr><td width="30">
	    "ComputeZMP"
	  </td>
	  <td>
	    "false"
	  </td>
	  <td>
	    Do not require computation of ZMP when calling CjrlDynamicRobot::computeForwardKinematics().
	  </td>
	</tr>
      </table>
   \endhtmlonly

   

*/
