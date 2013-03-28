
M3 JointArray Command Methods
********************************

.. currentmodule:: m3.joint_array



set_pwm
=================================
.. automethod:: M3JointArray.set_pwm(v,ind=None)

set_torque_mNm
=================================
.. automethod:: M3JointArray.set_torque_mNm(v,ind=None)

set_torque_inLb
=================================
.. automethod:: M3JointArray.set_torque_inLb(v,ind=None)

set_theta_rad
=================================
.. automethod:: M3JointArray.set_theta_rad(v,ind=None)

set_theta_deg
=================================
.. automethod:: M3JointArray.set_theta_deg(v,ind=None)

set_stiffness
=================================
.. automethod:: M3JointArray.set_stiffness(v,ind=None)

set_slew_rate
=================================
.. automethod:: M3JointArray.set_slew_rate(v)

set_slew_rate_proportion
=================================
.. automethod:: M3JointArray.set_slew_rate_proportion(v,ind=None)

set_mode
=================================
.. automethod:: M3JointArray.set_mode(v,ind=None)

set_mode_off
=================================
.. automethod:: M3JointArray.set_mode_off(ind=None)

set_mode_pwm
=================================
.. automethod:: M3JointArray.set_mode_pwm(ind=None)

set_mode_torque
=================================
.. automethod:: M3JointArray.set_mode_torque(ind=None)

set_mode_torque_gc
=================================
.. automethod:: M3JointArray.set_mode_torque_gc(ind=None)

set_mode_theta
=================================
.. automethod:: M3JointArray.set_mode_theta(ind=None)

set_mode_theta_gc
=================================
.. automethod:: M3JointArray.set_mode_theta_gc(ind=None)


set_mode_theta_mj
=================================
.. automethod:: M3JointArray.set_mode_theta_mj(ind=None)

set_mode_theta_gc_mj
=================================
.. automethod:: M3JointArray.set_mode_theta_gc_mj(ind=None)

set_mode_splined_traj
=================================
.. automethod:: M3JointArray.set_mode_splined_traj(ind=None)

set_mode_splined_traj_gc
=================================
.. automethod:: M3JointArray.set_mode_splined_traj_gc(ind=None)

add_splined_traj_via_rad
=================================
.. automethod:: M3JointArray.add_splined_traj_via_rad(theta_des,thetadot_avg)

add_splined_traj_via_deg
=================================
.. automethod:: M3JointArray.add_splined_traj_via_deg(theta_des,thetadot_avg)

is_splined_traj_complete
=================================
.. automethod:: M3JointArray.is_splined_traj_complete()


