# xArm-Python-SDK

## Overview
xArm Python SDK

## Caution
- Please keep away from the robot arm to avoid personal injury or equipment damage.
- Make sure to do a safety assessment before moving to prevent collisions.
- Protect the arm before unlocking the joint.

## Installation
&ensp;&ensp;you can run examples without installation.Only Python3 is supported.
- download

  ```bash
  git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
  ```

- install

  ```bash
  python setup.py install
  ```

## Doc
- #### [API Document](doc/api/xarm_api.md)

- #### [API Code Document](doc/api/xarm_api_code.md)

- #### [UFACTORY ModbusTCP Manual](doc/UF_ModbusTCP_Manual.md)

## Update Summary
- > ### 1.13.0
  - Compatible with the standard Modbus TCP protocol, providing part of the standard Modbus TCP protocol interface

- > ### 1.12.2
  - Support partial Task feedback (requires firmware version greater than or equal to v2.1.0)

- > ### 1.11.6
  - Correct the ambiguity that the `set_position_aa` interface is true when both relative and is_tool_coord are true. After the correction, when is_tool_coord is true, relative is invalid (previously is_tool_coord was invalid when relative was true)

- > ### 1.11.5
  - Optimization pause time is too long (wait=true)
  - Add common motion api (Enabled after firmware version 1.11.100)
  - The Cartesian motion-related interface adds the motion_type parameter to determine the planning method (Enabled after firmware version 1.11.100)

- > ### 1.11.0
  - Support transparent transmission
    - 240: `set_tgpio_modbus_timeout(..., is_transparent_transmission=True)`
    - 241: `getset_tgpio_modbus_data(..., is_transparent_transmission=True)`
  - Modified the centroid unit of the `ft_sensor_iden_load` and `ft_sensor_cali_load` interfaces to millimeters (originally meters)


- >### [More](ReleaseNotes.md)


## [Example](example/wrapper/)

&ensp;&ensp; __Before running the example, please modify the IP in [robot.conf](example/wrapper/robot.conf) to corresponding IP you want to control.__

- #### [0000-template](example/wrapper/common/0000-template.py)

- ##### [0001-event_register](example/wrapper/common/0001-event_register.py)

- ##### [0002-get_property](example/wrapper/common/0002-get_property.py)

- ##### [0003-api_get](example/wrapper/common/0003-api_get.py)

- ##### [0004-servo_attach_detach](example/wrapper/common/0004-servo_attach_detach.py)

- ##### [1001-move_line](example/wrapper/common/1001-move_line.py)

- #####  [1002-move_line](example/wrapper/common/1002-move_line.py)

- #####  [1003-relative_move_line](example/wrapper/common/1003-relative_move_line.py)

- #####  [1004-move_arc_line](example/wrapper/common/1004-move_arc_line.py)

- #####  [1005-move_arc_line](example/wrapper/common/1005-move_arc_line.py)

- #####  [1006-move_tool_line](example/wrapper/common/1006-move_tool_line.py)

- #####  [1007-counter](example/wrapper/common/1007-counter.py)

- #####  [1008-move_line_aa](example/wrapper/common/1008-move_line_aa.py)

- #####  [1009-cartesian_velocity_control](example/wrapper/common/1009-cartesian_velocity_control.py)

- #####  [1010-cartesian_online_trajectory_planning](example/wrapper/common/1010-cartesian_online_trajectory_planning.py)

- #####  [2000-joint_velocity_control](example/wrapper/common/2000-joint_velocity_control.py)
  
- #####  [2006-joint_online_trajectory_planning](example/wrapper/common/2006-joint_online_trajectory_planning.py)

- ##### 2001-move_joint --> [xarm5](example/wrapper/xarm5/2001-move_joint.py) --- [xarm6](example/wrapper/xarm6/2001-move_joint.py) --- [xarm7](example/wrapper/xarm7/2001-move_joint.py)

- ##### 2002-move_joint --> [xarm5](example/wrapper/xarm5/2002-move_joint.py) --- [xarm6](example/wrapper/xarm6/2002-move_joint.py) --- [xarm7](example/wrapper/xarm7/2002-move_joint.py)

- ##### 2003-move_joint --> [xarm5](example/wrapper/xarm5/2003-move_joint.py) --- [xarm6](example/wrapper/xarm6/2003-move_joint.py) --- [xarm7](example/wrapper/xarm7/2003-move_joint.py)

- ##### 2004-move_joint --> [xarm5](example/wrapper/xarm5/2004-move_joint.py) --- [xarm6](example/wrapper/xarm6/2004-move_joint.py) --- [xarm7](example/wrapper/xarm7/2004-move_joint.py)

- ##### 2005-move_arc_joint --> [xarm5](example/wrapper/xarm5/2005-move_arc_joint.py) --- [xarm6](example/wrapper/xarm6/2005-move_arc_joint.py) --- [xarm7](example/wrapper/xarm7/2005-move_arc_joint.py)

- ##### [3001-move_circle](example/wrapper/common/3001-move_circle.py)

- ##### [3002-record_trajectory](example/wrapper/common/3002-record_trajectory.py)

- ##### [3003-playback_trajectory](example/wrapper/common/3003-playback_trajectory.py)
  
- ##### [3004-get_report_data](example/wrapper/common/3004-get_report_data.py)
  
- ##### [3005-task_feedback](example/wrapper/common/3005-task_feedback.py)

- ##### [3006-standard_modbus_tcp](example/wrapper/common/3006-standard_modbus_tcp.py)

- ##### [5000-set_tgpio_modbus](example/wrapper/common/5000-set_tgpio_modbus.py)

- ##### [5001-get_tgpio_digital](example/wrapper/common/5001-get_tgpio_digital.py)

- ##### [5002-get_tgpio_analog](example/wrapper/common/5002-get_tgpio_analog.py)

- ##### [5003-set_tgpio_digital](example/wrapper/common/5003-set_tgpio_digital.py)

- ##### [5004-set_gripper](example/wrapper/common/5004-set_gripper.py)

- ##### [5005-get_cgpio_digital_analog](example/wrapper/common/5005-get_cgpio_digital_analog.py)

- ##### [5006-set_cgpio_digital_analog](example/wrapper/common/5006-set_cgpio_digital_analog.py)

- ##### [5007-set_cgpio_input_output_function](example/wrapper/common/5007-set_cgpio_input_output_function.py)

- ##### [5008-get_cgpio_state](example/wrapper/common/5008-get_cgpio_state.py)

- ##### [5009-set_bio_gripper](example/wrapper/common/5009-set_bio_gripper.py)

- ##### [6001-set_reduced_mode](example/wrapper/common/6001-set_reduced_mode.py)

- ##### [6002-set_fense_mode](example/wrapper/common/6002-set_fense_mode.py)

- ##### [7001-servo_j](example/wrapper/common/7001-servo_j.py)

- ##### [7002-servo_cartesian](example/wrapper/common/7002-servo_cartesian.py)

- ##### [7003-servo_cartesian_aa](example/wrapper/common/7003-servo_cartesian_aa.py)

- ##### [8000-load_identify_current](example/wrapper/common/8000-load_identify_current.py)

- ##### [8001-force_tech](example/wrapper/common/8001-force_tech.py)

- ##### [8002-impedance](example/wrapper/common/8002-impedance.py)

- ##### [8003-force_control](example/wrapper/common/8003-force_control.py)

- ##### [8004-load_identify](example/wrapper/common/8004-load_identify.py)

- ##### [8005-read_force_data](example/wrapper/common/8005-read_force_data.py)

- ##### [8006-save_force_zero](example/wrapper/common/8006-save_force_zero.py)

- ##### [8010-get_ft_sensor_config](example/wrapper/common/8010-get_ft_sensor_config.py)

- ##### [9000-set_linear_track](example/wrapper/common/9000-set_linear_track.py)

- ##### [blockly_to_python](example/wrapper/tool/blockly_to_python.py)

- ##### [get_report_data_with_protocol](example/wrapper/common/get_report_data_with_protocol.py)


- #### Import
  ```python
  from xarm.wrapper import XArmAPI
  arm = XArmAPI('COM5')
  arm = XArmAPI('192.168.1.113')
  arm = XArmAPI('192.168.1.113', do_not_open=False)
  arm = XArmAPI('192.168.1.113', is_radian=False)
  ```

- #### Connect/Disconnect
  ```python
  arm.connect(...)
  arm.disconnect()
  ```

- #### Move
  ```python
  arm.reset(...)
  arm.set_position(...)
  arm.set_servo_angle(...)
  arm.set_servo_angle_j(...)
  arm.set_servo_cartesian(...)
  arm.move_gohome(...)
  arm.move_circle(...)
  arm.emergency_stop()
  arm.set_position_aa(...)
  arm.set_servo_cartesian_aa(...)
  arm.vc_set_joint_velocity(...)
  arm.vc_set_cartesian_velocity(...)
  ```

- #### Set
  ```python
  arm.set_servo_attach(...)
  arm.set_servo_detach(...)
  arm.set_state(...)
  arm.set_mode(...)
  arm.motion_enable(...)
  arm.set_pause_time(...)
  ```

- #### Get
  ```python
  arm.get_version()
  arm.get_state()
  arm.get_is_moving()
  arm.get_cmdnum()
  arm.get_err_warn_code()
  arm.get_position(...)
  arm.get_servo_angle(...)
  arm.get_position_aa(...)
  arm.get_pose_offset(...)
  ```

- #### Setting
  ```python
  arm.set_tcp_offset(...)
  arm.set_tcp_jerk(...)
  arm.set_tcp_maxacc(...)
  arm.set_joint_jerk(...)
  arm.set_joint_maxacc(...)
  arm.set_tcp_load(...)
  arm.set_collision_sensitivity(...)
  arm.set_teach_sensitivity(...)
  arm.set_gravity_direction(...)
  arm.config_tgpio_reset_when_stop(...)
  arm.config_cgpio_reset_when_stop(...)
  arm.set_report_tau_or_i(...)
  arm.set_self_collision_detection(...)
  arm.set_collision_tool_model(...)
  arm.clean_conf()
  arm.save_conf()
  ```

- #### Gripper
  ```python
  arm.set_gripper_enable(...)
  arm.set_gripper_mode(...)
  arm.set_gripper_speed(...)
  arm.set_gripper_position(...)
  arm.get_gripper_position()
  arm.get_gripper_err_code()
  arm.clean_gripper_error()
  ```

- #### BIO Gripper

  ```python
  arm.set_bio_gripper_enable(...)
  arm.set_bio_gripper_speed(...)
  arm.open_bio_grippe(...)
  arm.close_bio_gripper(...)
  arm.get_bio_gripper_status()
  arm.get_bio_gripper_error()
  arm.clean_bio_gripper_error()
  ```

- #### RobotIQ Gripper

  ```python
  arm.robotiq_reset()
  arm.robotiq_set_activate(...)
  arm.robotiq_set_position(...)
  arm.robotiq_open(...)
  arm.robotiq_close(...)
  arm.robotiq_get_status(...)
  ```

- #### Modbus of the end tools

  ```python
  arm.set_tgpio_modbus_timeout(...)
  arm.set_tgpio_modbus_baudrate(...)
  arm.get_tgpio_modbus_baudrate(...)
  arm.getset_tgpio_modbus_data(...)
  ```

- #### GPIO

  ```python
  # Tool GPIO
  arm.get_tgpio_digital(...)
  arm.set_tgpio_digital(...)
  arm.get_tgpio_analog(...)
  arm.set_tgpio_digital_with_xyz(...)
  # Controller GPIO
  arm.get_cgpio_digital(...)
  arm.get_cgpio_analog(...)
  arm.set_cgpio_digital(...)
  arm.set_cgpio_analog(...)
  arm.set_cgpio_digital_input_function(...)
  arm.set_cgpio_digital_output_function(...)
  arm.get_cgpio_state()
  arm.set_cgpio_digital_with_xyz(...)
  arm.set_cgpio_analog_with_xyz(...)
  ```

- #### Linear Track
  
  ```python
  arm.get_linear_track_pos()
  arm.get_linear_track_status()
  arm.get_linear_track_error()
  arm.get_linear_track_is_enabled()
  arm.get_linear_track_on_zero()
  arm.get_linear_track_sci()
  arm.get_linear_track_sco()

  arm.clean_linear_track_error(...)
  arm.set_linear_track_enable(...)
  arm.set_linear_track_speed(...)
  arm.set_linear_track_back_origin(...)
  arm.set_linear_track_pos(...)
  arm.set_linear_track_stop(...)
  ```

- #### FT Sensor
  ```python
  arm.set_impedance(...)
  arm.set_impedance_mbk(...)
  arm.set_impedance_config(...)
  arm.config_force_control(...)
  arm.set_force_control_pid(...)
  arm.ft_sensor_set_zero(...)
  arm.ft_sensor_iden_load(...)
  arm.ft_sensor_cali_load(...)
  arm.ft_sensor_enable(...)
  arm.ft_sensor_app_set(...)
  arm.ft_sensor_app_get(...)
  arm.get_ft_sensor_data(...)
  arm.get_ft_senfor_config(...)
  arm.get_ft_sensor_error(...)
  ```

- #### Other
  ```python
  arm.set_pause_time(...)
  arm.system_control(...)
  arm.clean_error()
  arm.clean_warn()
  arm.set_counter_reset()
  arm.set_counter_increase(...)
  ```

- #### Register/Release
  ```python
  arm.register_report_callback(...)
  arm.register_report_location_callback(...)
  arm.register_connect_changed_callback(callback)
  arm.register_state_changed_callback(callback)
  arm.register_mode_changed_callback(callback)
  arm.register_mtable_mtbrake_changed_callback(callback)
  arm.register_error_warn_changed_callback(callback)
  arm.register_cmdnum_changed_callback(callback)
  arm.register_temperature_changed_callback(callback)
  arm.register_count_changed_callback(callback)
  arm.release_report_callback(callback)
  arm.release_report_location_callback(callback)
  arm.release_connect_changed_callback(callback)
  arm.release_state_changed_callback(callback)
  arm.release_mode_changed_callback(callback)
  arm.release_mtable_mtbrake_changed_callback(callback)
  arm.release_error_warn_changed_callback(callback)
  arm.release_cmdnum_changed_callback(callback)
  arm.release_temperature_changed_callback(callback)
  arm.release_count_changed_callback(callback)
  ```

- #### Property
  ```python
  arm.connected
  arm.default_is_radian
  arm.version
  arm.position
  arm.last_used_position
  arm.tcp_speed_limit
  arm.tcp_acc_limit
  arm.last_used_tcp_speed
  arm.last_used_tcp_acc
  arm.angles
  arm.joint_speed_limit
  arm.joint_acc_limit
  arm.last_used_angles
  arm.last_used_joint_speed
  arm.last_used_joint_acc
  arm.tcp_offset
  arm.state
  arm.mode
  arm.joints_torque
  arm.tcp_load
  arm.collision_sensitivity
  arm.teach_sensitivity
  arm.motor_brake_states
  arm.motor_enable_states
  arm.has_err_warn
  arm.has_error
  arm.has_warn
  arm.error_code
  arm.warn_code
  arm.cmd_num
  arm.device_type
  arm.axis
  arm.gravity_direction
  arm.gpio_reset_config
  arm.count
  arm.temperatures
  arm.voltages
  arm.currents
  arm.cgpio_states
  ```

