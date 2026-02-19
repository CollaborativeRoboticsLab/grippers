ubuntu@5fa75cbd9fd5:~/colcon_ws$ ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close: true, torque: 0.0, use_torque_mode: false}"
Waiting for an action server to become available...
Sending goal:
     close: true
torque: 0.0
use_torque_mode: false

Goal accepted with ID: fef5aa5d8dc54c9581c631b8079c668a

Result:
    success: false
message: 'Close failed: disable_torque communication failed: [TxRxResult] There is no status packet!'

Goal finished with status: ABORTED
ubuntu@5fa75cbd9fd5:~/colcon_ws$ ros2 action send_goal /close_gripper gripper_msgs/action/CloseGripper "{close: true, torque: 0.0, use_torque_mode: false}"
Waiting for an action server to become available...
Sending goal:
     close: true
torque: 0.0
use_torque_mode: false

Goal accepted with ID: acd01d9d97424632a92ea9c69087edb8

Result:
    success: true
message: Close command sent.

Goal finished with status: SUCCEEDED
ubuntu@5fa75cbd9fd5:~/colcon_ws$ ros2 action send_goal /open_gripper gripper_msgs/action/OpenGripper "{torque: 0.0, use_torque_mode: false}"
Waiting for an action server to become available...
Sending goal:
     torque: 0.0
use_torque_mode: false

Goal accepted with ID: ff2f8cac2cab4047a1c18c5a3065328f

Result:
    success: false
message: 'Open failed: disable_torque returned error: [RxPacketError] CRC doesn''t match!'

Goal finished with status: ABORTED
ubuntu@5fa75cbd9fd5:~/colcon_ws$ ros2 action send_goal /open_gripper gripper_msgs/action/OpenGripper "{torque: 0.0, use_torque_mode: false}"
Waiting for an action server to become available...
Sending goal:
     torque: 0.0
use_torque_mode: false

Goal accepted with ID: a437279029814002b2d140c721dc37e6

Result:
    success: true
message: Open command sent.
