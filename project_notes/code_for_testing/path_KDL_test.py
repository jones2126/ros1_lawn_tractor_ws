
'''
In the cpp source code there are these lines:
//! Compute transform that transforms a pose into the robot frame (base_link)
KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose,
                               const geometry_msgs::Transform& tf);

So I translate this to be:
Input Parameters:
const geometry_msgs::Pose& pose: The pose you want to transform, typically 
representing the position and orientation of an object or point in space.
const geometry_msgs::Transform& tf: The transformation information that defines how 
to convert the pose into the base_link frame. This could include rotation and 
translation information to align the pose with the robot's coordinate system.

"path_segment" topic is the source of the pose data used in transformToBaseLink.  See
the 


Return Value:
KDL::Frame: The transformed pose, represented as a rigid body transformation 
(including rotation and translation) in the KDL format. This transformed pose is 
now expressed in the coordinate frame of the robot's base link (base_link).

Purpose:
Imagine you have a robot, and you want to know where its hand is pointing. You 
know where the hand is pointing right now (that's the "pose"), but then you move the 
robot's arm (that's the "transformation"), and you want to know where the hand is 
pointing after the movement.

Here's what you started with:

Pose: The hand was pointing at a spot 1 block to the right, 2 blocks up, and 3 blocks 
forward from where the robot was standing.
Transformation: You then moved the hand 2 blocks to the right, 3 blocks up, and 4 
blocks forward.

Here's how you can figure out where the hand ended up:

Right (X): It started 1 block to the right and moved 2 more blocks to the right, so 
it's now 3 blocks to the right.
Up (Y): It started 2 blocks up and moved 3 more blocks up, so it's now 5 blocks up.
Forward (Z): It started 3 blocks forward and moved 4 more blocks forward, so it's 
now 7 blocks forward.

So the final position of the hand is 3 blocks to the right, 5 blocks up, and 7 blocks 
forward from where the robot was standing.

The numbers in the output, [3,5,7][3,5,7], tell you exactly where the hand is pointing 
after the movement. The other numbers, [1,0,0;0,1,0;0,0,1][1,0,0;0,1,0;0,0,1], tell 
you that the hand didn't twist or turn while it was moving; it just went straight 
to the new spot. That part is a bit more complicated, but for this example, you can 
think of it as the hand staying level and pointing in the same direction as it moved.
'''

# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_KDL_test.py
import PyKDL as kdl
from geometry_msgs.msg import Pose, Transform

def transformToBaseLink(pose, tf):
    # Convert geometry_msgs::Pose to KDL::Frame
    poseFrame = kdl.Frame(
        kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        kdl.Vector(pose.position.x, pose.position.y, pose.position.z))

    # Convert geometry_msgs::Transform to KDL::Frame
    tfFrame = kdl.Frame(
        kdl.Rotation.Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w),
        kdl.Vector(tf.translation.x, tf.translation.y, tf.translation.z))

    # Apply the transformation
    return tfFrame * poseFrame

# Sample pose
pose = Pose()
pose.position.x = 1.0
pose.position.y = 2.0
pose.position.z = 3.0
pose.orientation.w = 1.0  # Identity quaternion

# Sample transform
tf = Transform()
tf.translation.x = 2.0
tf.translation.y = 3.0
tf.translation.z = 4.0
tf.rotation.w = 1.0  # Identity quaternion

# Call the function
transformedPose = transformToBaseLink(pose, tf)

# Print the result
print("Transformed Pose:", transformedPose)
