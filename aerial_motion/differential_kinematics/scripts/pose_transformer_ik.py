#!/usr/bin/env python3

import rospy
from differential_kinematics.msg import TargetPose
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_geometry_msgs # required
import numpy as np

class PoseTransformer:
    def __init__(self):
        rospy.init_node('pose_transformer', anonymous=False)
        rospy.loginfo("PoseTransgormer node started successfully")

        self.handover_dis_x = rospy.get_param('~x', 0.1)
        self.handover_dis_y = rospy.get_param('~y', 0.0)
        self.handover_dis_z = rospy.get_param('~z', 0.1)
        self.handover_roll = rospy.get_param('~roll', 0.0)
        self.handover_pitch = rospy.get_param('~pitch', 0.1)
        self.handover_yaw = rospy.get_param('~yaw', np.pi)
        

        self.hand_pose_sub = rospy.Subscriber('/desired_3D_pose', PoseStamped, self.hand_pose_cb)

        self.ef_pose_pub = rospy.Publisher('/ef_pose', PoseStamped, queue_size=10)

        self.world2hand_tf_bc = tf2_ros.TransformBroadcaster()
        self.world2hand_tf = TransformStamped()
        self.world2hand_tf.header.frame_id = "world"
        self.world2hand_tf.child_frame_id = "hand_frame"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ef_pose_in_hand = PoseStamped()
        self.ef_pose_in_hand.header.frame_id = "hand_frame"
        self.ef_pose_in_hand.pose.position.x = self.handover_dis_x
        self.ef_pose_in_hand.pose.position.y = self.handover_dis_y
        self.ef_pose_in_hand.pose.position.z = self.handover_dis_z

        #このクオータニオンで手に対するエンドエフェクタの姿勢を設定
        q = quaternion_from_euler(self.handover_roll, self.handover_pitch, self.handover_yaw)

        self.ef_pose_in_hand.pose.orientation.x = q[0]
        self.ef_pose_in_hand.pose.orientation.y = q[1]
        self.ef_pose_in_hand.pose.orientation.z = q[2]
        self.ef_pose_in_hand.pose.orientation.w = q[3]

        # IKのトピックとの接続準備
        self.ik_pub = rospy.Publisher('/dragon/end_effector_ik', TargetPose, queue_size=1)

    def hand_pose_cb(self,msg):
        # 毎回最新のパラメータを反映する
        self.handover_dis_x = rospy.get_param('~x', self.handover_dis_x)
        self.handover_dis_y = rospy.get_param('~y', self.handover_dis_y)
        self.handover_dis_z = rospy.get_param('~z', self.handover_dis_z)
        self.handover_roll = rospy.get_param('~roll', self.handover_roll)
        self.handover_pitch = rospy.get_param('~pitch', self.handover_pitch)
        self.handover_yaw = rospy.get_param('~yaw', self.handover_yaw)

        # ここで、ef_pose_in_handを更新する
        self.ef_pose_in_hand.pose.position.x = self.handover_dis_x
        self.ef_pose_in_hand.pose.position.y = self.handover_dis_y
        self.ef_pose_in_hand.pose.position.z = self.handover_dis_z

        q = quaternion_from_euler(self.handover_roll, self.handover_pitch, self.handover_yaw)
        self.ef_pose_in_hand.pose.orientation.x = q[0]
        self.ef_pose_in_hand.pose.orientation.y = q[1]
        self.ef_pose_in_hand.pose.orientation.z = q[2]
        self.ef_pose_in_hand.pose.orientation.w = q[3]

        # 手のTF更新
        self.world2hand_tf.header.stamp = rospy.Time.now()
        self.world2hand_tf.transform.translation.x = msg.pose.position.x
        self.world2hand_tf.transform.translation.y = msg.pose.position.y
        self.world2hand_tf.transform.translation.z = msg.pose.position.z
        self.world2hand_tf.transform.rotation = msg.pose.orientation
        self.world2hand_tf_bc.sendTransform(self.world2hand_tf)

        self.transform_pose()

    def transform_pose(self):
        self.ef_pose_in_hand.header.stamp =self.world2hand_tf.header.stamp #Use Teh same timestamp as the hand frame)
        
        try:
            ef_pose_in_world = self.tf_buffer.transform(self.ef_pose_in_hand, "world",rospy.Duration(0.5)) #waiting for TF timeout of 0.5 seonds
            self.ef_pose_pub.publish(ef_pose_in_world)

            # ====== ここからIK topic部分を追加 ======
            # ①位置をそのままコピー
            tx = ef_pose_in_world.pose.position.x
            ty = ef_pose_in_world.pose.position.y
            tz = ef_pose_in_world.pose.position.z

            # ②姿勢：クオータニオン　→ roll,pitch,yaw へ変換
            q = ef_pose_in_world.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = euler_from_quaternion(quat)

            # ③メッセージを作成
            msg = TargetPose()
            msg.target_pos.x = tx
            msg.target_pos.y = ty
            msg.target_pos.z = tz

            msg.target_rot.x = roll
            msg.target_rot.y = pitch
            msg.target_rot.z = yaw

            msg.orientation = True
            msg.full_body = True
            msg.collision_avoidance = False
            msg.tran_free_axis = ''
            msg.rot_free_axis = '' 
            msg.debug = False

            # ④メッセージのpublish
            self.ik_pub.publish(msg)


        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    transformer = PoseTransformer()
    rospy.spin()
