#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from differential_kinematics.srv import TargetPose, TargetPoseRequest, TargetPoseResponse
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

        # IKサービスとの接続準備
        rospy.loginfo("Waiting for /dragon/end_effector_ik service...")
        rospy.wait_for_service('/dragon/end_effector_ik')
        self.ik_client = rospy.ServiceProxy('/dragon/end_effector_ik', TargetPose)
        rospy.loginfo("Connected to /dragon/end_effector_ik service")

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

            # ====== ここからIKサービス呼び出し部分を追加 ======
            # ①位置をそのままコピー
            tx = ef_pose_in_world.pose.position.x
            ty = ef_pose_in_world.pose.position.y
            tz = ef_pose_in_world.pose.position.z

            # ②姿勢：クオータニオン　→ roll,pitch,yaw へ変換
            q = ef_pose_in_world.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = euler_from_quaternion(quat)

            # ③サービスリクエストを作成
            req = TargetPoseRequest()
            req.target_pos.x = tx
            req.target_pos.y = ty
            req.target_pos.z = tz

            req.target_rot.x = roll
            req.target_rot.y = pitch
            req.target_rot.z = yaw

            req.orientation = True
            req.full_body = True
            req.collision_avoidance = False
            req.tran_free_axis = ''
            req.rot_free_axis = '' 
            req.debug = False

            # ④サービスを実行
            res = self.ik_client(req)
            rospy.loginfo("EndEffectorIK status: %s", res.status)


        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    transformer = PoseTransformer()
    rospy.spin()
