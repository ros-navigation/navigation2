# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from geometry_msgs.msg import Quaternion, Transform
import numpy as np
import tf_transformations


"""
Transformation utilities for the loopback simulator
"""


def addYawToQuat(quaternion, yaw_to_add):
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    _, _, yaw = tf_transformations.euler_from_quaternion(q)
    new_yaw = yaw + yaw_to_add
    new_quaternion_array = tf_transformations.quaternion_from_euler(0.0, 0.0, new_yaw)
    new_quaternion = Quaternion()
    new_quaternion.x = new_quaternion_array[0]
    new_quaternion.y = new_quaternion_array[1]
    new_quaternion.z = new_quaternion_array[2]
    new_quaternion.w = new_quaternion_array[3]
    return new_quaternion


def transformStampedToMatrix(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    matrix = np.eye(4)
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z
    rotation_matrix = tf_transformations.quaternion_matrix([
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w
    ])
    matrix[:3, :3] = rotation_matrix[:3, :3]
    return matrix


def matrixToTransform(matrix):
    transform = Transform()
    transform.translation.x = matrix[0, 3]
    transform.translation.y = matrix[1, 3]
    transform.translation.z = matrix[2, 3]
    quaternion = tf_transformations.quaternion_from_matrix(matrix)
    transform.rotation.x = quaternion[0]
    transform.rotation.y = quaternion[1]
    transform.rotation.z = quaternion[2]
    transform.rotation.w = quaternion[3]
    return transform
