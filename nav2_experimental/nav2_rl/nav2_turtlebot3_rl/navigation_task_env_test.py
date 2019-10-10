# Copyright (c) 2019 Intel Corporation
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

from navigation_task_env import NavigationTaskEnv
import rclpy

class Test(NavigationTaskEnv):
    '''
    This is a simple test to check if the navigation task is
    getting the path using the ComputePathToPose action.
    '''
    def __init__(self):
        super().__init__()

    def get_actions(self):
        pass

    def main(self, args=None):
        '''
        Call reset and check if new path is received
        '''
        self.reset()
        for i in (self.result_path):
            print(str(i.pose.position.x) + " " + str(i.pose.position.y))

if __name__=="__main__":
    rclpy.init()
    test = Test()
    test.main()
