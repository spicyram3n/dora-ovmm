import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'hsrb_moveit/hsrb_moveit_config/launch'))
from robot_description import add_joints_and_links


class PlanningModelTest(unittest.TestCase):
    def test_zero_travel_sensor_joint_is_not_an_ik_degree_of_freedom(self):
        root = add_joints_and_links('''<robot name="hsrc">
          <joint name="wrist_ft_sensor_frame_joint" type="revolute">
            <origin xyz="0 0 0.1" rpy="3.141592653589793 0 0"/>
            <parent link="mount"/><child link="sensor"/>
            <limit lower="0" upper="0" effort="100" velocity="1.5"/>
          </joint>
          <joint name="arm_lift_joint" type="prismatic">
            <limit lower="0" upper="0.69" effort="300" velocity="0.2"/>
          </joint>
        </robot>''')
        sensor = root.find("joint[@name='wrist_ft_sensor_frame_joint']")
        self.assertEqual(sensor.get('type'), 'fixed')
        self.assertEqual(sensor.find('origin').get('xyz'), '0 0 0.1')
        self.assertEqual(root.find("joint[@name='arm_lift_joint']").get('type'), 'prismatic')

if __name__ == '__main__':
    unittest.main()
