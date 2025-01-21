# Flying Elephant Trunk ("会飞的象鼻")
![image](https://github.com/arclab-hku/AET/blob/master/code_availability/cover_fig.jpg)

# Publication:

R. Peng, Y. Wang, M. Lu, and P. Lu, “A dexterous and compliant aerial continuum manipulator for cluttered and constrained environments,” Nature Communications, vol. 16, no. 1, p. 889, Jan. 2025, doi: 10.1038/s41467-024-55157-2.

**Bilibili:**   https://space.bilibili.com/1778365431


**Youtube:**    https://www.youtube.com/@arclab7795

 
# 1. Prerequisites

Ubuntu >= 18.04 

ROS >= Melodic

 
# 2. Build
Copy the folder "code_availability/kinematics_model" into your ros workspace.

$ `catkin_make`

$ `source devel/setup.bash`

$ `roslaunch kinematics_model visualize.launch`

$ `rosrun kinematics_model model_visualization`


# 3. Demo

![image](https://github.com/arclab-hku/AET/blob/master/code_availability/code_demo.gif)

![image](https://github.com/arclab-hku/AET/blob/master/code_availability/flying_AET.gif)

# 4. License
The source code of this ROS package is released under [GPLv2](https://www.gnu.org/licenses/) license. We only allow it free for academic usage with several patents. 
For commercial use or cooperation, please contact Dr. Peng Lu lupeng@hku.hk.

For any technical issues, please contact me via email pengrui-rio@connect.hku.hk.


# 5. Citation
   
    @Article{Peng2025,
    author={Rui Peng, Yu Wang, Minghao Lu and Peng Lu},
    title={A dexterous and compliant aerial continuum manipulator for cluttered and constrained environments},
    journal={Nature Communications},
    year={2025},
    month={Jan},
    day={21},
    volume={16},
    number={1},
    pages={889},
    abstract={Aerial manipulators can manipulate objects while flying, allowing them to perform tasks in dangerous or inaccessible areas. Advanced aerial manipulation systems are often based on rigid-link mechanisms, but the balance between dexterity and payload capacity limits their broader application. Combining unmanned aerial vehicles with continuum manipulators emerges as a solution to this trade-off, but these systems face challenges with large actuation systems and unstable control. To address these challenges, we propose Aerial Elephant Trunk, an aerial continuum manipulator inspired by the elephant trunk, featuring a small-scale quadrotor and a dexterous, compliant tendon-driven continuum arm for versatile operation in both indoor and outdoor settings. We develop state estimation for the quadrotor using an Extended Kalman Filter, shape estimation for the continuum arm based on piecewise constant curvature, and whole-body motion planning using minimum jerk principles. Through comprehensive fundamental verifications, we demonstrate that our system can adapt to various constrained environments, such as navigating through narrow holes, tubes, or crevices, and can handle a range of objects, including slender, deformable, irregular, or heavy items. Our system can potentially be deployed in challenging conditions, such as pipeline maintenance or electricity line inspection at high altitudes.},
    issn={2041-1723},
    doi={10.1038/s41467-024-55157-2},
    url={https://doi.org/10.1038/s41467-024-55157-2}
    }
 
