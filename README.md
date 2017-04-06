# GARBAGE COLLECTOR ROBOT

The earth is all creatures’ home that should be clean for the residents. However we are polluting our home irresponsibly. As a result, we all are against with garbage pollution. Therefore, we thought a solution for this problem; using garbage collector robots. Robots are getting more useful and importance in real world. All areas, fields and industries has own robot to profit time, money and yields. There are too many projects that use robotic technology to solve problems. In this project, we designed a robot to collect garbage around it by using automatic grasping from a single-view. Grasping the objects and control the arm of robots are really significant step for collecting the garbage. Thus, the pose and shape of the objects are important to recognize. However, for test section of project, just stick is used for collection. The outline proposed work is as follows: introduction section introduces what is our problem and explained why it is important. In background section introduces what kinds of solutions exist for this problem and how to implement our project and youBot to ROS environment. In design & implementation parts explain how we designed and implement our algorithm for our problem. Evaluation part shows simulation test results and our achievements. With conclusion, all project will explained with last sentences.

## INTRODUCTION
Trash on our planet appeared almost simultaneously with the man, but until recently, its existence did not attach much importance. Garbage is something we are willing to pay to get rid of. If we were not willing to pay to get rid of it then it would be a commodity with value. In the middle ages, the garbage was the biggest problem of large cities, but it was not so much as now. In fact, at the future that may be much more serious problem for the world. Because the world is more crowded than the past and the population is increasing. Also, the rate of personal contamination is higher than past. If we do not give weight to this problem, at the future, personal polluting may increase too.
We can comprehend how the world is getting dirty by looking the table at figure-1. The total solid waste generation and per capita generation between 1960 and 2012 are indicated at the table. The total solid waste generation for the world is about 88 million tons. That value increases until 2005. The maximum amount of waste were generated at 2005, about 253 million tons. Although it decreases from 2005 to 2012, it is not an acceptable value, about 251 million tons. If we look at solid waste generation for per capita in a day, we see another factor why the earth is getting dirty. A person who lived in 1960 would generate 2.68 lbs. garbage in a day where 1 lbs. equals to 0.45 kg. This amount is 4.38 lbs. for who would live in 2012. These values proves that if we do not deal with pollution and go on messing the earth, this will be more than a problem for us.
The world has own magnificent weapon for the pollution, the ground. It can exterminate most kind of solid waste with time. But the time is different for the type of solid waste relatively. To illustrate, a paper is recycled in 3 months by the ground. However, this process takes 4000 years for a glass bottle. Further examples are shown in figure-3. Actually the time for recycling is related with the molecular structure for the solid waste. The organic waste can be recycled easier than chemicals. There is a fact that most of the solid waste for the 21th century earth is chemical based. Thus, the extermination ability of the ground is inadequate for cleaning the world by itself. This means that we should not leave the world alone against the solid pollution that is caused by us.
Actually, many people are employed as dustman for collecting garbage. Moreover, some of us try to keep clean in front of their house, garden or street. Also, there are many recycle bin for collecting garbage that can be recycled. However, we all pollute the earth more than cleaning. As a consequence, that pollution threatens to sociologic and biologic health of the people seriously. Recent physiological research has shown that the presence of garbage in the streets, parks and squares untidy can be bad for mental health and lead to depression. Thus, we need more effective solutions for getting rid of the pollution. But how can we obstruct the garbage pollution? Should more dustmen be employed? Should they spend more and more time for cleaning? Or what?
Robotic is a new technology that is getting improved rapidly day by day. We benefits from robots in many areas, such as industry, medicine and military because, they have many advantages. They are never tired, never say “NO!”, do not have salary except their charge and repair outcome, and can work more than a human. Thus, their usage area spreads rapidly.
We can also use robots for collecting garbage from the environment. In order to do this, we choose most suitable designed robot and program it for making it a garbage collector. Garbage is typically comprised of two important aspects: the material it is made from and the shape the material is in. For example, a plastic bottle is made from PET plastic and is in the shape of a bottle. Thus, the robot we choose should be able to pick the garbage up. Also, it should be able to scan the environment in for detecting garbage, to move through to the garbage, and carry it to a container.

## BACKGROUND
To get our achievement which is collect to garbage from streets, we used KUKA Youbot and its hardware. We provided our algorithm to this robot and collect the achievement results. Almost all of our mile stones to come up with this idea, are done with good results. In the section to follow, we will introduce KUKA youbot, ROS and installation packet.
### KUKA YouBot
In this section, we will introduce KUKA youBot and explain why we use this robot.
KUKA youBot user manual explains that “The KUKA youBot is a mobile manipulator that was primarily developed for education and research in mobile manipulation. KUKA youBot (shown in Figure 2) comes with fully open interfaces and allows the developers to access the system on nearly all levels of hardware control. It further comes with an application programming interface (KUKA youBot API), with interfaces and wrappers for recent robotic frameworks, with an open source simulation in Gazebo[1].
As same user manual YouBot has
• An arm with 5 degrees of freedom (0.5 kg payload)
• 2 finger gripper that is attached to the arm
• Omni-directional mobile platform with 4 omni-wheels (20 kg payload)

Arm is suitable to collect garbage around the streets because it moves 3 dimensional direction to get something. Therefore, garbage can be grasped easily by this. But there is a problem that grippers are smaller to hold even coke can. So, in project we implement garbage with stick which is suitable for youbot gripper finger’s gap. That means 2 fingers gripper is appropriate for special garbage. Omni-directional mobile platform with 4 omni-wheels help us to find garbage easily around the robot and if robot will stuck somewhere, this feature will be nice to get rid of there. Also, at the same location, it can turn around itself, so take a profit for place.

### ROS (Robot Operating System)
ROS ( Robot Operating System) is an open source software framework for robotic development. ROS is to provide a common platform to make the construction of capable robotic applications quicker and easier. Some of the features it provides include hardware abstraction, device drivers, message-passing, and package management [2].
There are three ways that nodes may communicate in a ROS environment:
1. By publishing messages to a topic.
2. By listening to messages published on a topic.
3. By calling a service provided by another node.

### Installation Software on Your Linux
Almost 2 weeks are spent to install youbot with all drivers to our Ubuntu version. Firstly to implement youbot to environment, you have to install ROS as explained above why it is important. We used fuerte environment to implement KUKA youBot. Via this webmail to install ROS
- http://wiki.ros.org/fuerte/Installation/Ubuntu

After ROS installment, we have to take as clone github for packet and drivers. Firstly, one document name is ros_stacks should be created, and this documents should download and unzip to this file.
- https://github.com/youbot/youbot-ros-pkg/tree/fuerte
- https://github.com/ipa320/cob_common/tree/fuerte

After this step, ros-fuerte-pr2-controllers should be installed for ROS. Therefore, in command line next command will be send.
- $ sudo apt-get install ros-electric-pr2-controllers # required for trajectory_msgs
- cd ~/ros_stacks

Now, all drivers should be made after this steps. Firstly, we have to show to ROS, ros_stacks is our ROS_PATH. Then, Youbot_Driver, Youbot_Description and our project files should be made by command as “rosmake ….” .
After all steps, our project will implement on ROS and we can run and analyze on Gazebo. [3]

### Literature Background

- (Saxena et al., 2008) developed a learning algorithm that predicts the grasp position of
an object directly as a function of its image. Their algorithm focuses on the task grasping
points that are trained with labelled synthetic images of a different number of objects.
- (Kragic & Bjorkman, 2006) developed a vision-guided grasping system. Their
approach was based on integrated monocular and binocular cues from five cameras to provide
robust 3D object information.
- (Wang & Jiang, 2005) developed a framework of automatic grasping of unknown
objects by using a laser-range scanner and simulation environment.
- (Richtsfeld & Zillich, 2008) published a method to calculate possible grasping points
for unknown objects with the help pf the flat top surfaces of the objects based on a laser-range
scanner system. However, there exist different approaches for grasping for grasping quasi
planar objects.
- (Goldfeder et al., 2007) presented a grasp planner which considers the full range of
parameters of a real hand and arbitrary object, including physical and material properties as
well as environmental obstacles and forces.


Continue with our project simulation environment installation and results, please read [project report](https://github.com/bahadirbal/GarbageCollectorRobot/blob/master/Project-Report.pdf).
