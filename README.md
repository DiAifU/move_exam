# Correction de l'examen Move 2017-2018
#### CPE Lyon

## Setup
- Initialize a workspace
- Clone the project in src folder

## Create first URDF
- Add printer.urdf with only one link to load the ```printer.dae``` file
- Add ```rviz.launch``` to preview your URDF
- Add ```RobotModel``` and ```TF``` in RVIZ
- Set Fixed frame to base_link
- Save RVIZ config to ```move_printer/config/rviz.rviz```
- Modify ```rviz.launch``` to have the saved config to be loaded by default by replacing the line
```xml
<node name="rviz" pkg="rviz" type="rviz" required="true" />
by
<node name="rviz" pkg="rviz" type="rviz" args="-d \'$(find move_printer)/config/rviz.rviz\'" required="true" />
```

#### At that point, you should have RVIZ running with the printer model loaded correctly with a wrong orientation

![RVIZ running with config](https://github.com/DiAifU/move_exam/raw/master/screenshots/rviz_config.png)

## Modify URDF

We are now going to turn the printer in the right direction. Let's put pi/2 in roll. We now have that result:

![Fixed printer order](https://github.com/DiAifU/move_exam/raw/master/screenshots/printer_roll_modified.png)

### We now want to add the extruder

- First add a new link named extruder_link
- By default, the TF is at the center of the cylinder, we move the origin of the cylinder half of its size on z axis to have it at the bottom of the cylinder
- Since we're using XACRO to parse our URDF, we can define variables to make our file cleaner. To do that:
  - Add the xmlns definition for xacro :
  ```xml
  Replace
  <robot name="printer">
    by
  <robot name="printer" xmlns:xacro="http://www.ros.org/wiki/xacro">
```
  - Then define three variables right under of the ```<robot>``` tag :

    ```xml
    <xacro:property name="plate_height" value="0.025" />
    <xacro:property name="extruder_radius" value="0.01" />
    <xacro:property name="extruder_length" value="0.05" />
```
- Now let's move the origin of the extruder_link to half of its size on z axis.
```xml
Replace
<origin rpy="0 0 0" xyz="0 0 0"/>
by
<origin rpy="0 0 0" xyz="0 0 ${extruder_length/2}"/>
```

#### At this point, we should have the TF at the bottom of the cylinder. We now need to set up a joint to be able to display the extruder

- Let's first add a new ```plate_length``` variable :
```xml
<xacro:property name="plate_length" value="0.25" />
```
- Let's now add a first joint :
```xml
  <joint name="joint1" type="prismatic">
    <parent link="base_link"/>
    <child link="extruder_link"/>
    <origin rpy="0 0 0" xyz="0 0 0.35"/>
    <limit lower="${-plate_length/2}" upper="${plate_length/2}" effort="1000.0" velocity="0.1" />
  </joint>
```
We're not specifying it here but by default if the ```<axis>``` tag is not defined, it assumes that the axis for this joint is the x axis. So we make sure to set the lower and upper limit to half of the plate length
- Let's save the file and run rviz again. Here is what we have :
![Model with extruder](https://github.com/DiAifU/move_exam/raw/master/screenshots/extruder.png)
We can also move the extruder along the x axis:
![Model with extruder](https://github.com/DiAifU/move_exam/raw/master/screenshots/extruder_x.png)

### Here is the tricky part

In order to have the extruder to move on the 3 axis, we have to add two virtual prismatic joints along the real one :
- First create twos links ```extruder_link_virt1``` and ```extruder_link_virt2``` and then create the joints in this order :
```base_link --> extruder_link_virt1 --> extruder_link_virt2 --> extruder_link```
- Set the axis and limits of the links as follows :
![Fixed printer order](https://github.com/DiAifU/move_exam/raw/master/screenshots/urdf_extruder_complete.png)

### Now create the MoveIt! package for the robot
- First rename ```printer.urdf``` to ```printer.urdf.xacro``` in order to avoid parsing problems in MoveIt! (think about updating rviz.launch).
- Run ```roslaunch moveit_setup_assistant setup_assistant.launch```
- Load the urdf in the first window
- When loaded, go to the Self Collision Matrix tab and generate it automatically
  - Go to the Planning Group tab and create a group composed of all the joints and named ```printer_group```
  - In Robot Poses tab, add the ```init_pose``` and set all the angles to 0
  - In End Effectors tab, add the effector called ```extruder``` in group ```printer_group``` and set the parent_link to ```extruder_link```
  - Generate the package and save it in a folder named ```move_printer_moveit```

### You can now start testing MoveIt! by running the ```demo.launch``` in your created MoveIt! package.
- ```catkin_make``` your workspace
- Source it
- ```roslaunch move_printer_moveit demo.launch```
![MoveIt](https://github.com/DiAifU/move_exam/raw/master/screenshots/moveit.gif)

## Create move_printer_moveit node:

We first start by creating the python file (don't forget to ``` $ chmod +x <your_file.py> ```)
Now we have to create a set of waypoints to draw a circle with our effector using the cartesian path.
We know that the equations of a circle are:
 -  x = a + R cos(angle)
 -  y = b + R sin (angle)
 So we gonna add for each degree of the 360° of a circle a new wpose on the waypoints.
 ```python
 for angle in range(0,360):
     wpose.position.z = 0.2
     wpose.position.x = radius * cos(self.deg_to_rad(angle))
     wpose.position.y = radius * sin(self.deg_to_rad(angle))
     waypoints.append(copy.deepcopy(wpose))

  ```
Then we process our plan and execute it :
  ```python
  (plan, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
  self.group.plan()
  rospy.sleep(1)
  self.group.execute(plan,wait=True)
  ```
