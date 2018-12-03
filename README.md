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
```<node name="rviz" pkg="rviz" type="rviz" required="true" />``` by
```<node name="rviz" pkg="rviz" type="rviz" args="-d '$(find move_printer)/config/rviz.rviz'" required="true" />```

#### At that point, you should have RVIZ running with the printer model loaded correctly with a wrong orientation

![RVIZ running with config](screenshots/rviz_config.png)
