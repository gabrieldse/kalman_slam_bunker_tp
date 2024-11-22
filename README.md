# Package POL_BUNKER en humble

Modele urdf du bunker : my_bunker.xacro

Principe : multi roue (2x8) pour 'simuler' les chenilles avec le plugin libgazebo_ros_diff_drive

Possible d'utiliser libSimpleTrackedVehiclePlugin.so ? sur la base de l'exemple : 
/usr/share/gazebo-11/worlds/tracked_vehicle_simple.world

contenu du modèle :
- my_bunker.xacro
- inertial.xacro (macros pour matrices d'inerties)
- macro-roue.xacro (pour placer les link-joint de chaque roues)
- macro-gps.xacro (pour placer gps-link, gps-joint et son plugin )
- macro-imu.xacro (pour placer imu-link, imu-joint etle plugin de l'imu)


Pour lancer la simu gazebo :
```
ros2 launch pol_bunker bunker-sim.launch.py
```

## HECTOR Plugins
Hector gazebo fournit pluieurs plugin pour beaucoup de versions de ROS-gazebo (cf branches) [ici](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo)

Récupérer le zip 
https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/archive/refs/heads/humble-devel.zip
et le décompresser dans src. Renommer le dossier "hector_gazebo"

Et compiler le package pour que le bunker puisse l'utiliser
```
colcon build --packages-select hector_gazebo_plugins
```
