# movo_ws

This git project is a catkin workspace.  


In this directory use:
```
catkin make 

source ~\.bashrc 
```

To launch a simulated movo:
``roslaunch lis_movo_pkg sim.launch``


To launch move group:
`` roslaunch lis_movo_pkg moveit.launch``


To untuck the arms:
``rosrun lis_movo_pkg tuckarms.py -u``


To tuck the arms:
``rosrun lis_movo_pkg tuckarms.py -t``


