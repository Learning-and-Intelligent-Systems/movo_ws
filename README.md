# movo_ws

This git project is a catkin workspace.  


In this directory use:
```
catkin make 

source ~/.bashrc 
source devel/setup.bash
```

To launch a simulated movo:
``roslaunch lis_movo_pkg sim.launch``


To launch move group:
`` roslaunch lis_movo_pkg moveit.launch``


With move group running, you can use the following commands to tuck and untuck the arms.


* To untuck the arms:
    ``rosrun lis_movo_pkg tuckarms.py -u``

* To tuck the arms:
    ``rosrun lis_movo_pkg tuckarms.py -t``

--------------

In ``src/lis_movo_pkg/scripts`` there are some example files for moving the head, torso, and base that work with the simulated robot.



