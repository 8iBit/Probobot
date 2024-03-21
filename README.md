# Probobot!!!!!
### this project use ros2 distro is humble.
Getting started
1.Clone the repo:
```git clone https://github.com/8iBit/Probobot```

2.Navigate to your workspace:
```cd ~/Probobot```

3.Update dependencies:
```rosdep update```

4.Install dependencies:
```rosdep install --ignore-src --from-paths src -y -r```

5.Compile
```colcon build```

#Operating Instructions
After you build, remember to source the proper install folder...
```source ~/Probobot/install/local_setup.bash```

And then run the launch file...
```ros2 launch probobot gazebo.launch.py```

After that run the this file to control robot
```ros2 run probobot botcontrol.py``` 
