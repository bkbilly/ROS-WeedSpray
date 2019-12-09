# ROS Assignment
The objective of the project is to have a simulated robot called Throvald to run across the field of crops and find the bad weeds so that it can spray them. With image processing extracts the middle point of the recognized weed and all of the points are published as a pointCloud.

## Requerements
The project requires some dependencies that can be found on the LCAS Repository which can be installed like this:
```bash
curl https://raw.githubusercontent.com/LCAS/rosdistro/master/lcas-rosdistro-setup.sh | bash -
sudo apt install ros-kinetic-uol-cmp3103m ros-kinetic-uol-cmp9767m-base ros-kinetic-uol-cmp9767m-tutorial ros-kinetic-rqt-image-view
```
## How to run
To run the program there are some requirements. First of all the gazebo simulation must be running with the move_base. Follow bellow commands:
```bash
roslaunch uol_cmp9767m_base thorvald-sim.launch map_server:=true
roslaunch uol_cmp9767m_tutorial move_base.launch
rqt_image_view
python ros_readimg.py
```

## Neural network Train
One option to locate the weeds is by training a model, but this was very time consuming because of the annotation that had to be done. I used [this](http://www.robots.ox.ac.uk/~vgg/software/via/) annotation tool though there are many more. For the training I used a tool called [mrcnn](https://github.com/matterport/Mask_RCNN.git) and followed their tutorial about baloon training.

The training took more than 35 hours. The final result wasn't as satisfying as it should and it takes more than 3 seconds to get a result for each picture, so it wasn't usable for a real time application like we wanted.


## Articles that help
 * https://www.learnopencv.com/histogram-of-oriented-gradients/
 * https://towardsdatascience.com/image-classification-in-10-minutes-with-mnist-dataset-54c35b77a38d
 * https://www.analyticsvidhya.com/blog/2019/01/build-image-classification-model-10-minutes/
