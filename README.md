# ROS Assignment
The objective of the project is to have a simulated robot called Throvald to run across the field of crops and find the bad weeds so that it can spray them. With image processing extracts the middle point of the recognized weed and all of the points are published as a pointCloud. Another node gets these points and desides if it needs to spray and publishes all the collected points so that it will be visualized on Rviz.

## Image Processing
In order to collect the bad weeds from the images the background had to be removed and then find the weeds. The implementation works with color matching through HSV images, though I also tried with [convolutional neural network (CNN)](mask_rcnn/README.md).

## Requerements
The project requires some dependencies that can be found on the LCAS Repository which can be installed like this:
```bash
curl https://raw.githubusercontent.com/LCAS/rosdistro/master/lcas-rosdistro-setup.sh | bash -
sudo apt install ros-kinetic-uol-cmp3103m ros-kinetic-uol-cmp9767m-base ros-kinetic-uol-cmp9767m-tutorial ros-kinetic-rqt-image-view
```

## How to run
To run the program there are some requirements. First of all the gazebo simulation must be running with the move_base. It can run everything with this command: ```roslaunch ROS-WeedSpray rosweedspray.launch```


## Articles that help
 * https://www.learnopencv.com/histogram-of-oriented-gradients/
 * https://towardsdatascience.com/image-classification-in-10-minutes-with-mnist-dataset-54c35b77a38d
 * https://www.analyticsvidhya.com/blog/2019/01/build-image-classification-model-10-minutes/
