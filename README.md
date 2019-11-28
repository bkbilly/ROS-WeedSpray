# ROS Assignment

## Requerements

```bash
sudo apt install ros-kinetic-uol-cmp3103m ros-kinetic-uol-cmp9767m-base ros-kinetic-uol-cmp9767m-tutorial ros-kinetic-rqt-image-view
```
## How to run

```bash
roslaunch uol_cmp9767m_base thorvald-sim.launch map_server:=true
roslaunch uol_cmp9767m_tutorial move_base.launch
rqt_image_view
python ros_readimg.py
```

## Neural network Train
Using the annotation tool http://www.robots.ox.ac.uk/~vgg/software/via/
```bash
git clone https://github.com/matterport/Mask_RCNN.git
pip install mrcnn
python3 Mask_RCNN/samples/balloon/balloon.py train --dataset=plants/ --weights=coco
```

https://www.learnopencv.com/histogram-of-oriented-gradients/
https://towardsdatascience.com/image-classification-in-10-minutes-with-mnist-dataset-54c35b77a38d
https://www.analyticsvidhya.com/blog/2019/01/build-image-classification-model-10-minutes/
