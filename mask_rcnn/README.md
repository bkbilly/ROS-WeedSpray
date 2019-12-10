# Neural network Train
One option to locate the weeds is by training a model, but this was very time consuming because of the annotation that had to be done. I used [this](http://www.robots.ox.ac.uk/~vgg/software/via/) annotation tool though there are many more. For the training I used a tool called [mrcnn](https://github.com/matterport/Mask_RCNN.git) and followed their tutorial about baloon training.

The training took more than 35 hours. The final result wasn't as satisfying as it should and it takes more than 3 seconds to get a result for each picture, so it wasn't usable for a real time application like we wanted.
