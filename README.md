
## Overview of node functionality

This ROS node implements the [FastSLAM2.0](http://robots.stanford.edu/papers/Thrun03g.pdf) algorithm. FastSLAM2.0 solves the Simultaneous Localization And Mapping Problem ([SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)).   

In short, the node processes sensor measurements and publishes an estimation of the cars environment (coordinates of cones) and position.

### Prerequisites 

To properly understand the theory behind FastSLAM2.0 we recommend the reader to familiarize themselves with Kalman Filters.

### Relevant litterature

Our reference when writing the code was the following well detailed [paper](http://robots.stanford.edu/papers/Thrun03g.pdf). It is 48 pages long, however for those not interested in experimental results, reading the first 25 pages is definetly sufficient. The paper was written by the authors of the algorithm and assumes no prior knowledge of the field. Note that page 22 is a life saver, it contains pseudo code for the entire algorithm (although our code does not follow this to the tee).

For those who prefer learning from videos, Cyrill Stachniss has a great [playlist](https://www.youtube.com/watch?v=U6vr3iNrwRA&list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) on youtube. This playlist consists of lectures from a robotics course that revolves around the SLAM problem. Hence, the theory presented is broader than in the article above. The FastSLAM2.0 theory is less detailed however.
