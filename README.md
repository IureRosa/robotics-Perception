# Robotics: Perception

How can robots perceive the world and their own movements so that they accomplish navigation and manipulation tasks?  In this module, we will study how images and videos acquired by cameras mounted on robots are transformed into representations like features and optical flow.  Such 2D representations allow us then to extract 3D information about where the camera is and in which direction the robot moves.  You will come to understand how grasping objects is facilitated by the computation of 3D posing of objects and navigation can be accomplished by visual odometry and landmark-based localization.

## Geometry of Image Formation

A tutorial on the standard camera models used in computer vision. These models allow us to understand, in a geometric fashion, how light from a scene enters a camera and projects onto a 2D image. By defining these models mathematically, we will be able understand exactly how a point in 3D corresponds to a point in the image and how an image will change as we move a camera in a 3D environment. In the later modules, we will be able to use this information to perform complex perception tasks such as reconstructing 3D scenes from video.

## Projective Transformations

Now that we have a good camera model, we will explore the geometry of perspective projections in depth. We will find that this projection is the cause of the main challenge in perception, as we lose a dimension that we can no longer directly observe. In this module, we will learn about several properties of projective transformations in depth, such as vanishing points, which allow us to infer complex information beyond our basic camera model.

## Pose Estimation

In this module we will be learning about feature extraction and pose estimation from two images. We will learn how to find the most salient parts of an image and track them across multiple frames (i.e. in a video sequence). We will then learn how to use features to find the position of the camera with respect to another reference frame on a plane using Homographies. We will also learn about how to make these techniques more robust, using least squares to hand noisy feature points or RANSAC to remove completely erroneous feature points.

## Multi-View Geometry

Now we will use what we learned from two view geometry and extend it to sequences of images, such as a video. We will explain the fundamental geometric constraints between point features in images, the Epipolar constraint, and learn how to use it to extract the relative poses between multiple frames. We will finish by combining all this information together for the application of Structure from Motion, where we will compute the trajectory of a camera and a map throughout many frames and refine our estimates using Bundle adjustment.

