# Virtual-Vehicle
<p>  The project i.e. Virtual Vehicle which is a virtual self-driving car uses the Carla
simulator to test and develop algorithms and systems for autonomous driving.</p><br>
<p>CARLA (Car Learning to Act) is an open-source simulator designed for research
and development in autonomous driving. It provides a realistic virtual
environment for testing and training autonomous driving algorithms. Developed
by the Computer Vision Center (CVC) and the Universitat Autònoma de
Barcelona (UAB), CARLA aims to bridge the gap between simulation and
real-world autonomous driving.</p>
<p><b>In the first step (Attaching and Enabling the Camera Sensor)</b>, Carla has the environment (server) and then agents (clients).
With Carla, we get a car, an environment to drive that car in, and then we have a
bunch of sensors that we can place upon the car to emulate real-life self-driving
car sensors. Things like LIDAR, cameras, accelerometers, and so on.<br>
Here's the Video Demo Attached:<br>
<a href="https://drive.google.com/file/d/1c5AiB13mxToQQ0STBamHvlo0eqg77qOu/view?usp=drive_link">Create Client and World then attach camera to the vehicle</a></p>
<br>

<p><b>Second Step (Creating and Cleaning of List of Actors):</b>
To begin, there are several types of objects in Carla. First, you of course have the
"world." This is your environment. Then, you have the actors within this world.
Actors are things like your car, the sensors on your car, pedestrians, and so on.
Finally, we have blueprints. The blueprints are the attributes of our actors. The
most important thing we'll take care of immediately is the list of actors, and
cleaning them up when we're done.<br>
Here's the Video Demo Attached:<br>
<a href="https://drive.google.com/file/d/1rVo0o4OsNWRuVT9NugaPZTN7_VlfdwSa/view?usp=drive_link">Localization</a></p>
<br>

<p><b>Third Step (Lane Detection) :</b>
Next step is ‘Lane detection’ in Carla, lane detection can be done by a camera
sensor attached to the vehicle and capturing images of the road ahead. These
images can then be processed using computer vision algorithms to detect the lane
markings. Once the lane boundaries are detected, they can be used to provide
information to the vehicle's control system, such as lane departure warnings or
trajectory planning.<br>
Here's the Video Demo Attached:<br>
<a href="https://drive.google.com/file/d/1FTR65-TLN8plPfp_NJ_7mx7Q69Mb9BrB/view?usp=drive_link">Lane Detection</a></p>
<br>

<p><b>Fourth Step (Object Detection) :</b>
‘Object detection’ in Carla involves using computer vision algorithms to identify
and locate objects in images or point cloud data captured by sensors attached to a
vehicle in the simulation environment.
To perform object detection in Carla, a sensor such as a camera is attached to a
vehicle and captures data about the surrounding environment. This data is then
processed using an object detection algorithm, which identifies and locates objects
in the data, such as pedestrians, vehicles, and traffic signs.<br>
Here's the Video Demo Attached:<br>
<a href="https://drive.google.com/file/d/1IOZN77rVQ7B4s9Tmcdki1i48gmTvSrOV/view?usp=drive_link">Object Detection</a></p>
