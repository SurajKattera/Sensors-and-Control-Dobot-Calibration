close all
clear all

%Subscribing to Intel RealSense RGB raw image rostopic
sub = rossubscriber('/camera/color/image_raw');

%Subscribing to joint states of dobot magician rostopic
jointStateSubscriber = rossubscriber('/dobot_magician/joint_states');
pause(2); 


%Dataset input
NumerOfDataSet = 21

%Create figure window
f = figure;

%Creationg of empty array to store data of pointclouds, images and joint
%angles
PointClouds = {};
Images = {};
Joints = zeros(1,4,NumerOfDataSet);

%Loop for data collection
%In each iteration of the loop an image is taken and the point cloud and 
%joint state of the Dobot is collected as well
%Requires keyboard input of the 'Enter' key after an image is taken
for x = 1:NumerOfDataSet
    ptc = rostopic("echo", '/camera/depth/color/points');
    img = readImage(sub.LatestMessage);
    imshow(img)
    f;
    pause(2);  
    
    scatter3(ptc)
    PointClouds{x} = ptc;
    Images{x} = img;
    test = jointStateSubscriber.LatestMessage.Position()
    Joints(:,:,x) = jointStateSubscriber.LatestMessage.Position();

    input("Dataset " + int2str(x) + " has been collected, press enter to continue");
end