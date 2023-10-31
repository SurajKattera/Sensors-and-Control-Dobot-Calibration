
clear;
close all;
clc;

%%%%%%%%%%%%%%%%%%%% Relative Transform calculation of End Effector

%Initialising the dobot magician robot

dobot = DobotMagician();

%Recorded joint angles for each picture of checkerboard taken
qValues = [[-0.0360, 0.1002, -0.0774, 0];...
    [-0.3123, 0.0989, -0.0462, 0];...
    [-0.1723, 0.2794 , -0.0057, 0];...
    [0.1312, 0.2808,  0.0275, 0];...
    [-0.2227, 0.1950, 0.1651, 0];...
    [0.4112, 0.1107, 0.1553, 0];...
    [0.0023, 0.2648, 0.3688, 0];...
    [-0.0859, 0.2774, 0.3838, 0];...
    [-0.0485, 0.2516, 0.4577, 0];...
    [-0.1723, 0.2530, 0.4584, 0];...
    [-0.3363, 0.1721, 0.4431, 0];...
    [-0.4255, 0.1254, 0.4549, 0];...
    [-0.1321, 0.2485, 0.4022, 0];...
    [-0.4514, -0.0695, -0.0184, 0];...
    [-0.2121, -0.0978, -0.0624, 0];...
    [-0.0605, 0.1347, -0.0308, 0];...
    [-0.3756, 0.1347, -0.0032, 0];...
    [-0.0730, 0.3760, 0.6946, 0];...
    [-0.6024, -0.0318, 0.2144, 0];...
    [-0.0605, 0.1233, -0.1538, 0]];


%Creating array to store transformation matrix's of end effector
transforms = zeros(4,4,20);

%Populating array with the transformation matrix of the end effector at
%each position a picture of the checkerboard was taken
%Forward kinematics was used on the Dobot Magician to determine the pose
for i = 1:20
    transforms(:,:,i) = dobot.model.fkine(qValues(i,:)).T;
end

%Creating array to store the relative transformation matrix's
relativeEEPoses = zeros(4,4,10);

%Populating end effector relative transform matrix
%Multiplying the inverse of the first pose by the transform of the second
%pose to get the relative pose between two positions
for i=2:11
    relativeEEPoses(:,:,i-1) = inv(transforms(:,:,i-1))*transforms(:,:,i);
end

%%%%%%%%%%%%%%%%%%%% Relative Transform calculation of Camera

%Sourcing the file of recorded images of the checkerboard 
images = imageSet(fullfile(toolboxdir('vision'),'visiondata',...
            'calibration','assignmentImages'));
imageFileNames = images.ImageLocation;

%Returns the detected points and dimensions of the checkerboard
[imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);

%Inputting the real life measurement of the checkerboard used in our
%calibration
squareSizeInMM = 23;

%Generates checkerboard corner locations and returns the world coordinates
%of these corners
worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);


%Calintration occurs here
I = readimage(images,1); 
imageSize = [size(I, 1),size(I, 2)];
params = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);

%Creating an array to store the camera transformation matrix for each
%picture taken
camTransf = zeros(4,4,11);

%Population camera transformation array
%The camera transformation matrix is determined through the extrinsic
%parameters of the camera 
for i=1:11
    camTransf(1:3,1:3,i) = params.PatternExtrinsics(i,1).R;
    camTransf(1:3,4,i) = params.PatternExtrinsics(i,1).Translation';
    camTransf(4,4,i)= 1;
end

%Creating array to store the camera's relative transform matrix
relativeCamPoses = zeros(4,4,10);

%Populating array of camera relative poses
%Multiplying the inverse of the first pose by the transform of the second
%pose to get the relative pose between two positions
for i=2:11
    relativeCamPoses(:,:,i-1) = inv(camTransf(:,:,i-1))*camTransf(:,:,i);
end

%%%%%%%%%%% AX = XB Calculation using Sylvester Equation

%Identity matrix creation for C variable of Sylvester Equation
C = eye(4);

%Creating an array to store the relative poses
relativePose = zeros(4,4,10);

%Relative pose calculation between end effector and camera
for i = 1:10
relativePose(:,:,i) = sylvester(relativeEEPoses(:,:,i), relativeCamPoses(:,:,i),C);
end    


