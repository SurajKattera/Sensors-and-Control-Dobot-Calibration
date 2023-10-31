close all
clear all


%Subscribing to Intel RealSense RGB image rostopic
imageSubscriber = rossubscriber('/camera/color/image_raw');
pause(2);

%Subscribing to Dobot Magician end effector poses rostopic
endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses'); 
pause(2); 

%Subscribing to Dobot Magician joint states rostopic
jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); 
pause(2); 


%Focal length and camera projection points obtained from calibration
f = 467.33;  
px = 653.06;
py = 370.90;
l = 0.4; 


%%%Target checkerboard coordinates
Target = [  98.029823,124.24613;
97.581894,	175.09007;
98.180656,	232.20535;
98.784599,	283.78436;
98.541260,	340.70947;
98.489349,	392.51096;
149.18468,	123.80252;
150.62863,	175.45163;
150.11748,	391.98083;
150.82480,	230.37936;
151.46211,	284.20789;
151.62430,	176.68651;
152.37946,	338.26138;
205.04482,	176.32657;
206.36974,	123.57497;
205.99097,	230.49854;
206.15982,	284.34326;
205.89124,	337.63004;
207.44662,	391.42401;
257.80109,	123.11823;
259.29324,	391.21695;
259.66595,	175.65617;
259.87192,	229.22629;
260.18698,	284.25458;
261.14481,	337.18588;
312.95850,	175.80249;
314.08447,	229.56863;
313.90051,	284.13818;
313.97327,	336.91351;
314.74115,	122.43214;
316.53714,	390.78210;
366.18488,	121.49434;
367.17163,	173.40294;
367.36356,	229.66827;
367.77753,	282.17322;
368.35699,	337.88535;
368.26477,	390.42542;
421.40057,	174.78711;
421.87552,	227.22845;
422.18103,	283.62039;
422.51974,	335.82733;
424.01630,	120.56118;
426.13281,	390.85281;
476.39630,	119.45531;
477.14035,	171.51414;
476.88132,	228.70683;
477.30649,	281.36819;
478.41516,	337.92551;
478.85843,	390.87396;
480.41141,	336.21780;
532.49609,	173.34612;
532.53912,	225.46172;
533.23090,	283.70401;
533.71869,	335.49115;]

%%%%%%%%%%%%%%%%%%%%%Start of continuous visual servo loop
while true

% Live end effector depth    
currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;    
Z = currentEndEffectorPoseMsg.Pose.Position.Z; 

% Live image feed and conversion to grayscale
img = readImage(imageSubscriber.LatestMessage);
img_new = rgb2gray(img);

% Calculate corner points using Harris Feature detector
cp = detectHarrisFeatures(img_new);

%Storing the current observed corner points  
Obs = cp.Location; % Where the image corners are observed currently


%Conversion of target points and observed points from pixel coordinates to
%real world coordinates
xy = [(Target(1:30,1)-px)/f,(Target(1:30,1)-py)/f];
Obsxy = [(Obs(1:30,1)-px)/f,(Obs(1:30,1)-py)/f];

%Creating array for number of points
%30 points used to prevent harris corner mismatch errors
n = length(Target(1:30,1));

%Interaction matrix Lx function used
Lx = [];
for i=1:n;
    Lxi = FuncLx(xy(i,1),xy(i,2),Z);
    Lx = [Lx;Lxi];
end

%Error between observed points and target points
e2 = Obsxy-xy;
e = reshape(e2',[],1);
de = -e*l;
Lx2 = inv(Lx'*Lx)*Lx';


%Velocity vector to get to target points
Vc = -l*Lx2*e;

%Extract the current joint state
currentJointState = jointStateSubscriber.LatestMessage.Position; % Get the latest message


%Joint velocity movement timestep
timestep = 0.2; 

%Multiplying the XYZ componenets of the velocity vector by timestep
VcX = Vc(1)*timestep;
VcY = Vc(2)*timestep;
VcZ = Vc(3)*timestep;

%Set next joint target
%Velocity vector added to current joint state
jointTarget = [currentJointState(1)+VcX,currentJointState(2)+VcY,currentJointState(3)+VcZ,0]; 

%Sending joint state command to Dobot Magician
[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

end