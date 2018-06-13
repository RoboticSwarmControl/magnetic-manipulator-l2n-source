function [TargetPointX,TargetPointY,TargetPointTheta,VelocitySetpointX,VelocitySetpointY,VelocitySetpointTheta,HistoryErrorPosXOutput,HistoryErrorPosYOutput,HistoryErrorPosThetaOutput, XHighResTrajectory,YHighResTrajectory] = TrajectoryController2(FilteredPosition,TrajectoryData,KpXY,KdXY,KiXY,KpTheta,KdTheta,KiTheta, MaxVelocity,HistoryErrorPosXInput,HistoryErrorPosYInput,HistoryErrorPosThetaInput)

%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018

%This function is the trajectory controller
%This function takes as input the position of the robot and the trajectoiry
%data. It calculates the velocity setpoint to be sent to the velocity
%controller.
P=FilteredPosition;
TrajectoryData=[TrajectoryData;TrajectoryData(1,:)]; %The last point of the trajectory is set to be the same as the first point.
[n,m]=size(TrajectoryData);
%DistanceTargetPointResolution=0.0001; %[m] distance step to search the closest point of the trajectory
NbPtsInterp=1000;%Number of points used to interpolate the trajectory


XTrajectory=TrajectoryData(:,1);
YTrajectory=TrajectoryData(:,2);
AngleTrajectory=TrajectoryData(:,3);
VelocityTrajectory=TrajectoryData(:,4);


%Create high resolution trajectory via interpolation

ExtraPoints=3; %positive integer This variable controls the number of points to add at the begining and the end of the trajectory data to make a smouth closed loop. Try to set to 0 to see its effect.  

InitialLength=length(VelocityTrajectory);
% NegativeDistanceVelocityTrajectory=zeros(ExtraPoints,1);
% NegativeDistanceXTrajectory=zeros(ExtraPoints,1);
% NegativeDistanceYTrajectory=zeros(ExtraPoints,1);
% NegativeDistanceAngleTrajectory=zeros(ExtraPoints,1);
LowResIndex=(linspace(0,1,n))';
% NegativeDistanceLowResIndex=zeros(ExtraPoints,1);
% for i=1:ExtraPoints
%     VelocityTrajectory(InitialLength+i)=VelocityTrajectory(InitialLength)+VelocityTrajectory(i);
%     XTrajectory(InitialLength+i)=VelocityTrajectory(InitialLength)+VelocityTrajectory(i);
%     YTrajectory(InitialLength+i)=YTrajectory(InitialLength)+YTrajectory(i);
%     AngleTrajectory(InitialLength+i)=AngleTrajectory(InitialLength)+AngleTrajectory(i);
%    LowResIndex(InitialLength+i)=LowResIndex(InitialLength)+LowResIndex(i);
%     NegativeDistanceVelocityTrajectory(ExtraPoints-i+1)=-VelocityTrajectory(i);
%     NegativeDistanceXTrajectory(ExtraPoints-i+1)=-XTrajectory(i);
%     NegativeDistanceYTrajectory(ExtraPoints-i+1)=-YTrajectory(i);
%     NegativeDistanceAngleTrajectory(ExtraPoints-i+1)=-AngleTrajectory(i);
%     NegativeDistanceLowResIndex(ExtraPoints-i+1)=-LowResIndex(i);
% end
%     VelocityTrajectory=[NegativeDistanceVelocityTrajectory;VelocityTrajectory];
%     XTrajectory=[NegativeDistanceXTrajectory;XTrajectory];
%     YTrajectory=[NegativeDistanceYTrajectory;YTrajectory];
%     AngleTrajectory=[NegativeDistanceAngleTrajectory;AngleTrajectory];
%     LowResIndex=[NegativeDistanceLowResIndex;LowResIndex];
    

HighResIndex=linspace(0,1,NbPtsInterp);

XHighResTrajectory = (interp1(LowResIndex,XTrajectory,HighResIndex,'spline'))';
YHighResTrajectory = (interp1(LowResIndex,YTrajectory,HighResIndex,'spline'))';
AngleHighResTrajectory = (interp1(LowResIndex,AngleTrajectory,HighResIndex,'spline'))';
VelocityHighResTrajectory = (interp1(LowResIndex,VelocityTrajectory,HighResIndex,'spline'))';
Curvature=LineCurvature2D([XHighResTrajectory';YHighResTrajectory']');
Curvature=[Curvature;Curvature];
for yi=1:length(VelocityHighResTrajectory)
    VelocityHighResTrajectory(yi)=VelocityHighResTrajectory(yi)./(1+0.003.*abs(Curvature(yi+2)));
end
%//////////////////////////////////////////////////////
%Sarch for the position of the trajectory that is the closest to the
%robot position. This position will be called TargetPoint. 
%DistanceTargetPoint is the distance that the robot would  needs to travel to reach TargetPoint from the first point on the trajectory if the control is perfect (the robot is always on the trajectory centerline). 
%The trajectory is devined using via points. An
%interpolation needs to be made between the via points. 
%Distance TargetPoint is used as an index to compute TargetPoint using the
%discretized path TrajectoryData.

RobotDistanceFromPath=sqrt((P(1)-XHighResTrajectory).^2+(P(2)-YHighResTrajectory).^2);
[NotUsed,MinimumDistanceIndex]=min( RobotDistanceFromPath);


% PathDistance=zeros(n,1);
% for i=2:n
%     PathDistance(i,1)=PathDistance(i-1)+sqrt((TrajectoryData(i,1)-TrajectoryData(i-1,1)).^2+(TrajectoryData(i,2)-TrajectoryData(i-1,2)).^2);
% end
% 
% 
% %Create high resolution trajectory via interpolation
% 
% ExtraPoints=3; %positive integer This variable controls the number of points to add at the begining and the end of the trajectory data to make a smouth closed loop. Try to set to 0 to see its effect.  
% 
% InitialLength=length(PathDistanceHighRes);
% for i=1:ExtraPoints
%     PathDistance(InitialLength+i)=PathDistance(InitialLength)+PathDistance(i);
%     NegativePathDistance(i)=-PathDistance(i);
% end
%     
% PathDistanceHighRes=linspace(0,PathDistance(n,1),round(PathDistance(n,1)/DistanceTargetPointResolution));
% XHighResTrajectory = (interp1(PathDistance,XTrajectory,PathDistanceHighRes,'spline'))';
% YHighResTrajectory = (interp1(PathDistance,YTrajectory,PathDistanceHighRes,'spline'))';
% AngleHighResTrajectory = (interp1(PathDistance,AngleTrajectory,PathDistanceHighRes,'spline'))';
% VelocityHighResTrajectory = (interp1(PathDistance,VelocityTrajectory,PathDistanceHighRes,'spline'))';
%     
% RobotDistanceFromPath=sqrt((P(1)-XHighResTrajectory(:,1)).^2+(P(2)-YHighResTrajectory(:,1)).^2);
% [NotUsed,MinimumDistanceIndex]=min( RobotDistanceFromPath);
% 
TargetPointX=XHighResTrajectory(MinimumDistanceIndex);
TargetPointY=YHighResTrajectory(MinimumDistanceIndex);
TargetPointTheta=AngleHighResTrajectory(MinimumDistanceIndex);
TargetVelocityNorm=VelocityHighResTrajectory(MinimumDistanceIndex);

if MinimumDistanceIndex==length(VelocityHighResTrajectory)
TargetPathOrientation=atan2(YHighResTrajectory(2)-TargetPointY, XHighResTrajectory(2)-TargetPointX);  
else
TargetPathOrientation=atan2(YHighResTrajectory(MinimumDistanceIndex+1)-TargetPointY,XHighResTrajectory(MinimumDistanceIndex+1)-TargetPointX);
end
TargetVelocityX=TargetVelocityNorm*cos(TargetPathOrientation);
TargetVelocityY=TargetVelocityNorm*sin(TargetPathOrientation);

ErrorPosX=TargetPointX-P(1);
ErrorPosY=TargetPointY-P(2);

VelocityXToCorrectPosition=KpXY.*ErrorPosX;
VelocityYToCorrectPosition=KpXY.*ErrorPosY;

%ErrorPosTheta=TargetPointTheta-P(3);
%VelocityThetaToCorrectPosition=KpTheta.*ErrorPosTheta;

if TargetPointTheta>(pi/2) && P(3)<(pi-TargetPointTheta)
    VelocityThetaToCorrectPosition=KpTheta.*(2.*pi-TargetPointTheta+P(3));
elseif TargetPointTheta<-(pi/2) && P(3)>(pi+TargetPointTheta)
    VelocityThetaToCorrectPosition=KpTheta.*(2.*pi+TargetPointTheta-P(3));
else
    VelocityThetaToCorrectPosition=KpTheta.*(TargetPointTheta-P(3));
end
VelocitySetpointTemp=[VelocityXToCorrectPosition+TargetVelocityX , VelocityYToCorrectPosition+TargetVelocityY];


if norm(VelocitySetpointTemp)>MaxVelocity
    VelocitySetpointTemp=MaxVelocity.*VelocitySetpointTemp./norm(VelocitySetpointTemp);
end
VelocitySetpointX=VelocitySetpointTemp(1);
VelocitySetpointY=VelocitySetpointTemp(2);
VelocitySetpointTheta=VelocityThetaToCorrectPosition;
HistoryErrorPosXOutput=0;
HistoryErrorPosYOutput=0;
HistoryErrorPosThetaOutput=0;
