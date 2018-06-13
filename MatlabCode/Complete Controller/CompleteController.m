function [NewState, NewErrorCovMatrix , I, TargetPointX, TargetPointY, XHighResTrajectory,YHighResTrajectory, HistoryErrorVelocityOut , HistoryErrorPositionOut,TargetPointAngle, VelocitySetpoint,LastPositionMeasureTime, PreviousCorrectAngleMeasuredOut, ActOut] = CompleteController(PreviousMeasuredPosition, MeasuredPosition, HistoryErrorPosition, HistoryErrorVelocity, time,TrajectoryData,m, LastPositionMeasureTime,LastTime,Mass , DragCoef,PreviousState, PreviousErrorCovMatrix, J, IPrevious, PreviousCorrectAngleMeasuredIn,SpeedOverwrite,ActIn)

%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018

%This function is the complete controller composed of the filter, the
%trajectory controller, thr velocity controller and the inverse
%calculeation

%It takes as input the Position of the robot, the trajectory data and the previous sate data.

TrajectoryData=[-0.0577500000000000,-0.00550000000000000,0,0.0100000000000000;-0.0577500000000000,0.00770000000000000,0,0.0100000000000000;-0.0577500000000000,0.0192500000000000,0,0.0100000000000000;-0.0569250000000000,0.0357500000000000,0.000235500000000000,0.0100000000000000;-0.0412500000000000,0.0357500000000000,0.000117809724509617,0.0100000000000000;-0.0396000000000000,-0.0187000000000000,0,0.0100000000000000;-0.0247500000000000,-0.0187000000000000,-7.50000000000001e-05,0.0100000000000000;-0.0214500000000000,0.0275000000000000,-0.000235619449019235,0.0100000000000000;-0.0165000000000000,0.0440000000000000,-0.000435000000000000,0.0100000000000000;0.0198000000000000,0.0440000000000000,-0.000435000000000000,0.0100000000000000;0.0198000000000000,0.0110000000000000,-0.000353561944901924,0.0100000000000000;0.0412500000000000,0.0110000000000000,-0.000235619449019235,0.0100000000000000;0.0412500000000000,0.0357500000000000,-0.000235619449019235,0.0100000000000000;0.0577500000000000,0.0357500000000000,0,0.0100000000000000;0.0577500000000000,-0.0335500000000000,0.000240000000000000,0.0100000000000000;0.0412500000000000,-0.0335500000000000,0.000150000000000000,0.0100000000000000;0.0412500000000000,-0.00550000000000000,7.50000000000001e-05,0.0100000000000000;0.0198000000000000,-0.00550000000000000,-7.50000000000001e-05,0.0100000000000000;0.0198000000000000,-0.0335500000000000,-0.000300000000000000,0.0100000000000000;0.00329999999999999,-0.0335500000000000,-0.000450000000000000,0.0100000000000000;0.00329999999999999,0.0275000000000000,-0.000375000000000000,0.0100000000000000;-0.00330000000000001,0.0275000000000000,-0.000375000000000000,0.0100000000000000;-0.00495000000000000,-0.0220000000000000,-0.000375000000000000,0.0100000000000000;-0.0148500000000000,-0.0418000000000000,0,0.0100000000000000;-0.0495000000000000,-0.0418000000000000,0,0.0100000000000000;-0.0569250000000000,-0.0253000000000000,0,0.0100000000000000;-0.0577500000000000,-0.0170500000000000,0,0.0100000000000000];
TrajectoryData(:,4)=SpeedOverwrite.*TrajectoryData(:,4);

T=0.31;%[m] length of the side of the cube formed by the coils.
coil_radius=0.1; %[m] Average redius of the electromagnets
NbTurns=795; %[no unit] Number of turns in the electromagnet
MaxI=1.4; %Maximum current in the coils (saturation value)

KpXY=30; %Proportional component of trajectory pid
KdXY=0; %Derivative component of trajectory pid [not implemented, this constant is not used yet]
KiXY=0; %Integral component of trajectory pid [not implemented, this constant is not used yet]
KpTheta=5e-2; %Proportional component of trajectory pidf
KdTheta=0; %Derivative component of trajectory pid [not implemented, this constant is not used yet]
KiTheta=0; %Integral component of trajectory pid [not implemented, this constant is not used yet]
KpFD=0.003;%1;%.01;%;s1%1;%0.01;%e-2; %Proportional component of velocity pid
KpFQ=0.0003;
KdF=0.001; %Derivative component of velocity pid [not implemented, this constant is not used yet]
KiF=3e-3; %Integral component of velocity pid
KpT=1.0e-3; %Proportional component of velocity pid
KdT=0; %Derivative component of velocity pid [not implemented, this constant is not used yet]
KiT=0e-4; %Integral component of velocity pid
MaxVelocity=1; %[m/s]Maximum velocity setpoint


if PreviousMeasuredPosition==MeasuredPosition
    NewPositionMeasured=false;
    MeasuredVelocity=0;
    
    MeasureDuration=0; %This variable needs to be assigned but is not used since no measurement eas acquired
else
    NewPositionMeasured=true;
    MeasureDuration=time-LastPositionMeasureTime;
    LastPositionMeasureTime=time;
    MeasuredVelocity=(MeasuredPosition-PreviousMeasuredPosition)./MeasureDuration;
end
    

%[ FilteredPosition , CalculatedVelocity ]=Filter2( [MeasuredPosition], PreviousFilteredPosition,PreviousCalculatedVelocity, NewPositionMeasured, MeasureDuration )   ;
 
[ NewState, NewErrorCovMatrix, PreviousCorrectAngleMeasuredOut ] = KalmanFilter( PreviousState, PreviousErrorCovMatrix, MeasuredPosition, NewPositionMeasured,time, LastTime,coil_radius,T, DragCoef, m, Mass, J, NbTurns.*IPrevious, MeasuredVelocity, PreviousCorrectAngleMeasuredIn,MeasureDuration,ActIn);


P=[NewState(1),NewState(2),NewState(3)];
CalculatedVelocity=[NewState(4),NewState(5),NewState(6)];

[TargetPointX,TargetPointY,TargetPointAngle,VelocitySetpointX,VelocitySetpointY,VelocitySetpointTheta,HistoryErrorPosXOutput,HistoryErrorPosYOutput,HistoryErrorPosThetaOutput, XHighResTrajectory,YHighResTrajectory] = TrajectoryController2(P,TrajectoryData,KpXY,KdXY,KiXY,KpTheta,KdTheta,KiTheta, MaxVelocity,0,0,0);
HistoryErrorAngleVelocity=0;

%Perform the transform to the DQ coordinate system
%It is a rotation by an angle equal to the angle of the robot;

%build rotation matrix
RotationMatrix=[cos(NewState(3)), sin(NewState(3)); -sin(NewState(3)),cos(NewState(3))];

%Rotate the velocity setpoint
VsetpointXY=[VelocitySetpointX;VelocitySetpointY];
VsetpointDQ=RotationMatrix*VsetpointXY;

%Rotate the current state of the robot
%Remark: the rotated state has only 2 components as the angle is always
%zero in the rotated reference frame

%[NewState(1);NewState(2)]
NewStateDQ=RotationMatrix*[NewState(4);NewState(5)];


%[ ForceSetpoint , TorqueSetpoint, HistoryErrorVelocity , HistoryErrorAngleVelocity ] = VelocityControllerForDQInverse( CalculatedVelocity, [VelocitySetpointX,VelocitySetpointY] , VelocitySetpointTheta , KpF,KiF,KdF, KpT,KiT,KdT ,HistoryErrorVelocity, HistoryErrorAngleVelocity,  time, CdForceModel, CdTorqueModel );
[ FDQ , TorqueSetpoint, HistoryErrorVelocityOutput , HistoryErrorAngleVelocityOutput ] = VelocityControllerDQ( NewStateDQ, VsetpointDQ,  KpFD,KpFQ, KiF,KdF, KpT,KiT,KdT);%

%rotate the force setpoint to the original axis frame
ForceSetpoint=RotationMatrix'*FDQ;
%pause(1)

[I,ActOut]=InverseMagnetics([P(1),P(2)],P(3),  TorqueSetpoint, ForceSetpoint ,coil_radius,NbTurns,m,T,MaxI);

HistoryErrorVelocityOut=HistoryErrorVelocity;%,HistoryErrorAngleVelocity];
HistoryErrorPositionOut=0;%[HistoryErrorPosXOutput,HistoryErrorPosYOutput,HistoryErrorPosThetaOutput];
VelocitySetpoint=[VelocitySetpointX, VelocitySetpointY,VelocitySetpointTheta];

end

