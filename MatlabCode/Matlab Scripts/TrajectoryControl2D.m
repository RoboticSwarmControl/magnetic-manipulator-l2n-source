%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018

%This script is an algorithm simulating the functioning of a 2D magnetic
%manipulator. The system manipulates a cylinder magnet in the X-Y plane. The cylinder
%revolution axis is in the same plane. The system has four electromagnets
%arranges in a square shape.

%Clear out everything

clc
clear all
close all
image_index=1;
%Add folder path
addpath('Filters')
addpath('Measurement Simulation')
addpath('Trajectory Controllers')
addpath('Velocity Controllers')
addpath('Inverse Magnetics')
addpath('Magnetic Field Functions')
addpath('Magnetic Field Gradient Functions')
addpath('Physics Simulation')
addpath('Trajectory Files')
addpath('Other Functions')
addpath('Matlab Scripts')
addpath('Complete Controller')


%////////////////////////////////////////////////////////////////////////
%/////////////Begining of variables and constant definition//////////////

%Variables used to control the figures
%These values can be modified by the user
PlotPosition=[10,100,1500,680]; %Position of the window [x,y,wigth width, height]
nb_pts_per_axis_plot_conditioning=100; %Number of points to plot on each axis of the actuation matrix conditioning map
Plot_Side_Length=0.15;% [m]%Length to plot on each axis of the actuation matrix conditioning map
PlotConditioning=false; % Only plot conditioning when true. Set to false to dislabe this plot and speed up the conputation
NbPtsTimePlot=20; %Number of time points to plot for the plots as function of time.
P=[0.002,-0.005,0]; %Initial position of the robot [ Xposition [m] , Yposition [m], Angle [rad]]

%Parameters of the controller
%These values can be modified by the user
HistoryErrorVelocityLegnth=300; %Number of points in the vector HistoryErrorVelocity. This vector is used to compute the integral component of the Velocity controller. It has a finite length and therfore allow forgeting old errors.
P=[0.002,-0.005,0]; %Initial position of the robot [ Xposition [m] , Yposition [m], Angle [rad]]


%Other Constants definition
%These values can be modified by the user
T=0.31;%[m] length of the side of the cube formed by the coils.
Robot_Length=6.3e-3;%Length of the cylinder magnet[m]
Robot_diameter=1.56e-3;%[m] Diameter of the cylinder magnet[m]
RobotDensity=8000; %[Kg/m^3] Density of the robot, 8000 for Neodium magnets
coil_radius=0.1; %[m] Average radius of the electromagnets
NbTurns=795; %[no unit] Number of turns in the electromagnet
M=0.87e6; %Magnetization of the magnet [A/m]
CdForceModel=0.0015; %[N.s/m] Drag coefficient used in the controller internal model
CdTorqueModel=0.00000005; 
RealCd=0.0015; %[N.s/m] Real Drag coefficient of the robot
ControllerSpeedFactor=5; %[] 
deltaT=0.0002; %[s] Time step for physics simulation

%Program Variables initialization, modify at your own risk.
time=0; %starti ng time.
LastTime=time-deltaT;
DragCoef=[CdForceModel,CdForceModel,CdTorqueModel];
TimeVector=0; %Time vector initialization. Used for plot
VelocitySetpointVector=[0;0]; %Velocity Setpoint vector initialization. Used for plot
VelocityVector=[0;0;0]; %Velocity vector initialization. Used for plot
V=[0,0,0]; %Velocity of the robot [ Xvelocity [m/s] , Yvelocity [m/s], Angle Velocity [rad/s]]
MaxB=0; %Maximum norm of flux density encountered in the map (used to set maximum flux density value of the map display)
HistoryErrorVelocity=zeros(3,HistoryErrorVelocityLegnth); %Initialize HistoryErrorVelocity
PhysicsIterNumber=0; %iteration number of physics simulation
NewErrorCovMatrix=eye(6,6); %Initialization of the covariance matrix used by the Kalman Filter
I=[0;0;0;0]; %Initialization of the current
StateVector=[0;0;0;0;0;0]; %Initialization of the state vector
IVector=[0;0;0;0]; %Initialization of the current vector
MeasuredPositionVector=[0;0;0]; %Initialization of the measured position vector
RealPositionVector=[0;0;0]; %Initialization of the real position vector
Act=zeros(3,4); %Initialization of the actuation matrix
NewState=[P(1);P(2);P(3);0;0;0]; %Initialize robot initial state
CorrectAngleMeasured=P(3); %Initialize first measured angle
NewPositionMeasured=true; %Tell the controller a new position was measured

%Load trajectory data
temp=load('TrajectoryDataUH');
TrajectoryData=temp.TrajectoryData;


%Calculate constants used in the program
m=pi.*((Robot_diameter./2).^2).*Robot_Length.*M; %[A.m^2] Equivalent magnetic moment of the robot.
RobotMass=pi.*((Robot_diameter./2).^2).*Robot_Length.*RobotDensity; %[Kg] Mass of the robot
J=0.25*RobotMass*((Robot_diameter./2).^2)+(1/12)*RobotMass.*Robot_Length.*Robot_Length; %[Kg.m^2] Moment of inertia of the robot

%/////////////End of variables and constant definition//////////////
%////////////////////////////////////////////////////////////////////////





    
%/////////////////////////////////////////////////////////////////////////
%Compute 3D map of the magnetic field produced by each coil for one amp. 
%Used for plot during compuation

xmin=-T/2.*1; %[m] minimum x plot value
xmax=T/2.*1; %[m] maximum x plot value
ymin=-T/2.*1; %[m] minimum y plot value
ymax=T/2.*1; %[m] maximum y plot value
NbPtsX=1000; %Numbers of points to plot on x axis for flux density map Must be a multiple of 10 top plot quiver correctly
NbPtsY=1000; %Numbers of points to plot on y axis for flux density map Must be a multiple of 10 top plot quiver correctly
Xplot=linspace(xmin,xmax,NbPtsX); %Initialize x vector
Yplot=linspace(ymin,ymax,NbPtsY); %Initialize y vector
XplotQuiver=linspace(xmin,xmax,NbPtsX./50); %Initialize x vector
YplotQuiver=linspace(ymin,ymax,NbPtsY./50); %Initialize y vector
for i=1:NbPtsX
    for j=1:NbPtsY
        MatPlotB(j,i,:,:)=[GFunction( 1,[Xplot(i),Yplot(j)],coil_radius,T );GFunction( 2,[Xplot(i),Yplot(j)],coil_radius,T );GFunction( 3,[Xplot(i),Yplot(j)],coil_radius,T );GFunction( 4,[Xplot(i),Yplot(j)],coil_radius ,T)]  ;       %#ok<*SAGROW>     
    end
end
for i=1:NbPtsX./50
    for j=1:NbPtsY./50
        MatPlotBQuiver(j,i,:,:)=[GFunction( 1,[XplotQuiver(i),YplotQuiver(j)],coil_radius,T );GFunction( 2,[XplotQuiver(i),YplotQuiver(j)],coil_radius,T );GFunction( 3,[XplotQuiver(i),YplotQuiver(j)],coil_radius,T );GFunction( 4,[XplotQuiver(i),YplotQuiver(j)],coil_radius ,T)]  ;       %#ok<*SAGROW>     
    end
end
%END  of  compute 3D map of the magnetic field produced by each coil for one amp. 
%/////////////////////////////////////////////////////////////////////////


%Update state of the robot
LastMeasuredPosition=P;
LastMeasuredTime=0;
LastPositionMeasureTime=0;
MeasuredPosition=P;
CalculatedVelocity=[0,0,0];

%///////////////////////////////////////////////////////////////////////
%///////////Begining the main loop////////////////////////////////


while 1==1



if   ~mod(PhysicsIterNumber,ControllerSpeedFactor) %The controller runs ControllerSpeedFactor time slower than the physics simulation. The controller outputs are updated every ControllerSpeedFactor iterations.

%Simulate the measurement of the position using the camera  
 PreviousMeasuredPosition=MeasuredPosition;
[ MeasuredPosition,LastMeasuredTime ] = GaussianNoiseSensing( P, time, LastMeasuredTime,PreviousMeasuredPosition );

 
 
 HistoryErrorPosition=0;
 
 %Update data used by the Kalman filter
 PreviousState=NewState;
 PreviousErrorCovMatrix=NewErrorCovMatrix;
 
 
 %Compute controller output
 [NewState, NewErrorCovMatrix , I, TargetPointX, TargetPointY, XHighResTrajectory,YHighResTrajectory, HistoryErrorVelocityOut , HistoryErrorPositionOut,TargetPointAngle, VelocitySetpoint,LastPositionMeasureTime, CorrectAngleMeasured, Act] = CompleteController(PreviousMeasuredPosition, MeasuredPosition, HistoryErrorPosition, HistoryErrorVelocity, time,TrajectoryData,m, LastPositionMeasureTime,LastTime,RobotMass , DragCoef,PreviousState, PreviousErrorCovMatrix, J, I,CorrectAngleMeasured,15,Act);
 
 %Update last iteration time
 LastTime=time;
end

%Compute the physics
[ NewP, NewV ] = ComputePhysics1( P , V, I, RobotMass, J , deltaT,coil_radius,T,RealCd,NbTurns,m,10) ;
P=NewP; %Update current position
V=NewV; %Update current velocity

time=time+deltaT; %update time




%////////////////////////////////////////////////////////////////////////////////////////
%The rest of the code generates the figure
if   ~mod(PhysicsIterNumber,1.*ControllerSpeedFactor)

%figure(1)  
clf   %Clear previous plots
set(gcf, 'Position', PlotPosition)

%[-0.0550000000000000,-0.0350000000000000,2,0.100000000000000;-0.0650000000000000,0,1.57000000000000,0.150000000000000;-0.0450000000000000,0.0450000000000000,0.785398163397448,0.100000000000000;0,0.0600000000000000,0,0.100000000000000;0.0350000000000000,0.0500000000000000,-0.500000000000000,0.0300000000000000;0.0450000000000000,0.0400000000000000,-1.57079632679490,0.0300000000000000;0.0400000000000000,0.0300000000000000,-2.90000000000000,0.0300000000000000;0.0100000000000000,0.0300000000000000,-2.90000000000000,0.100000000000000;-0.0100000000000000,0.0300000000000000,-2.35707963267949,0.100000000000000;-0.0200000000000000,0.0100000000000000,-1.57079632679490,0.100000000000000;-0.0250000000000000,-0.0100000000000000,-1.57079632679490,0.0500000000000000;-0.0200000000000000,-0.0300000000000000,0,0.0500000000000000;-0.00300000000000000,-0.0250000000000000,1.60000000000000,0.0500000000000000;0,-0.0100000000000000,1,0.0500000000000000;0.0100000000000000,0,0.500000000000000,0.0500000000000000;0.0550000000000000,0.00500000000000000,-0.500000000000000,0.0500000000000000;0.0600000000000000,-0.00700000000000000,-2,0.0300000000000000;0.0500000000000000,-0.0150000000000000,-3,0.0300000000000000;0.0350000000000000,-0.0250000000000000,-2.50000000000000,0.0500000000000000;0.0250000000000000,-0.0400000000000000,-2.50000000000000,0.0500000000000000;0.0100000000000000,-0.0600000000000000,-2.50000000000000,0.0500000000000000;-0.0200000000000000,-0.0600000000000000,-2.50000000000000,0.0500000000000000]
%///////////////////////////////////////////////////////////////////
%Plot of the conditioning and the trajectory
figure(1)
subplot(1,2,1)

%Plot the user defined trajectory points
plot(TrajectoryData(:,1),TrajectoryData(:,2),'kx')
axis equal
hold on

%Plot the trajectory centerline
plot(XHighResTrajectory,YHighResTrajectory,'r')
hold on




%Draw the robot as a rectangle
h=DrawRectangle([P(1),P(2),0.015/2,0.0075/2,P(3)],'b');
hold on
%hmeasurement=DrawRectangle([MeasuredPosition(1),MeasuredPosition(2),0.005/2,0.005/2,P(3)],'g');
%hold on


xlabel('X Position [m]')
ylabel('Y Position [m]')
axis equal
hold on
%End of first subplot


%This part computes the magnetic flux density for the plot


for i2=1:NbPtsX
    for j2=1:NbPtsY        
        temp=MatPlotB(j2,i2,1,1);%MatPlotB(j,i2,:,:)*i2;
        Bx(j2,i2)=MatPlotB(j2,i2,1,1).*NbTurns.*I(1)+MatPlotB(j2,i2,2,1).*NbTurns.*I(2)+MatPlotB(j2,i2,3,1).*NbTurns.*I(3)+MatPlotB(j2,i2,4,1).*NbTurns.*I(4);
        By(j2,i2)=MatPlotB(j2,i2,1,2).*NbTurns.*I(1)+MatPlotB(j2,i2,2,2).*NbTurns.*I(2)+MatPlotB(j2,i2,3,2).*NbTurns.*I(3)+MatPlotB(j2,i2,4,2).*NbTurns.*I(4);
    end
end
for i3=1:length(XplotQuiver)
    for j3=1:length(YplotQuiver)        
        temp=MatPlotBQuiver(j3,i3,1,1);
        BxQuiver(j3,i3)=MatPlotBQuiver(j3,i3,1,1).*NbTurns.*I(1)+MatPlotBQuiver(j3,i3,2,1).*NbTurns.*I(2)+MatPlotBQuiver(j3,i3,3,1).*NbTurns.*I(3)+MatPlotBQuiver(j3,i3,4,1).*NbTurns.*I(4);
        ByQuiver(j3,i3)=MatPlotBQuiver(j3,i3,1,2).*NbTurns.*I(1)+MatPlotBQuiver(j3,i3,2,2).*NbTurns.*I(2)+MatPlotBQuiver(j3,i3,3,2).*NbTurns.*I(3)+MatPlotBQuiver(j3,i3,4,2).*NbTurns.*I(4);
    end
end
quiver(P(1)-0.003.*cos(P(3)),P(2)-0.003.*sin(P(3)),0.006.*cos(P(3)),0.006.*sin(P(3)),'color','k','MaxHeadSize',10,'linewidth',0.5)
axis equal
xlim([-0.1,0.1])
ylim([-0.1,0.1])
grid on
subplot(1,2,2)

%Calculate the norm of the flux density
normB=sqrt(By.*By+Bx.*Bx);

%Display the map of flux density norm
imagesc(Xplot,Yplot,normB)
hB=colorbar; %Display colorbar
hB.Location='north';
ylabel(hB,'Flux density norm [T]','FontSize', 11);

%update maximum flux density value
if max(max(normB))>MaxB
    MaxB=max(max(normB));
end
caxis([0 0.005]) %Set colorbar limits
hold on 
xlabel('X Position [m]')
ylabel('Y Position [m]')

title(['Time: ',num2str(time-0.0002),' s'])
quiver(XplotQuiver,YplotQuiver,BxQuiver,ByQuiver,3,'color',[1,0,0])

%Show the position of the robot with a restangle
%h=DrawRectangle([P(1),P(2),0.015,0.0075,P(3)],'g');
h=DrawRectangle([P(1),P(2),0.015/2,0.0075/2,P(3)],'g');
%quiver(P(1)-0.005.*cos(P(3)),P(2)-0.005.*sin(P(3)),0.01.*cos(P(3)),0.01.*sin(P(3)),'color','k','MaxHeadSize',10,'linewidth',0.5)
quiver(P(1)-0.003.*cos(P(3)),P(2)-0.003.*sin(P(3)),0.006.*cos(P(3)),0.006.*sin(P(3)),'color','g','MaxHeadSize',10,'linewidth',0.5)
%MeasuredPosition(3)
%FilteredPosition(3)
% xlim([-0.2,0.2])
% ylim([-0.2,0.2])



%adjust plot parameters
set(gca,'Ydir','normal')
axis equal
xlim([-0.1,0.1])
ylim([-0.1,0.1])

%The followings vectors store the previous values of some variables
%They are used to plot curves as a funtion of time
TimeVector=[TimeVector,time]; %#ok<*AGROW>
RealPositionVector=[RealPositionVector,[P(1);P(2);P(3)]];
IVector=[IVector,[I(1);I(2);I(3);I(4)]];
VelocityVector=[VelocityVector,[V(1);V(2);V(3)]];
VelocitySetpointVector=[VelocitySetpointVector,[VelocitySetpoint(1);VelocitySetpoint(2)]];
StateVector=[StateVector,NewState];
MeasuredPositionVector=[MeasuredPositionVector,[MeasuredPosition(1);MeasuredPosition(2);MeasuredPosition(3)]];

%Remove old values(Only a NbPtsTimePlot number of points are plotted and
%therfore stored)
if length(TimeVector)>NbPtsTimePlot
    IVector=IVector(:,2:length(TimeVector));
    VelocityVector=VelocityVector(:,2:length(TimeVector));
    VelocitySetpointVector=VelocitySetpointVector(:,2:length(TimeVector));
    StateVector=StateVector(:,2:length(TimeVector));
    MeasuredPositionVector=MeasuredPositionVector(:,2:length(TimeVector));
    RealPositionVector=RealPositionVector(:,2:length(TimeVector));
    TimeVector=TimeVector(2:length(TimeVector));
end

%%///////////////////////////////////////////////////////////////////////
%%Optional figure, uncomment if needed
%%/////////////////////////////////////////////////////////////////////
% 
% figure(2)
% subplot(2,2,1)
% plot(TimeVector,VelocitySetpointVector(1,:),'b--',TimeVector,VelocitySetpointVector(2,:),'r--',TimeVector,VelocityVector(1,:),'b-',TimeVector,VelocityVector(2,:),'r-')
% legend('X Velocity Setpoint','Y Velocity Setpoint','X Velocity','Y Velocity')
% xlabel('Time[s]')
% ylabel('Velocity [m/s]')
% 
% 
% subplot(2,2,2)
% plot(TimeVector,VelocityVector(1,:),'b-',TimeVector,StateVector(4,:),'r-')
% legend('Real X Velocity','Estimated X Velocity')
% xlabel('Time[s]')
% ylabel('Velocity [m/s]')
% 
% subplot(2,2,3)
% plot(TimeVector,MeasuredPositionVector(1,:),'b-',TimeVector,StateVector(1,:),'r--',TimeVector,RealPositionVector(1,:))
% legend('Measured X Position','Estimated X Position','Real X Position')
% xlabel('Time[s]')
% ylabel('Position [m]')
% 
% subplot(2,2,4)
% 
% plot(TimeVector,IVector(1,:),TimeVector,IVector(2,:),TimeVector,IVector(3,:),TimeVector,IVector(4,:))
% legend('I1','I2','I3','I4')
% xlabel('Time[s]')
% ylabel('Current [A]')
% grid on
% drawnow
%%///////////////////////////////////////////////////////////////////////
%%END OF Optional figure
%%/////////////////////////////////////////////////////////////////////
% 
bar([1 10 7 8 2 2 9 3 6])
fig = gcf;
fig.InvertHardcopy = 'off';
filename=[char(string(image_index)),'.png']
saveas(gcf,filename)

drawnow
image_index=image_index+1;
end
%Update iteration counter
PhysicsIterNumber=PhysicsIterNumber+1;


%///////////END of the main loop////////////////////////////////
%///////////////////////////////////////////////////////////////////////
end