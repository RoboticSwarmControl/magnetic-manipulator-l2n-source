function [ NewState, NewErrorCovMatrix, NewMeasuredAngle] = KalmanFilter( PreviousState, PreviousErrorCovMatrix, LastMeasuredPosition, NewPositionMeasured,Time, LastTime,coil_radius,T, DragCoef,m, Mass,J, I, MeasuredVelocity, PreviousMeasuredAngle,DeltaTMeasure,Act)
%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018


dt=Time-LastTime; %calculate time step

%DragCoef is a coluunm vector containing [drag coefficient alog x axis;drag
%coefficient alog y axis; rotational drag coeficient]
%mass: mass of the robot [Kg]
%I: [A] current vector, i.e. control input

%NewPositionMeasured : boolean, true if a new position measurement was
%obtained

%Initialize Q and R covariance matrices
Q=80.*[1,0,0,0.5,0,0;0,1,0,0,0.5,0;0,0,1,0,0,0.5;0.5,0,0,10,0,0;0,0.5,0,0,10,0;0,0,0.5,0,0,100];
%Q=[(dt.^4)./4,(dt.^3)./2,(dt.^3)./2,(dt.^3)./2,(dt.^3)./2,(dt.^3)./2;(dt.^3)./2,(dt.^4)./4,(dt.^3)./2,(dt.^3)./2,(dt.^3)./2,(dt.^3)./2;(dt.^3)./2,(dt.^3)./2,(dt.^4)./4,(dt.^3)./2,(dt.^3)./2,(dt.^3)./2;(dt.^3)./2,(dt.^3)./2,(dt.^3)./2,dt.^2,(dt.^3)./2,(dt.^3)./2;(dt.^3)./2,(dt.^3)./2,(dt.^3)./2,(dt.^3)./2,dt.^2,(dt.^3)./2;(dt.^3)./2,(dt.^3)./2,(dt.^3)./2,(dt.^3)./2,(dt.^3)./2,dt.^2];
R=0.00001.*[0.1,0,0,0,0,0;0,0.1,0,0,0,0;0,0,0.1,0,0,0;0,0,0,0.01,0,0;0,0,0,0,0.01,0;0,0,0,0,0,0.01]; %]eye(length(PreviousState));

%Build the A matrix of the Kalman Filter
A=[1 0 0 dt 0 0; 0 1 0 0 dt 0; 0 0 1 0 0 dt;0 0 0 1-dt.*DragCoef(1)./Mass 0 0; 0 0 0 0 1-dt.*DragCoef(2)./Mass 0; 0 0 0 0 0 1-dt.*DragCoef(3)./J];

%Build the B matrix of the Kalman Filter

B=[Act(1,:).*dt.*dt.*0.5./Mass; Act(2,:).*dt.*dt.*0.5./Mass; Act(3,:).*dt.*dt.*0.5./J; Act(1,:).*dt./Mass;Act(2,:).*dt./Mass;Act(3,:).*dt./J];

%////////////////////////////////////////////////////////////////////////
%Prediction Step
NewState=A*PreviousState+B*I;

NewErrorCovMatrix=A*PreviousErrorCovMatrix*A'+Q;


%Measurement update step

if NewPositionMeasured

   %The controller gives the angle at +-pi, we first have to figure out
   %which id the correct angle. The sosultion chosen is the one that is closest to the estimated state 
    ThetaPossible1=LastMeasuredPosition(3);
    ThetaPossible2=LastMeasuredPosition(3)-pi;
    
    ThetaPossible=[ThetaPossible1;ThetaPossible2];
    ChangeErrorAnglePossible=abs([NewState(3);NewState(3)]-ThetaPossible);
    
    [NotUsed,Index]=min(ChangeErrorAnglePossible);
    Theta=ThetaPossible(Index);
  
    NewMeasuredAngle=Theta;
    
   
   Measurement=[LastMeasuredPosition(1);LastMeasuredPosition(2);Theta;MeasuredVelocity(1);MeasuredVelocity(2);(Theta-PreviousMeasuredAngle)./DeltaTMeasure];
   
   
   C=eye(6,6);

    %Calculate Kalman Coeficient
    K=(NewErrorCovMatrix*C')/(C*NewErrorCovMatrix*C'+R);
    
    %Calculate a posteriori state
    NewState=NewState+K*(Measurement-C*NewState);
    
else
    NewMeasuredAngle=PreviousMeasuredAngle;
    
end

end