function [ NewP, NewV ] = ComputePhysics1( P,V, I , RobotMass, J,deltaT ,coil_radius,T,Cd,NbTurns,m,maxI)
%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018


X=P(1);
Y=P(2);
Angle=P(3);
Vx=V(1);
Vy=V(2);
angularVel=V(3);

%Simulate saturation of currents
for i=1:4
    if I(i)>maxI
        I(i)=maxI;
    end
    if I(i)<-maxI
        I(i)=-maxI;
    end
end
    

%Perform the forward magnetic calculation
I=NbTurns.*I;

mx=m.*cos(Angle);
my=m.*sin(Angle);

%Here begins the calculation of the torque
Robot_Position=[X,Y];
G1=GFunction(1,Robot_Position,coil_radius,T);
G2=GFunction(2,Robot_Position,coil_radius,T);
G3=GFunction(3,Robot_Position,coil_radius,T);
G4=GFunction(4,Robot_Position,coil_radius,T);

Gx=[G1(1) G2(1) G3(1) G4(1)];
Gy=[G1(2) G2(2) G3(2) G4(2)];

Torque=(mx.*Gy-my.*Gx)*I;
%Here begins the calculation of the force

dG1dx=dGFunction( 1,Robot_Position,coil_radius,1,T );
dG1dy=dGFunction( 1,Robot_Position,coil_radius,2,T );

dG2dx=dGFunction( 2,Robot_Position,coil_radius,1,T );
dG2dy=dGFunction( 2,Robot_Position,coil_radius,2,T );

dG3dx=dGFunction( 3,Robot_Position,coil_radius,1,T );
dG3dy=dGFunction( 3,Robot_Position,coil_radius,2,T );

dG4dx=dGFunction( 4,Robot_Position,coil_radius,1,T );
dG4dy=dGFunction( 4,Robot_Position,coil_radius,2,T );

dGx_dx= [dG1dx(1) dG2dx(1) dG3dx(1) dG4dx(1)];
dGx_dy= [dG1dy(1) dG2dy(1) dG3dy(1) dG4dy(1)];
dGy_dx= [dG1dx(2) dG2dx(2) dG3dx(2) dG4dx(2)];
dGy_dy= [dG1dy(2) dG2dy(2) dG3dy(2) dG4dy(2)];


ForceX=(mx.*dGx_dx+my.*dGy_dx)*I;
ForceY=(mx.*dGx_dy+my.*dGy_dy)*I;

%////////////////////////////////////////////////////////////
F=[ForceX,ForceY];

FrictionX=-Cd.*Vx;
FrictionY=-Cd.*Vy;

accelerationX=(1./RobotMass).*(F(1)+FrictionX);
accelerationY=(1./RobotMass).*(F(2)+FrictionY);

NewVelocityX=Vx+accelerationX.*deltaT;
NewVelocityY=Vy+accelerationY.*deltaT;
NewPositionX=X+NewVelocityX.*deltaT+0.5.*accelerationX.*deltaT.*deltaT;
NewPositionY=Y+NewVelocityY.*deltaT+0.5.*accelerationY.*deltaT.*deltaT;


domega=(Torque-5e-8.*angularVel)./J; %This has to be modified
NewAngleVel=angularVel+domega.*deltaT;
NewAngle=Angle+NewAngleVel.*deltaT+0.5.*domega.*deltaT.*deltaT;

% if NewAngle>pi
%     NewAngle=NewAngle-2*pi;
%    % ResetAngle=1;
% end
% 
% if NewAngle<pi
%     NewAngle=NewAngle+2*pi;
%     %ResetAngle=1;
% end

%Process Noise
VarianceOfNoiseVelocity=0.00025;% [m/s]

NoiseVelocityPosition = VarianceOfNoiseVelocity.*randn(1,2);

VarianceOfNoiseAngleVelocity=0.1;% [rad/s]

NoiseAngleVelocityPosition = VarianceOfNoiseAngleVelocity.*randn(1,1);

NewP=[NewPositionX,NewPositionY,NewAngle];
NewV=[NewVelocityX+NoiseVelocityPosition(1),NewVelocityY+NoiseVelocityPosition(2),NewAngleVel+NoiseAngleVelocityPosition];
end

