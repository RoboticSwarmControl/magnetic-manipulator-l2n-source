function [ I ,D] = InverseMagnetics(Robot_Position,AngleRobot, Torque, F ,coil_radius,NbTurns,m,T,maxI)

%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018

%INverse magnetic calculation
mx=m.*cos(AngleRobot);
my=m.*sin(AngleRobot);

G1=GFunction(1,Robot_Position,coil_radius,T);
G2=GFunction(2,Robot_Position,coil_radius,T);
G3=GFunction(3,Robot_Position,coil_radius,T);
G4=GFunction(4,Robot_Position,coil_radius,T);

Gx=[G1(1) G2(1) G3(1) G4(1)];
Gy=[G1(2) G2(2) G3(2) G4(2)];

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

D=[(mx.*dGx_dx+my.*dGy_dx);(mx.*dGx_dy+my.*dGy_dy);mx.*Gy-my.*Gx];

Saturating=false;

Solution=D'*inv(D*D')*[F(1);F(2);Torque];

Current1=Solution(1)/NbTurns; %[A] 
Current2=Solution(2)/NbTurns; %[A]
Current3=Solution(3)/NbTurns;%[A]
Current4=Solution(4)/NbTurns; %[A]

%Scale down I vector if saturation is detected
while (Current1>maxI || Current2>maxI || Current3>maxI || Current4>maxI || Current1<-maxI || Current2<-maxI || Current3<-maxI || Current4<-maxI)
F=0.95.*F;
Torque=0.95.*Torque;
Solution=D'*inv(D*D')*[F(1);F(2);Torque];
Saturating=true;
Current1=Solution(1)/NbTurns; %[A] 
Current2=Solution(2)/NbTurns; %[A]
Current3=Solution(3)/NbTurns;%[A]
Current4=Solution(4)/NbTurns; %[A]
end

if Saturating
    disp(['Current Saturates'])
end
I=[Current1;Current2;Current3;Current4];
end

