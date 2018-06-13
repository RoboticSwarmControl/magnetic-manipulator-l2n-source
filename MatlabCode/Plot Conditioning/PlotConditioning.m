
%This Script generates 3 plots:
%_a map of the conditioning of the actuation matrix.
%_a map of showing the norm of the current to produce a force Fd along
%the d axis of the robot (direction of magnetization)
%_a map of showing the norm of the current to produce a force Fq along
%the q axis of the robot (90deg from direction of magnetization).

%clear everything
clc
clear all
close all

%Add folder path
addpath('Inverse Magnetics')
addpath('Magnetic Field Functions')
addpath('Magnetic Field Gradient Functions')


%Constant definition
%These values can safely be modifyed bu the user
F=1e-3; %[N] force to apply on the robot
AngleStep=pi/64; %[rad] The code continuously generates the plots and increases the angle of the robot by this value at each new plot
NbPtsX=20; %Number of points to plot along the X axis
NbPtsY=20; %Number of points to plot along the Y axis
XMin=-0.075; %[m] minimum X to plot
XMax=0.075; %[m] maximum X to plot
YMin=-0.075; %[m] minimum Y to plot
YMax=0.075; %[m] maximum Y to plot
Robot_Length=6.3e-3;%Length of the cylinder magnet[m]
Robot_diameter=1.56e-3;%[m] Diameter of the cylinder magnet[m]
M=0.87e6; %Magnetization of the magnet [A/m]
coil_radius=0.1; %[m] Average radius of the electromagnets
T=0.31;%[m] length of the side of the cube formed by the coils.
NormB=0.001; %[T] norm of B to apply to the robot
MaxConditioningValueToDisplay=100; %Maximum Conditioning Value to display
MaxCurrentToDisplay=1000000; %[A] %Maximum Current Value to display

%///////////////////////////////////////////////////////////////////////
%Calculate constants used in the program
X=linspace(XMin,XMax,NbPtsX);
Y=linspace(YMin,YMax,NbPtsY);
AngleRobot=0;
m=pi.*((Robot_diameter./2).^2).*Robot_Length.*M; %[A.m^2] Equivalent magnetic moment of the robot


%///////////////////////////////////////////////////////////////////////////
%Begining of the main loop
%//////////////////////////////////////////////////////////////////////

while true %Stop the program with ctrl-c

    Cond=zeros(NbPtsX,NbPtsY); %reset Matrix to store the conditioning map
    Id=zeros(NbPtsX,NbPtsY); %reset Matrix to store the norm of Id map
    Iq=zeros(NbPtsX,NbPtsY); %reset Matrix to store the norm of Iq map
    IdLS=zeros(NbPtsX,NbPtsY); %reset Matrix to store the norm of Id map
    IqLS=zeros(NbPtsX,NbPtsY); %reset Matrix to store the norm of Iq map
    
    for i=1:NbPtsX %Compute every points along X
        for k=1:NbPtsY %Compute every points along Y
  
            
            
            %//////////////////////////////////////////////////////////
            %This part solves the problem when applying Fx,Fx,Bx and By
            mx=m.*cos(AngleRobot);
            my=m.*sin(AngleRobot);
            Robot_Position=[X(i),Y(k)];
            
            %The field is aligned with the robot = no torque
            Bx=NormB*cos(AngleRobot); 
            By=NormB*sin(AngleRobot);
            
            %Begin magnetic calculation
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


            D=[Gx;Gy;cos(AngleRobot).*(mx.*dGx_dx+my.*dGy_dx)+sin(AngleRobot).*(mx.*dGx_dy+my.*dGy_dy);-sin(AngleRobot).*(mx.*dGx_dx+my.*dGy_dx)+cos(AngleRobot).*(mx.*dGx_dy+my.*dGy_dy)];
            Cond(i,k)=cond(D);
            
%             %Calculate the euclidian norm of the current for a force applied along the d-axis
             
             Id(i,k)=norm(inv(D)*[Bx;By;F;0]);
%             %Calculate the euclidian norm of the current for a force applied along the d-axis
             Iq(i,k)=norm(inv(D)*[-Bx;By;0;F]);

            %//////////////////////////////////////////////////////////
            %This part solves the problem when applying Fx,Fy,and torque
            
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

            %Calculate the euclidian norm of the current for a force applied along the d-axis
            IdLS(i,k)=norm(D'*inv(D*D')*[F.*cos(AngleRobot);F.*sin(AngleRobot);0.000]);
            %test=D'*inv(D*D')*[F.*cos(AngleRobot);F.*sin(AngleRobot);1]
            %Calculate the euclidian norm of the current for a force applied along the d-axis
            IqLS(i,k)=norm(D'*inv(D*D')*[-F.*sin(AngleRobot);F.*cos(AngleRobot);0.000]);
            
          

         end
    end
    cla
    subplot(2,3,1)
    figure(1)
    PLT1=imagesc(Cond)
    title('Condition Number')
    xlabel('X [m]')
    ylabel('Y [m]')
    caxis([0 MaxConditioningValueToDisplay])
    colorbar
    subplot(2,3,2)
    PLT2=imagesc(Id)
    title('|I| with Fd')
    xlabel('X [m]')
    ylabel('Y [m]')
    caxis([0 MaxCurrentToDisplay])
    colorbar
    subplot(2,3,3)
    PLT3=imagesc(Iq)
    title('|I| with Fq')
    xlabel('X [m]')
    ylabel('Y [m]')
    caxis([0 MaxCurrentToDisplay])
    colorbar
    subplot(2,3,5)
    PLT5=imagesc(IdLS)
    title('|I| with Fd Least Square')
    xlabel('X [m]')
    ylabel('Y [m]')
    caxis([0 MaxCurrentToDisplay])
    colorbar
    subplot(2,3,6)
    PLT6=imagesc(IqLS)
    title('|I| with Fq Least Square')
    xlabel('X [m]')
    ylabel('Y [m]')
    caxis([0 MaxCurrentToDisplay])
    colorbar
    subplot(2,3,4)
    quiver(0,0,mx,my)
    xlim([-m,m])
    ylim([-m,m])
    drawnow
    pause(0.01)
    AngleRobot=AngleRobot+AngleStep;
end