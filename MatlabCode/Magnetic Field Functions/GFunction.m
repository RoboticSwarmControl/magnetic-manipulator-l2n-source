function [ Fvalue ] = GFunction( coil_number,P,a ,T)

%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018


%This function computes the magnetic field produced by a single current
%loop in a cylindrical system. The calculation uses equations presented in
%:Simple Analytic Expressions for the Magnetic Field of a Circular Current
%Loop, Simpson, James C.; Lane, John E.; Immer, Christopher D.; Youngquist,
%Robert C, NASA/TM-2013-21791.

%a: radius of the current loop [m]
%I: Current in the circular loop [A]
%M: Vector containing the coordinates (r,theta,z) of the calculation point in the
%cylindrical system. ([m],[rad],[m])

%B: Magnetic flux density vector (Br,Btheta,Bz) in the cylindrical coordinate system [T].


X=P(1);
Y=P(2);

%convert the point location from the manipulator coordinate system to the
%coil coordinate system

if (coil_number==1)
    R=abs(X);
    Z=Y+T/2;
end

if (coil_number==2)
    R=abs(Y);
    Z=T/2-X;
end

if (coil_number==3)
    R=abs(X);
    Z=T/2-Y;
end

if (coil_number==4)
    R=abs(Y);
    Z=X+T/2;
end

mu0=4*pi()*1e-7; %vacuum permeability 

%R=M(1) %radius position of the calculation point
%Z=M(3) %Z position of the calculation points

%//////////////////////////////////////////////////////////
%////// Here starts the calculation of the magnetic field using the
%equations of the paper.
k=(4.*a.*R)./((a+R).*(a+R)+Z.*Z);
[I1,I2] = ellipke(k,10000000);
Br=((mu0)./(2.*pi.*R)).*(Z./(sqrt((a+R).*(a+R)+Z.*Z))).*(-I1+((a.*a+R.*R+Z.*Z)./((a-R).*(a-R)+Z.*Z)).*I2);
Bz=((mu0)./(2.*pi)).*(1./(sqrt((a+R).*(a+R)+Z.*Z))).*(I1+((a.*a-R.*R-Z.*Z)./((a-R).*(a-R)+Z.*Z)).*I2);
%End of the calculation of the magnetic field using the equations of the paper.
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

%There is cases where the result is NAN
%    at a=r and z=0: This point has no physical sens since the considered
%    wire is of infinitly small diameter
%    at r=0: The equations for Br returns NAN but the result is 0

%The following fix this problem:
% if isnan(Br)
%     Br=0;
% end
% if isnan(Bz)
%     Bz=0;
% end

if R==0
    Br=0;
end

if abs(a-R)<(a./100) && abs(Z)<(a./100)
    Br=0;
    Bz=0;
end


%rotate and return the result
if coil_number==1
    if X>0
        Fvalue=[Br,Bz];
    else
        Fvalue=[-Br,Bz];
    end
end

if coil_number==2
    if Y>0
        Fvalue=[-Bz,Br];
    else
        Fvalue=[-Bz,-Br];
    end
end

if coil_number==3
    if X>0
        Fvalue=[Br,-Bz];
    else
        Fvalue=[-Br,-Bz];
    end
end

if coil_number==4
    if Y>0
        Fvalue=[Bz,Br];
    else
        Fvalue=[Bz,-Br];
    end
end

end

