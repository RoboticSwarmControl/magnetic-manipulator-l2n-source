function [ G ] = dGFunction( coil_number,P,a,AxisDerivative,T )

%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018

%Axis==1 => x
%Axis==2 => y

dl=0.0001; %delta for differenciation [m]

if AxisDerivative==1
    Gplus=GFunction( coil_number,[P(1)+dl;P(2)],a ,T);
    Gminus=GFunction( coil_number,[P(1)-dl;P(2)],a,T );
end

if AxisDerivative==2
    Gplus=GFunction( coil_number,[P(1);P(2)+dl],a,T );
    Gminus=GFunction( coil_number,[P(1);P(2)-dl],a ,T);
end



G=(Gplus-Gminus)./(2*dl);

end

