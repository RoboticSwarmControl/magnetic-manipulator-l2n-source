function [ MeasuredPosition,LastMeasureTime ] = GaussianNoiseSensing( Position, time, LastMeasureTime,LastMeasuredPosition )
%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018

%PERFECTSENSING This funtion simulate a noisy position sensing


MeasurePeriod=0.005; %[s]

if (time-LastMeasureTime)>MeasurePeriod

VarianceOfNoiseXY=0.0001; %meters
VarianceOfNoiseAngle=0.01;% [rad]

NoiseXYPosition = VarianceOfNoiseXY.*randn(1,2);
NoiseAnglePosition = VarianceOfNoiseAngle.*randn(1,1);

MeasuredPosition=Position+[NoiseXYPosition,NoiseAnglePosition];
LastMeasureTime=time;

while MeasuredPosition(3)<0
    MeasuredPosition(3)=MeasuredPosition(3)+pi; %The real controler output an angle between 0 and pi
end
else
    
    MeasuredPosition=LastMeasuredPosition;
    LastMeasureTime=LastMeasureTime;
end


end

