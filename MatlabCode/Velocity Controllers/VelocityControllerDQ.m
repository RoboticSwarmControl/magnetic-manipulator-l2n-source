function [ FDQ , Torque, HistoryErrorVelocityOutput , HistoryErrorAngleVelocityOutput ] = VelocityControllerDQ( StateDQ, VelocitySetpointDQ,  KpFD,KpFQ, KiF,KdF, KpA,KiA,KdA);%Velocity, VelocitySetpoint , AngleVelocitySetpoint , KpF,KiF,KdF, KpA,KiA,KdA ,HistoryErrorVelocityInput, HistoryErrorAngleVelocityInput,  time, CdForceModel, CdTorqueModel )

HistoryErrorVelocityOutput=0;
HistoryErrorAngleVelocityOutput =0;
%HistoryErrorAngleVelocityOutput=0;
%First the optimum control is computed (compensation od the drag)
%The drag is assumed to be proportional to the velocity 

%OptimumControlForce=0;%CdForceModel.*VelocitySetpoint;
%OptimumControlTorque=0;%CdTorqueModel.*AngleVelocitySetpoint;

%Calculate the error on the velocity
ErrorVelocityDQ=VelocitySetpointDQ-StateDQ;




%Proportional correction of velocity
%The DQ force controller is only able to apply a force on the D axis
%Place the new error value at teh end of HistoryErrorVelocityInput vector
%HistoryErrorVelocityInput=[HistoryErrorVelocityInput,[ErrorVelocity';time]];

FDQ=[KpFD;KpFQ].*ErrorVelocityDQ;

%Remove the oldest value in HistoryErrorVelocityInput
%HistoryErrorVelocityInput=HistoryErrorVelocityInput(:,2:length(HistoryErrorVelocityInput));

%Calculate the integral of the error
%[m,n]=size(HistoryErrorVelocityInput);
%IntegralErrorVx=0;
%IntegralErrorVy=0;
%for i=n:-1:2
 %  IntegralErrorVx=IntegralErrorVx+HistoryErrorVelocityInput(1,i).*(HistoryErrorVelocityInput(3,i)-HistoryErrorVelocityInput(3,i-1)); %previous value + error*duration
  % IntegralErrorVy=IntegralErrorVy+HistoryErrorVelocityInput(2,i).*(HistoryErrorVelocityInput(3,i)-HistoryErrorVelocityInput(3,i-1));
%end
%Calculate the Output of the controller


%F=OptimumControlForce+KpF.*ErrorVelocity+KiF.*[IntegralErrorVx,IntegralErrorVy];
Torque=0.0001.*ErrorVelocityDQ(2);

%HistoryErrorVelocityOutput=HistoryErrorVelocityInput; %will need to be changed when integral component will be added

end

