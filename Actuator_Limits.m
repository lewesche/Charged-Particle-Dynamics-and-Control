%Control Actuator Limits Function
%Leif Wesche

function [q0] = Actuator_Limits(q0, q0_max)
for i=1:length(q0)
    if q0(i) >= q0_max
        q0(i)=q0_max;
    else if q0(i) <= -q0_max
        q0(i)=-q0_max;
        end
    end
end
end