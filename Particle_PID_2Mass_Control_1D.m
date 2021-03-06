%Particle Soup (1D) Control Test2
%Leif Wesche

clear all
close all
clc
addpath('C:\Users\ASUS\Desktop\Github\Particle Soup')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333;            %Set Time Step (seconds)
run_time=10;        %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[2, 2];              %Particle Mass
qi=[5, 5];             %Charge (Positive Only)
q_variance=0.5;     %Max Random Charge Variance
xi=[1.8, -2];            %Initial X Position
vxi=[0, 0];           %Initial X Velocity

qlc=10;             %Left Bounary Controlled Charge
qrc=10;             %Right Boundary Controlled Charge
qlf=10;             %Left Boundary Fixed Charge
qrf=10;             %Right Boundary Fixed Charge

b=6;               %Boundary Size

x_desired=[1, -1];        %Desired Particle Position
Kpi=4;              %PID Proportional Gain
Kd=4;               %PID Derivative Gain
Ki=0.8;             %PID Integral Gain

scale=1;            %Scale Kp after a certain time to help "unstick" particles? (1=yes, 0=no)
t_scale=4;          %Time to turn on Kp scaling

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

qmax=[qi, qlc, qrc, qlf, qrf]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, qlc, qrc, qlf, qrf]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

xc=xi;
xf=xi;
vxc=vxi;
vxf=vxi;
Kp=Kpi;

n=length(m);
E=[xc(1)-x_desired(1); xc(2)-x_desired(2)];
F=[];
fig=figure;
set(gcf, 'Position', [0, 0, 1920, 1080]);
for i=t
    tc=round(i);
    %Apply Charge Variance
    if mod(i,0.5) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    %Calculate Error
    for j=[1:n]
    e(j,:)=xc(j)-x_desired(j);
    end
    E=[E, e];

    %Implement PID Control Law
    for j=[1:n]
    if i==0
        int(j)=0;
    else
    if i/dt <= 80
        int(j)=Ki*trapz([0:dt:i],E(j,2:end));
    else
        int(j)=Ki*trapz([(i-80*dt):dt:i],E(j,end-80:end));
    end
    end
    
    %Ramp up Kp based on Integral output 
    if i >= t_scale
        Kp=Kp+Kpi*(mean(abs(int(1))+abs(int(2)))).^(2)*scale; 
    end  
    
    pid(j)=Kp*e(j) + Kd*vxc(j) +int(j);
    end
    qrc=pid(1)*(xc(1)-b/2)^2;
    qlc=-pid(2)*(xc(2)+b/2)^2;
    %Limit Control Outputs
    if qrc >= 10
        qrc=10;
    elseif qrc <= - 10
        qrc=-10;
    end
    if qlc >= 10
        qlc=10;
    elseif qlc <= - 10
        qlc=-10;
    end
    %Turn Off Actuators at Desired Position - Wont Turn Off For Multi-Mass!
%     if abs(qlc) <=0.1 && abs(qrc)<= 0.1 && e(1)<=0.01 && e(2)<=0.01
%         qlc=0;  qrc=0; e=0;
%     end
   
   
    %Calculate Dynamics - Controlled Particle
    [xc, vxc, KEc, PEc, TEc]=Particle_Dynamics_1D(m, q, xc, vxc, b, qlc, qrc, i, dt);
    %Calculate Dynamics - Contained Particle
    [xf, vxf, KEf, PEf, TEf]=Particle_Dynamics_1D(m, q, xf, vxf, b, qlf, qrf, i, dt);
    
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
   
   subplot(2,1,1)
   if qlf >= 0
   plot(-b/2, 0, 'o', 'markeredgecolor', ([.01 .88 1]*qlf/qmax).^(1/4), 'markerfacecolor', ([.01 .88 1]*qlf/qmax).^(1/4), 'linewidth', 4); hold on
   else
   plot(-b/2, 0, 'o', 'markeredgecolor', (abs([1 0.01 0.01]*qlf/qmax)).^(1/4), 'markerfacecolor', (abs([1 0.01 0.01]*qlf/qmax)).^(1/4), 'linewidth', 4); hold on
   end
   if qrf >= 0
   plot(b/2, 0, 'o', 'markeredgecolor', ([.01 .88 1]*qrf/qmax).^(1/4), 'markerfacecolor', ([.01 .88 1]*qrf/qmax).^(1/4), 'linewidth', 4); hold on
   else
   plot(b/2, 0, 'o', 'markeredgecolor', (abs([1 0.01 0.01]*qrf/qmax)).^(1/4), 'markerfacecolor', (abs([1 0.01 0.01]*qrf/qmax)).^(1/4), 'linewidth', 4); hold on
   end 
   axis([-b/2, b/2, -1, 1])  
   for i=[1:n]  
   plot(xf(i), 0, 'o', 'markeredgecolor',  ([.01 .88 1]*q(i)/qmax).^(1/4), 'markerfacecolor', ([.01 .88 1]*q(i)/qmax).^(1/4), 'linewidth', (m(i)*15/mmax)); hold on
   end
   title(['Fixed Charges        Q_{L} =', num2str(qlf), '       Q_{R} =', num2str(qrf), '      Time Elapsed:' num2str(tc), 'Sec'])
   grid on
   hold off
   
   subplot(2,1,2)
   if qlc >= 0
   plot(-b/2, 0, 'o', 'markeredgecolor', ([.01 .88 1]*qlc/qmax).^(1/4), 'markerfacecolor', ([.01 .88 1]*qlc/qmax).^(1/4), 'linewidth', 4); hold on
   else
   plot(-b/2, 0, 'o', 'markeredgecolor', (abs([1 0.01 0.01]*qlc/qmax)).^(1/4), 'markerfacecolor', (abs([1 0.01 0.01]*qlc/qmax)).^(1/4), 'linewidth', 4); hold on
   end
   if qrc >= 0
   plot(b/2, 0, 'o', 'markeredgecolor', ([.01 .88 1]*qrc/qmax).^(1/4) , 'markerfacecolor', ([.01 .88 1]*qrc/qmax).^(1/4), 'linewidth', 4); hold on
   else
   plot(b/2, 0, 'o', 'markeredgecolor', (abs([1 0.01 0.01]*qrc/qmax)).^(1/4), 'markerfacecolor', (abs([1 0.01 0.01]*qrc/qmax)).^(1/4), 'linewidth', 4); hold on
   end 
   axis([-b/2, b/2, -1, 1])  
   plot(x_desired, [0, 0], 'kx', 'linewidth', 3)
   for i=[1:n]  
   plot(xc(i), 0, 'o', 'markeredgecolor',  ([.01 .88 1]*q(i)/qmax).^(1/4), 'markerfacecolor', ([.01 .88 1]*q(i)/qmax).^(1/4), 'linewidth', (m(i)*15/mmax)); hold on
   end
   grid on
   title(['Dynamic Charges        Q_{L} =', num2str(qlc), '       Q_{R} =', num2str(qrc), '         X_{Desired}=', num2str(flip(x_desired)), '        Error=', num2str(round(flip(e.'),2)), '      Kp=', num2str(Kp)])
   hold off
   pause(dt)
%    F=[F, getframe(fig)];
end

% v=VideoWriter('Particles_Controlled_2Mass_1D_4.avi','Uncompressed AVI');
% open(v)
% writeVideo(v,F)
% close(v)
