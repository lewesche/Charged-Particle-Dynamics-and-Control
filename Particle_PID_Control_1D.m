%Particle Soup (1D) Control Test2
%Leif Wesche

clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.01;            %Set Time Step (seconds)
run_time=20;        %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1];              %Particle Mass
qi=[5];             %Charge (Positive Only)
q_variance=0.5;     %Max Random Charge Variance
xi=[-1];             %Initial X Position

qlc=10;              %Left Bounary Controlled Charge
qrc=10;              %Right Boundary Controlled Charge
qlf=10;              %Left Boundary Fixed Charge
qrf=10;              %Right Boundary Fixed Charge

b=10;               %Boundary Size

x_desired=4.9;  %Desired Particle Position
Kp=4;         %PID Proportional Gain
Kd=2;       %PID Derivative Gain
Ki=1;        %PID Integral Gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

qmax=[qi, qlc, qrc, qlf, qrf]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, qlc, qrc, qlf, qrf]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

xc=xi;
xf=xi;
vxc=vxi;
vxf=vxi;

n=length(m);
E=[];

for i=t
    %Apply Charge Variance
    if mod(i,0.25) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    %Implement Control Law
    e=xc-x_desired;
    pid=Kp*e + Kd*vxc;
    qrc=pid*(xc-b/2)^2;
    qlc=-pid*(xc+b/2)^2;
   
    if qrc >= 10
        qrc=10;
    elseif qrc <= - 10
        qrc=-10;
    end
    if qlc >= 10
        qlc=10;
    elseif qlc <= - 10
        qlc=-10
    end
    
    %Calculate Dynamics - Controlled Particle
    [xc, vxc]=Particle_Dynamics_1D(m, q, xc, vxc, b, qlc, qrc, i, dt);
    %Calculate Dynamics - Contained Particle
    [xf, vxf]=Particle_Dynamics_1D(m, q, xf, vxf, b, qlf, qrf, i, dt);
    
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
   title(['Fixed Charges        Q_{L} =', num2str(qlf), '       Q_{R} =', num2str(qrf)])
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
   for i=[1:n]  
   plot(xc(i), 0, 'o', 'markeredgecolor',  ([.01 .88 1]*q(i)/qmax).^(1/4), 'markerfacecolor', ([.01 .88 1]*q(i)/qmax).^(1/4), 'linewidth', (m(i)*15/mmax)); hold on
   end
   grid on
   title(['Dynamic Charges        Q_{L} =', num2str(qlc), '       Q_{R} =', num2str(qrc), '         X_{Desired}=', num2str(x_desired)])
   hold off
   pause(dt)
end
