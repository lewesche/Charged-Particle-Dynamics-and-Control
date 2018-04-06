%Particle Soup (1D) Control
%Leif Wesche

clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333;            %Set Time Step (seconds)
run_time=9;        %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1];              %Particle Mass
qi=[5];             %Charge (Positive Only)
q_variance=0.5;     %Max Random Charge Variance
xi=[4.5];           %Initial X Position
vxi=[-2];           %Initial X Velocity

qc_max=10;          %Controlled Charge Max Absolute Value
qlf=10;             %Left Boundary Fixed Charge
qrf=10;             %Right Boundary Fixed Charge

b=12;               %Boundary Size

x_desired=[0];      %Desired Particle Position
Kp=4;               %PID Proportional Gain
Kd=3;               %PID Derivative Gain
Ki=0.8;             %PID Integral Gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

qmax=[qi, qc_max, qlf, qrf]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, qc_max, qlf, qrf]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

xc=xi;
xf=xi;
vxc=vxi;
vxf=vxi;

n=length(m);
E=[xc-x_desired];
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
    e=xc-x_desired;
    E=[E, e];
    
    %Implement PID Control Law
    if i==0
        int=0;
    else
    if i/dt <= 50
        int=Ki*trapz([0:dt:i],E(2:end));
    else
        int=Ki*trapz([(i-50*dt):dt:i],E(end-50:end));
    end
    end
    pid=Kp*e + Kd*vxc +int;
    qrc=pid*(xc-b/2)^2;
    qlc=-pid*(xc+b/2)^2;
    %Limit Control Outputs
    if qrc >= qc_max
        qrc=qc_max;
    elseif qrc <= -qc_max
        qrc=-qc_max;
    end
    if qlc >= qc_max
        qlc=qc_max;
    elseif qlc <= -qc_max
        qlc=-qc_max;
    end
    %Turn Off Actuators at Desired Position
    if abs(qlc) <=0.1 && abs(qrc)<= 0.1 && e<=0.01
        qlc=0;  qrc=0; e=0;
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
   plot(x_desired, 0, 'kx', 'linewidth', 3)
   axis([-b/2, b/2, -1, 1])  
   for i=[1:n]  
   plot(xc(i), 0, 'o', 'markeredgecolor',  ([.01 .88 1]*q(i)/qmax).^(1/4), 'markerfacecolor', ([.01 .88 1]*q(i)/qmax).^(1/4), 'linewidth', (m(i)*15/mmax)); hold on
   end
   grid on
   title(['Dynamic Charges        Q_{L} =', num2str(qlc), '       Q_{R} =', num2str(qrc), '         X_{Desired}=', num2str(x_desired), '        Error=', num2str(round(e,2))])
   hold off
   pause(dt)
%    F=[F, getframe(fig)];
end

% v=VideoWriter('Particles_Controlled_1D_3.avi','Uncompressed AVI');
% open(v)
% writeVideo(v,F)
% close(v)
