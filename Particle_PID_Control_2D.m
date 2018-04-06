%Particle Soup (2D) Control
%Leif Wesche

clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333/2;        %Set Time Step (seconds)
run_time=20;         %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1];              %Particle Mass
qi=[5];             %Charge (Positive Only)
q_variance=0.5;     %Max Random Charge Variance
xi=[0];             %Initial X Position
yi=[0];             %Initial Y Position
vxi=[0];            %Initial X Velocity
vyi=[0];           %Initial Y Velocity

q0_max=20;          %Total Distributed Charge of Borders
res=40;             %Total Number of Discrete Border Points
b=6;

x_desired=[-1];     %Desired X Position
y_desired=[2];      %Desired Y Position
Kp=5;               %PID Proportional Gain
Kd=1.5;               %PID Derivative Gain
Ki=1;             %PID Integral Gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

res=res/4;
x0=[linspace(-b, b, res), b*ones(1,res), linspace(b, -b, res), -b*ones(1,res)];
y0=[-b*ones(1,res), linspace(-b, b, res), b*ones(1,res), linspace(b, -b, res)];
xy0=[x0; y0];

qmax=[qi, q0_max]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, q0_max]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

n=length(m);

x=xi;
y=yi;
xy=[x; y];
vx=vxi;
vy=vyi;
v=[vx; vy];
E=[x-x_desired; y-y_desired];
x_desired_prev=[0]; y_desired_prev=[0];         v_desired=[0,0];
F=[];
fig=figure;
set(gcf, 'Position', [0, 0, 1920, 1080]);
for i=t
    tc=round(i);
    %Calculate Desired Position
    if mod(i/dt, 2*60) == 0
        x_desired=randi([-b+1, b-1]);
        y_desired=randi([-b+1, b-1]);
    end
    
    %Apply Charge Variance
    if mod(i,0.25) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    %Calculate Error
    e=[x-x_desired; y-y_desired];
    E=[E, e];
    %Calculate Velocity of Desired Position
    v_desired(1)=diff([x_desired_prev, x_desired])/dt;
    v_desired(2)=diff([y_desired_prev, y_desired])/dt; 
    x_desired_prev=x_desired; y_desired_prev=y_desired; 
    
    
    %Implement PID Control Law
    if i==0
        int=[0, 0];
    else
    if i/dt <= 20
        int(1)=Ki*trapz([0:dt:i],E(1, 2:end));
        int(2)=Ki*trapz([0:dt:i],E(2, 2:end));
    else
        int(1)=Ki*trapz([(i-20*dt):dt:i],E(1, end-20:end));
        int(2)=Ki*trapz([(i-20*dt):dt:i],E(2, end-20:end));
    end
    end
    pid(1)=Kp*e(1) + Kd*(vx-v_desired(1)) +int(1);
    pid(2)=Kp*e(2) + Kd*(vy-v_desired(2)) +int(2);
        
    qr=pid(1)*(x-b)^2;  ql=-pid(1)*(x+b)^2; qt=pid(2)*(y-b)^2;  qb=-pid(2)*(y+b)^2;
    qr=ones(1, res)*qr; ql=ones(1, res)*ql; qt=ones(1, res)*qt; qb=ones(1, res)*qb;
    q0=[qb, qr, qt, ql];
    
    %Limit Actuator Output
    q0=Actuator_Limits(q0, q0_max);
    
    %Turn Off Actuators at Desired Position
    if abs(mean(q0)) <=0.01 && abs(e(1))<=0.004 && abs(e(2))<0.004
        q0=zeros(size(q0)); e(1)=0; e(2)=0;
    end
    
    [x, y, vx, vy] = Particle_Dynamics_2D(m, q, x, y, vx, vy, xy0, q0, i, dt);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    for i=[1:length(q0)]
    if  q0(i)>=0  
       plot(x0(i), y0(i), 'o', 'markeredgecolor', ([.01 .88 1]*q0(i)/qmax).^(1/4), 'markerfacecolor', ([.01 .88 1]*q0(i)/qmax).^(1/4), 'linewidth', 3); hold on
    else 
       plot(x0(i), y0(i), 'o', 'markeredgecolor', (abs([1 0.01 0.01]*q0(i)/qmax)).^(1/4), 'markerfacecolor', (abs([1 0.01 0.01]*q0(i)/qmax)).^(1/4), 'linewidth', 3); hold on
    end
    end
    plot(x_desired, y_desired, 'kx', 'linewidth', 3)
    axis([-10, 10, -10, 10])
    for i=[1:n]  
        plot(x(i), y(i), 'o', 'markeredgecolor',  ([.01 .95 1]*q(i)/qmax).^(1/4), 'markerfacecolor', ([.01 .95 1]*q(i)/qmax).^(1/4), 'linewidth', (m(i)*10/mmax)); hold on
    end
    axis([-b, b, -b, b]) 
    grid on
    title(['Particle Soup (2D)      ', '(X,Y)_{Desired}=(', num2str([x_desired, y_desired]), ')      E_{xy}=(', num2str([round(e(1),2), round(e(2),2)]), ')      Time Elapsed:' num2str(tc), 'Sec'])
    hold off
    pause(dt)
   F=[F, getframe(fig)];    
end

v=VideoWriter('Particles_Controlled_2D_5.avi','Uncompressed AVI');
v.FrameRate = 60;
open(v)
writeVideo(v,F)
close(v)

