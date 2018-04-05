%Particle Soup (2D) Control
%Leif Wesche

clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.01;        %Set Time Step (seconds)
run_time=25;     %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1];              %Particle Mass
qi=[3];             %Charge (Positive Only)
q_variance=0.5;       %Max Random Charge Variance
xi=[1];             %Initial X Position
yi=[3];             %Initial Y Position
vxi=[8];            %Initial X Velocity
vyi=[8];            %Initial Y Velocity

qc_max=12;                %Total Distributed Charge of Borders
res=52;             %Total Number of Discrete Border Points
b=4;

x_desired=[0];      %Desired X Position
y_desired=[0];      %Desired Y Position
Kp=1;               %PID Proportional Gain
Kd=0;               %PID Derivative Gain
Ki=0;             %PID Integral Gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

res=res/4;
x0=[linspace(-b, b, res), b*ones(1,res), linspace(b, -b, res), -b*ones(1,res)];
y0=[-b*ones(1,res), linspace(-b, b, res), b*ones(1,res), linspace(b, -b, res)];
xy0=[x0; y0];

qmax=[qi, qc_max]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, qc_max]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

n=length(m);

x=xi;
y=yi;
xy=[x; y];
vx=vxi;
vy=vyi;
v=[vx; vy];
E=[x-x_desired; y-y_desired];
F=[];
fig=figure;
set(gcf, 'Position', [0, 0, 800, 400]);
for i=t
    tc=round(i);
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
    
    %Implement PID Control Law
    if i==0
        int=[0, 0];
    else
    if i/dt <= 50
        int(1)=Ki*trapz([0:dt:i],E(1, 2:end));
        int(2)=Ki*trapz([0:dt:i],E(2, 2:end));
    else
        int(1)=Ki*trapz([(i-50*dt):dt:i],E(1, end-50:end));
        int(2)=Ki*trapz([(i-50*dt):dt:i],E(2, end-50:end));
    end
    end
    pid(1)=Kp*e(1) + Kd*vx +int(1);
    pid(2)=Kp*e(2) + Kd*vy +int(2);
        
    qr=pid(1)*(x-b/2)^2;
    ql=-pid(1)*(x+b/2)^2;
    qt=pid(2)*(y-b/2)^2;
    qb=-pid(1)*(y+b/2)^2;
    %Limit Control Outputs
    if qr >= qc_max %Right Limit
        qr=qc_max;
    elseif qr <= -qc_max
        qr=-qc_max;
    end
    if ql >= qc_max %Left Limit
        ql=qc_max;
    elseif ql <= -qc_max
        ql=-qc_max;
    end
    
    if qt >= qc_max %Top Limit
        qt=qc_max;
    elseif qt <= -qc_max
        qt=-qc_max;
    end
    if qb >= qc_max %Bottom Limit
        qb=qc_max;
    elseif qb <= -qc_max
        qb=-qc_max;
    end
      q0=[ql, qr, qt, qb]
    
    qr=ones(1, res)*qr;
    ql=ones(1, res)*ql;
    qt=ones(1, res)*qt;
    qb=ones(1, res)*qb;
    
    q0=[ql, qr, qt, qb];
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
    axis([-10, 10, -10, 10])
    for i=[1:n]  
    plot(x(i), y(i), 'o', 'markeredgecolor',  ([.01 .95 1]*q(i)/qmax).^(1/4), 'markerfacecolor', ([.01 .95 1]*q(i)/qmax).^(1/4), 'linewidth', (m(i)*10/mmax)); hold on
    end
    grid on
    title(['Particle Soup (2D)      ', 'Time Elapsed:' num2str(tc), 'Sec'])
    hold off
    pause(dt)
    
end
