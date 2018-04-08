%Particle Soup (3D)
%Leif Wesche

clear all
close all
clc
addpath('C:\Users\ASUS\Desktop\Github\Particle Soup')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.02;        %Set Time Step (seconds)
run_time=50;    %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[0.2, 0.5];        %Particle Mass
qi=[3, 5];       %Charge (Positive Only)
q_variance=0;         %Max Random Charge Variance
xi=[2, -2];          %Initial X Position
yi=[0, 0];           %Initial Y Position
zi=[0.2, 0];          %Initial Z Position
vxi=[-2, 2];          %Initial X Velocity
vyi=[0, 0];          %Initial Y Velocity
vzi=[0, 0];          %Initial Z Velocity

q_box=400;       %Total Distributed Charge of Borders
res=20;         %Total Number of Discrete Border Points

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q_box=q_box/4; res=res/4;
q0=q_box/res;
%Bottom Bounds
dum=[linspace(-10, 10, res)];
x0b=[];
for i=1:res
x0b=[x0b, dum];
end
y0b=[];
for i=1:res
y0b=[y0b, dum(i)*ones(1,res)];
end
z0b=-10*ones(1,res^2);
%Left Right Side Bounds
x0s1=[-10*ones(1,res^2), 10*ones(1,res^2)];
y0s1=[];
for i=1:res
y0s1=[y0s1, dum];
end
y0s1=[y0s1, y0s1];
z0s1=[];
for i=1:res
z0s1=[z0s1, dum(i)*ones(1,res)];
end
z0s1=[z0s1, z0s1];
%Front Back Side Bounds
x0s2=[];
for i=1:res
x0s2=[x0s2, dum];
end
x0s2=[x0s2, x0s2];
y0s2=[-10*ones(1,res^2), 10*ones(1,res^2)];
z0s2=[];
for i=1:res
z0s2=[z0s2, dum(i)*ones(1,res)];
end
z0s2=[z0s2, z0s2];
%Top Bounds
x0t=[];
for i=1:res
x0t=[x0t, dum];
end
y0t=[];
for i=1:res
y0t=[y0t, dum(i)*ones(1,res)];
end
z0t=10*ones(1,res^2);
%Combine Bounds
x0=[x0b, x0s1, x0s2, x0t]; y0=[y0b, y0s1, y0s2, y0t]; z0=[z0b, z0s1, z0s2, z0t]; 
xyz0=[x0; y0; z0];
clearvars x0b x0s1 x0s2 x0t y0b y0s1 y0s2 y0t z0b z0s1 z0s2 z0t

qmax=[qi, q0]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, q0]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

n=length(m);

x=xi;
y=yi;
z=zi;
xyz=[x; y; z];
vx=vxi;
vy=vyi;
vz=vzi;
v=[vx; vy; vz];

for i=t
    %Apply Charge Variance
    if mod(i,0.25) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    F=zeros(n,3);
    
    for j=[1:n]
        %Calculate Force from Boundaries
        for p=[1:length(x0)]
            r=[(x(j)-x0(p)), (y(j)-y0(p)), (z(j)-z0(p))];         
            F(j,:)=F(j,:) +  (q(j)*q0)/(norm(r)^3) * r;
        end
        k=[1:n];
        k(j)=[];
        %Calculate Force from Particles
        for k=k
            r=[(x(j)-x(k)), (y(j)-y(k)), (z(j)-z(k))];
            F(j,:)=F(j,:) +  (q(j)*q(k))/(norm(r)^3) * r;
        end

        %Calculate Acceleration, Velocity, Position
        a(1,:)=F(:,1)./m';
        a(2,:)=F(:,2)./m';
        a(3,:)=F(:,3)./m';
        v=v+a.*dt;
        xyz=xyz+v.*dt;
        x=xyz(1,:);
        y=xyz(2,:); 
        z=xyz(3,:); 
    end

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    scatter3(x0, y0, z0, 'o', 'markeredgecolor', [.01*q0/qmax .88*q0/qmax 1*q0/qmax], 'markerfacecolor', [.01*q0/qmax .88*q0/qmax 1*q0/qmax], 'linewidth', 3); hold on
    axis([-10, 10, -10, 10, -10, 10])
    for i=[1:n]  
    scatter3(x(i), y(i), z(i), 'o', 'markeredgecolor',  [.01*q(i)/qmax .95*q(i)/qmax 1*q(i)/qmax], 'markerfacecolor', [.01*q(i)/qmax .95*q(i)/qmax 1*q(i)/qmax], 'linewidth', (m(i)*30/mmax)); hold on
    end
    grid on
    title('Particle Soup (2D) by Leif Wesche')
    hold off
    pause(dt)
    
end


%scatter3(x0, y0, z0)
