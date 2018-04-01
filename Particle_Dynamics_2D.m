%Particle Soup (2D) Dynamics Function
%Leif Wesche

function [x, y, vx, vy] = Particle_Dynamics_2D(m, q, x, y, vx, vy, xy0, q0, i, dt)

n=length(m);

F=zeros(n,2);
    
    for j=[1:n]
        %Calculate Force from Walls
        for p=[1:length(xy0)]
            r=[(x(j)-xy0(1,p)), (y(j)-xy0(2,p))];         
            F(j,:)=F(j,:) +  (q(j)*q0)/(norm(r)^3) * r;
        end
        k=[1:n];
        k(j)=[];
        %Calculate Force from Particles
        for k=k
            r=[(x(j)-x(k)), (y(j)-y(k))];
            F(j,:)=F(j,:) +  (q(j)*q(k))/(norm(r)^3) * r;
        end
        
        %Calculate Acceleration, Velocity, Position
        a(1,:)=F(:,1)./m';
        a(2,:)=F(:,2)./m';
        
        vxy=[vx; vy];
        vxy=vxy+a.*dt;
        vx=vxy(1,:);  vy=vxy(2,:);
        
        xy=[x; y];
        xy=xy+vxy.*dt;
        x=xy(1,:);  y=xy(2,:);
        
    end
end
