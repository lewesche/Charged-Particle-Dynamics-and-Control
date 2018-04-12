%Particle Soup (1D) Dynamics Function
%Leif Wesche

function [x, vx, KE, PE, E] = Particle_Dynamics_1D(m, q, x, vx, b, ql, qr, i, dt)

n=length(m);
PE=0;

    for j=[1:n]
        %Calculate Force From Boundaries
        rl=[(x(j)+b/2)];    rr=[(x(j)-b/2)];
        F(j)=(q(j)*ql)/(norm(rl)^3)*rl + (q(j)*qr)/(norm(rr)^3)*rr; 
        PE=PE + (ql*q(j)/abs(rl)) + (qr*q(j)/abs(rr));
        k=[1:n];
        k(j)=[];
        %Calculate Force from Particles
        for k=k
            r=[(x(j)-x(k))];
            F(j)=F(j) + (q(j)*q(k))/(norm(r)^3)*r;
            PE=PE + (q(j)*q(k)/abs(r));
        end
    end
    for j=[1:n]
        a(j)=F(j)/m(j);
        vx(j)=vx(j)+a(j)*dt*(1-12*(n-1)/100);
        KE=round(sum(0.5.*m.*abs(vx.^2)), 1);
        PE=round(PE, 1);
        E=PE+KE;
        
        x(j)=x(j)+vx(j)*dt;
    end
end