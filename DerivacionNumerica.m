function dx = DerivacionNumerica(x,t)

n = length(t);
dx = zeros(n,6);
dt = t(n)/n;
dx(1,:) = (x(2,:) - x(1,:))/dt;
for i=2:n-1
    dx(i,:) = (x(i+1,:) - x(i-1,:))/(2*dt); 
end
dx(n,:) = (x(n,:) - x(n-1,:))/dt;
end