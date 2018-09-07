n_steps = 200;
t_max = 20;
T = t_max / n_steps;

x = zeros(4,n_steps);
x_est = zeros(4,n_steps);
y = zeros(2,n_steps);
u = zeros(2,n_steps);
u(2,:) = 5*sin(linspace(0,4*pi,n_steps));

A_d = expm(A*T);
B_d = integral(@(t)expm(A*t), 0, T,'ArrayValued',true)*B;
C_d = C;
D_d = D;

u_sat = max(min(u,25),-25);

x(:,1) = [.5 ; 0 ; .5 ; 1];

for i = 1:n_steps-1
    x(:,i+1) = A_d*x(:,i) + B_d*u(:,i);
    y(:,i)   = C_d*x(:,i) + D_d*u(:,i);
    x_est(1:2,i+1) = x(1:2,i+1);
    x_est(3:4,i+1) = (x(3:4,i+1)-x(3:4,i))/T;
end
    