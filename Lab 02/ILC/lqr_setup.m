A = ...
[0 0 1.0000 0
0 0 0 1.0000
0 149.2751 -0.0104 0
0 261.6091 -0.0103 0];

B = ...
[0
0
49.7275
49.1493];


Q = diag([5,20,0.05,0.05]);
R = 0.1
lqr_controller = lqr(A,B,Q,R)


% Differential observer design
s = tf('s');
w_pass = 50;
obs = [1 0;
       0 1;
       w_pass*s/(s + w_pass) 0;
       0     w_pass*s/(s + w_pass)];
     
   
%{
% LQG Stuff
C = [1 0 0 0 ;
     0 1 0 0];
D = [0;0];
QXU = diag([50 5 .1 .1 5]);
QWV = diag([0.1 0.1 0.2 0.2 1 1]);

lqg_controller = lqg(ss(A,B,C,D),QXU,QWV);

%}