function [x,del,time,Xeq,xseq,X_optdel,X_opt,Y_opt,Z_opt,U_opt,V_opt,W_opt,inert_val] = optimal_control_40_full_rel(model_time,time_step,Afull,Bfull,trim_val,us0)

%x,del,time,Xeq,xseq,Z0_opt,U0_opt,W0_opt,X_opt,Z_opt,U_opt,W_opt,X_optdel,Cfull,U,W


u0 = trim_val(1);
v0 = trim_val(2);
w0 = trim_val(3);
phi0 = trim_val(4);
theta0 = trim_val(5);
psi0 = trim_val(6);


tf=model_time;
dt=time_step;
time=(xlsread('Heli_Full_40KIAS_Optimal_Relative.xlsx','Medium_40s','A2:A401')).';


%Final t=40s all states
U_max = 0.1;
V_max = 1.5;
W_max = 0.1;
X_max = 25000;
Y_max = 1000;
Z_max = 10;
delb_max = 1;
delc_max = 1;
dela_max = 1;
delp_max = 0.5;

%{
U_max = 0.1;
V_max = 1;
W_max = 0.1;
X_max = 25000;
Y_max = 1000;
Z_max = 10;
delb_max = 1;
delc_max = 1;
dela_max = 0.25;
delp_max = 0.01;
%}

Xeq=zeros(1,size(time,2));
xseq=zeros(1,size(time,2));

Snew=zeros(12,size(time,2)*12);
g=zeros(12,size(time,2));

for i=1:size(time,2)
    Xeq(i)=u0*time(i);
    xseq(i)=us0*time(i);
end

X_opt_rel=xlsread('Heli_Full_40KIAS_Optimal_Relative.xlsx','Medium_40s','B2:B401');
X_opt = X_opt_rel + xseq.';
X0_opt=X_opt(1);
X_opt=X_opt-X0_opt;
scale=X_opt(end)/(X_opt(end)-Xeq(end));
X_optdel=X_opt/scale;
U_opt_rel=xlsread('Heli_Full_40KIAS_Optimal_Relative.xlsx','Medium_40s','C2:C401');
U_opt = U_opt_rel + us0;
U0_opt=U_opt(1);
U_opt=U_opt-U0_opt;
Y_opt_rel=xlsread('Heli_Full_40KIAS_Optimal_Relative.xlsx','Medium_40s','D2:D401');
Y0_opt=Y_opt_rel(1);
Y_opt=Y_opt_rel-Y0_opt;
V_opt_rel=xlsread('Heli_Full_40KIAS_Optimal_Relative.xlsx','Medium_40s','E2:E401');
V0_opt=V_opt_rel(1);
V_opt=V_opt_rel-V0_opt;
Z_opt_rel=xlsread('Heli_Full_40KIAS_Optimal_Relative.xlsx','Medium_40s','F2:F401');
Z0_opt=Z_opt_rel(1);
Z_opt=Z_opt_rel-Z0_opt;
W_opt_rel=xlsread('Heli_Full_40KIAS_Optimal_Relative.xlsx','Medium_40s','G2:G401');
W0_opt=W_opt_rel(1);
W_opt=W_opt_rel-W0_opt;

inert_val = [U0_opt; V0_opt; W0_opt; X0_opt; Y0_opt; Z0_opt];

Q = eye(6);
Q(1,1) = 1/(tf*U_max);
Q(2,2) = 1/(tf*V_max);
Q(3,3) = 1/(tf*W_max);
Q(4,4) = 1/(tf*X_max);
Q(5,5) = 1/(tf*Y_max);
Q(6,6) = 1/(tf*Z_max);

R = eye(4);
R(1,1) = 1/(tf*delb_max);
R(2,2) = 1/(tf*dela_max);
R(3,3) = 1/(tf*delp_max);
R(4,4) = 1/(tf*delc_max);

Cfull = [0,0,0,-9.1547e-06,0.0016,0.0002,0.9988,-0.0009,0.0493,0,0,0;
    0,0,0,3.3324,0,-67.5899,0,-0.9998,-0.0175,0,0,0;
    0,0,0,0.0002,67.5899,0,0.0493,0.0175,-0.9986,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    0,1,0,0,0,0,0,0,0,0,0,0;
    0,0,1,0,0,0,0,0,0,0,0,0];


S0 = zeros(12);  % Some arbitrary matrix initial value
odefun = @(t,y) deriv(t,y,Afull,Bfull,R,Q,Cfull);  % Anonymous derivative function
tspan = tf:-0.10025:0;
[~,S] = ode45(odefun,tspan,S0(:));  % Pass in column vector initial value
S = reshape(S.',12,12,[]);


k=size(Snew,2)-11;
for i=(size(time,2)-1):-1:1
    Snew(1:12,k-12:k-1) = S(:,:,(size(time,2)-i+1));
    z = [U_opt(i+1);V_opt(i+1);W_opt(i+1);X_optdel(i+1);Y_opt(i+1);Z_opt(i+1)];
    g(1:12,i) = g(1:12,i+1) - (time(i+1)-time(i))*(-(Afull.' - Snew(1:12,k:k+11)*Bfull*(R\(Bfull.')))*g(1:12,i+1) + Cfull.'*Q*z);
    k=k-12;
end

U = zeros(size(time));
V = zeros(size(time));
W = zeros(size(time));
x = zeros(12,size(time,2));
del = zeros(4,size(time,2));
U(1) = cos(theta0)*cos(psi0)*u0 + (sin(phi0)*sin(theta0)*cos(psi0) - cos(phi0)*sin(psi0))*v0 + (cos(phi0)*sin(theta0)*cos(psi0) + sin(phi0)*sin(psi0))*w0;
V(1) = -(cos(theta0)*sin(psi0)*u0 + (sin(phi0)*sin(theta0)*sin(psi0) + cos(phi0)*cos(psi0))*v0 + (cos(phi0)*sin(theta0)*sin(psi0) - sin(phi0)*cos(psi0))*w0);
W(1) = -(-sin(theta0)*u0 + sin(phi0)*cos(theta0)*v0 + cos(phi0)*cos(theta0)*w0);


k=1;
for j=1:size(time,2)-1
    del(1:4,j) = -(R\(Bfull.'))*Snew(1:12,k:k+11)*x(1:12,j) - (R\(Bfull.'))*g(1:12,j);
    x(1:12,j+1) = x(1:12,j) + (time(j+1)-time(j))*(Afull*x(1:12,j) + Bfull*del(1:4,j));
    U(1,j+1) = cos(theta0+x(5,j+1))*cos(psi0+x(6,j+1))*(u0+x(7,j+1)) + (sin(phi0+x(4,j+1))*sin(theta0+x(5,j+1))*cos(psi0+x(6,j+1)) - cos(phi0+x(4,j+1))*sin(psi0+x(6,j+1)))*(v0+x(8,j+1)) + (cos(phi0+x(4,j+1))*sin(theta0+x(5,j+1))*cos(psi0+x(6,j+1)) ...
    + sin(phi0+x(4,j+1))*sin(psi0+x(6,j+1)))*(w0+x(9,j+1));
    V(1,j+1) = -(cos(theta0+x(5,j+1))*sin(psi0+x(6,j+1))*(u0+x(7,j+1)) + (sin(phi0+x(4,j+1))*sin(theta0+x(5,j+1))*sin(psi0+x(6,j+1)) + cos(phi0+x(4,j+1))*cos(psi0+x(6,j+1)))*(v0+x(8,j+1)) + (cos(phi0+x(4,j+1))*sin(theta0+x(5,j+1))*sin(psi0+x(6,j+1)) ...
    - sin(phi0+x(4,j+1))*cos(psi0+x(6,j+1)))*(w0+x(9,j+1)));
    W(1,j+1) = -(-sin(theta0+x(5,j+1))*(u0+x(7,j+1)) + sin(phi0+x(4,j+1))*cos(theta0+x(5,j+1))*(v0+x(8,j+1)) + cos(phi0+x(4,j+1))*cos(theta0+x(5,j+1))*(w0+x(9,j+1)));
    k=k+12;
end
end

