
tic
clc
clear
close all

%%MODEL PARAMETERS SELECTION
Afull = [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.9988, -0.0009, 0.0493, 0.0000, 0.0000, 0.0000;
    0.0000, 0.0000, 0.0000, 3.3325, 0.0000, -67.5899, 0.0000, -0.9998, -0.0175, 0.0000, 0.0000, 0.0000;
    0.0000, 0.0000, 0.0000, 0.0000, 67.5899, 0.0000, 0.0493, 0.0175, -0.9986, 0.0000, 0.0000, 0.0000;
    0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, -0.0009, 0.0494;
    0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.9998, 0.0175;
    0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0176, 1.0011;
    -0.0000, -0.0000, 0.0002, -1.7910, -28.7478, 0.0000, -0.0156, -0.0014, 0.0264, -1.0130, 1.1241, -0.4113;
    0.0000, 0.0000, 0.0001, 33.1424, -0.3921, -0.0000, 0.0079, -0.1015, 0.0165, -1.5512, -1.5106, -65.9068;
    -0.0000, -0.0000, -0.0033, -2.6711, 1.7629, -0.0000, -0.1352, 0.0043, -0.6177, 8.8359, 68.2588, 2.0707;
    0.0000, 0.0000, 0.0002, -0.7252, -0.6595, -0.0000, 0.0058, -0.0292, 0.0424, -7.1172, -1.6572, 0.1983;
    -0.0000, -0.0000, 0.0000, 0.0642, -0.7286, 0.0000, 0.0019, 0.0059, -0.0013, 0.0097, -1.5146, -0.0941;
    -0.0000, -0.0000, 0.0000, -0.8489, 0.0071, 0.0000, -0.0036, 0.0167, -0.0027, -0.1995, -0.0263, -0.6115];
Bfull = [0.0000, 0.0000, 0.0000, 0.0000;
    0.0000, 0.0000, 0.0000, 0.0000;
    0.0000, 0.0000, 0.0000, 0.0000;
    0.0000, 0.0000, 0.0000, 0.0000;
    0.0000, 0.0000, 0.0000, 0.0000;
    0.0000, 0.0000, 0.0000, 0.0000;
    -0.1577, -0.0046, 0.0529, 0.0601;
    0.0197, 0.0977, -0.0752, 0.0082;
    -0.1559, 0.0106, 0.0865, -0.8894;
    0.0307, 0.1185, -0.0356, 0.0067;
    0.0339, 0.0007, -0.0022, 0.0024;
    -0.0004, 0.0037, 0.0222, 0.0058];

u0 = 67.5077;       %reference velocity along body X axis, ft/sec (positive forward)
v0 = -0.0585;       %reference velocity along body Y axis, ft/sec (positive right)
w0 = 3.3319;        %reference velocity along body Z axis, ft/sec (positive down)
phi0 = -0.0175;     %reference roll attitude, rad
theta0 = 0.0493;    %reference pitch attitude, rad
psi0 = 0;           %reference yaw attitude, rad
del_b0 = 39.9557;   %trim longitudinal cyclic, percent
del_a0 = 53.1125;   %trim lateral cyclic, percent
del_p0 = 48.6692;   %trim pedal, percent
del_c0 = 34.5226;   %trim collective, percent
trim_val = [u0; v0; w0; phi0; theta0; psi0];

model_time=40;  %Enter time for vehicle to land in seconds (total simulation time)
time_step=0.1;  %Enter time step in seconds
tr=100;         %Enter number of random trajectories
band_z=5;       %final z-position error range, ft
band_y=5;       %final y-position error range, ft
band_x=5;       %final x-position error range, ft
band_w=3;     %final z-velocity error range, ft/sec
band_v=3;     %final y-velocity error range, ft/sec
band_u=3;     %final x-velocity error range, ft/sec
rb_inc=0.1;    %ddelb sensitivity increment, percentage of optimal trajectory
ra_inc=0.1;    %ddela sensitivity increment, percentage of optimal trajectory
rp_inc=0.1;    %ddelp sensitivity increment, percentage of optimal trajectory
rc_inc=0.1;     %ddelc sensitivity increment, percentage of optimal trajectory
%us0=8.14;        %Ship's average forward velocity (ft/sec), LOW level sea state
us0=33.49;        %Ship's average forward velocity (ft/sec), MEDIUM level sea state
%us0=49.98;        %Ship's average forward velocity (ft/sec), HIGH level sea state

%%SEA STATE SELECTION
%sdz=1.14;                %Standard deviation for ship's z-acceleration (ft/sec^2), LOW level sea state
sdz=2.43;                %Standard deviation for ship's z-acceleration (ft/sec^2), MEDIUM level sea state
%sdz=5.13;                %Standard deviation for ship's z-acceleration (ft/sec^2), HIGH level sea state
%sdy=0.736;                %Standard deviation for ship's y-velocity (ft/sec), MEDIUM level sea state
sdy=0.89;                %Standard deviation for ship's y-velocity (ft/sec), MEDIUM level sea state
%sdy=1.76;                %Standard deviation for ship's y-velocity (ft/sec), MEDIUM level sea state
%sdx=0.181;              %Standard deviation for ship's x-velocity (ft/sec^2), LOW level sea state
sdx=0.363;              %Standard deviation for ship's x-velocity (ft/sec^2), MEDIUM level sea state
%sdx=0.726;              %Standard deviation for ship's x-velocity (ft/sec^2), HIGH level sea state

%BVP Optimal Solution legend
%xvec(1)=X(t)       Perturbation in Vehicle forward position (X-inertial, positive forward)
%xvec(2)=Y(t)       Perturbation in Vehicle sideward position (Y-inertial, positive left)
%xvec(3)=Z(t)       Perturbation in Vehicle vertical position (Z-inertial, positive up)
%xvec(4)=phi(t)     Perturbation in Vehicle roll attitude  
%xvec(5)=theta(t)   Perturbation in Vehicle pitch attitude       
%xvec(6)=psi(t)     Perturbation in Vehicle yaw attitude
%xvec(7)=u(t)       Perturbation in Vehicle longitudinal velocity along body X axis
%xvec(8)=v(t)       Perturbation in Vehicle lateral velocity along body X axis
%xvec(9)=w(t)       Perturbation in Vehicle vertical velocity along body Z axis
%xvec(10)=p(t)      Perturbation in Vehicle roll rate
%xvec(11)=q(t)      Perturbation in Vehicle pitch rate
%xvec(12)=r(t)      Perturbation in Vehicle yaw rate

%Outputs Solution legend
%out_states(1)=U(t)  Vehicle forward velocity (X-inertial)
%out_states(2)=V(t)  Vehicle lateral velocity (Y-inertial)
%out_states(3)=W(t)  Vehicle vertical velocity (Z-inertial)
%out_states(4)=X(t)  Vehicle forward position (X-inertial)
%out_states(5)=Y(t)  Vehicle lateral position (Y-inertial)
%out_states(6)=Z(t)  Vehicle vertical position (Z-inertial)


%%OPTIMAL SOLUTION, SHIP AT CONSTANT X-VELOCITY AND Z=0
[xvec,del,tvec,Xeq,xseq,X_optdel,X_opt,Y_opt,Z_opt,U_opt,V_opt,W_opt,inert_val] = optimal_control_40_full_rel(model_time,time_step,Afull,Bfull,trim_val,us0);  %Optimal control and reference paths
dt=tvec(end)/length(tvec);    %time step (sec)
U0_opt = inert_val(1);
V0_opt = inert_val(2);
W0_opt = inert_val(3);
X0_opt = inert_val(4);
Y0_opt = inert_val(5);
Z0_opt = inert_val(6);


%%VARIABLE INITIALIZATION
del_con=repmat(del,[1 1 tr]);     %initial optimal command
states=xvec;    %initial states
states_p=repmat(xvec,[1 1 tr]);   %Predicted perturbation in states
pathcost=zeros(size(del_con));      %Initial path cost
Ji=ones(size(del_con));      %Performance index
Ji_mod=ones(size(del_con));  %Normalized by logarithmic scale (10^-3 to 10^1)
ddeli_e=zeros(size(del_con));  %Exponential weighing factor, x-axis
out_states=zeros(6,size(xvec,2));
cycles=zeros(size(tvec));


azs=zeros(1,size(xvec,2)); %Placeholder for ship's acceleration, z-axis
ws=zeros(1,size(xvec,2)); %Placeholder for ship's velocity, z-axis
zs=zeros(1,size(xvec,2)); %Placeholder for ship's position, z-axis
xs=zeros(1,size(xvec,2)); %Placeholder for changes in ship's position, x-axis
ys=zeros(1,size(xvec,2)); %Placeholder for ship's position, y-axis

np=zeros(1,size(xvec,2)); %Placeholder number of usable trajectories


%INERTIAL CALCULATIONS for INITIALIZATION
out_states(1,1) = cos(theta0)*cos(psi0)*u0 + (sin(phi0)*sin(theta0)*cos(psi0) - cos(phi0)*sin(psi0))*v0 + (cos(phi0)*sin(theta0)*cos(psi0) + sin(phi0)*sin(psi0))*w0;
out_states(2,1) = -(cos(theta0)*sin(psi0)*u0 + (sin(phi0)*sin(theta0)*sin(psi0) + cos(phi0)*cos(psi0))*v0 + (cos(phi0)*sin(theta0)*sin(psi0) - sin(phi0)*cos(psi0))*w0);
out_states(3,1) = -(-sin(theta0)*u0 + sin(phi0)*cos(theta0)*v0 + cos(phi0)*cos(theta0)*w0);
out_states(4,1) = states(1,1);
out_states(5,1) = states(2,1);
out_states(6,1) = states(3,1);

out_states_p=repmat(out_states,[1 1 tr]);  %Predicted output states
cost=0;


%%SHIP'S VERTICAL MOTION MODEL
rng('shuffle');
s=1;                    %The following while loop calculates the ship's heave motion.
                        %Ship acceleration is based on random sine wave with "sd"
                        %standard deviation and "f" frequency with period
                        %between 4.5 - 8 seconds
while s<size(xvec,2)
    A=sdz*randn(1);            %Normally distributed amplitude
    f=0.09722*rand(1)+0.125;  %Frequency range of 4.5-8 sec.
    T_2=round((1/(dt*f))/2);  %Half Period (index)
    hp=(s-1)+T_2;
    if hp<size(xvec,2)
        for s=s:hp
           azs(s)=A*sin(2*pi()*f*tvec(s)); %Calculates peak
        end
    else
        hp=size(xvec,2);
         for s=s:hp
           azs(s)=A*sin(2*pi()*f*tvec(s)); %Calculates peak
         end
    end
    A=sdz*randn(1);            %Draws new normally distributed random amplitude
    ep=s+T_2;
    if ep<size(xvec,2)
        for s=s+1:ep
           azs(s)=A*sin(2*pi()*f*tvec(s)); %Calculates valley
        end
    else
        ep=size(xvec,2);
        for s=s+1:ep
           azs(s)=A*sin(2*pi()*f*tvec(s)); %Calculates valley
        end
    end
end

for s=2:size(xvec,2)          %Calculates ship's vertical velocity and position
    ws(s)=azs(s)*dt;
    zs(s)=zs(s-1)+ws(s)*dt;
end

%%SHIP'S HORIZONTAL MOTION MODEL
us=sdx*randn(1,size(xvec,2)); %Ship's forward velocity due to surge motion
for s=2:size(xvec,2)          %Calculates ship's changes in x-position
    xs(s)=xs(s-1)+us(s)*dt;
end


%%SHIP'S LATERAL MOTION MODEL
vs=sdy*randn(1,size(xvec,2)); %Ship's sideward velocity due to sway motion
for s=2:size(xvec,2)          %Calculates ship's changes in y-position
    ys(s)=ys(s-1)+vs(s)*dt;
end


%%SENSITIVITY SELECTION FOR TRAJECTORY GENERATION
rb=0.1*ones(1,size(xvec,2));%Sensitivity of differential commanded longitudinal cyclic, starts at 2% of initial optimal longitudinal cyclic
ra=0.1*ones(1,size(xvec,2));%Sensitivity of differential commanded lateral cyclic, starts at 2% of initial optimal lateral cyclic
rp=0.1*ones(1,size(xvec,2));%Sensitivity of differential commanded pedal, starts at 2% of initial optimal pedal
rc=0.1*ones(1,size(xvec,2)); %Sensitivity of differential commanded collective, starts at 10% of initial optimal collective
F=logspace(-2,0.5,model_time/time_step);  %weight used to trade between path cost and terminal cost


%%PATH INTEGRAL MODEL
for j=2:size(xvec,2)
    n=tr;       %Counter of trajectories with final x-position outside of specified band in relation to ship's landing position
    m=1;        %Counter for number of cycles in while loop
    while (n>90 && m<10)                     %Limits trajectories outside of an specified band to 90/100 and 9 cycles
        delbmax=rb(j)*abs(del_con(1,:,:));                %Vehicle max differential longitudinal cyclic inch
        delamax=ra(j)*abs(del_con(2,:,:));                %Vehicle max differential lateral cyclic inch
        delpmax=rp(j)*abs(del_con(3,:,:));                %Vehicle max differential pedal inch
        delcmax=rc(j)*abs(del_con(4,:,:));                %Vehicle max differential collective inch
        ddelb=2*delbmax.*rand(size(del_con(1,:,:)))-delbmax;  %Creates random differential longitudinal cyclic variations at each node
        ddela=2*delamax.*rand(size(del_con(2,:,:)))-delamax;  %Creates random differential lateral cyclic variations at each node
        ddelp=2*delpmax.*rand(size(del_con(3,:,:)))-delpmax;  %Creates random differential pedal variations at each node
        ddelc=2*delcmax.*rand(size(del_con(4,:,:)))-delcmax;  %Creates random differential collective variations at each node
        del_bi=del_con(1,:,:)+ddelb;                        %Set of multiple trajectories
        del_ai=del_con(2,:,:)+ddela;                        %Set of multiple trajectories
        del_pi=del_con(3,:,:)+ddelp;                        %Set of multiple trajectories
        del_ci=del_con(4,:,:)+ddelc;                        %Set of multiple trajectories
        [states_p,out_states_p]=predicted_vehicle_state_trapz(del_bi,del_ai,del_pi,del_ci,dt,states_p,out_states_p,j,Afull,Bfull,trim_val);%Calculates predicted vehicle state based on vehicle dynamics        
        n=0;                                %Starts counter for trajectories with final position outside of specified band
        for l=1:size(del_con,3)                 %Calculates the performance index for each trajectory
            pc=zeros(4,1);                           %Resets path cost
            for k=j:size(del_con,2)
                pc(1)=pc(1)+0.5*dt*(0.025*del_bi(1,k,l)^2+0.25*((out_states_p(1,j,l)-(U0_opt+U_opt(j)))^2)+(1e-06)*((out_states_p(4,j,l)-X_optdel(j))^2));   %Calculates path cost for each trajectory
                pc(2)=pc(2)+0.5*dt*(0.025*del_ai(1,k,l)^2+0.0167*((out_states_p(2,j,l)-(V0_opt+V_opt(j)))^2)+(2.5e-05)*((out_states_p(5,j,l)-Y_opt(j))^2));   %Calculates path cost for each trajectory
                pc(3)=pc(3)+0.5*dt*(0.05*del_pi(1,k,l)^2);   %Calculates path cost for each trajectory
                pc(4)=pc(4)+0.5*dt*(0.025*del_ci(1,k,l)^2+0.25*((out_states_p(4,j,l)-(W0_opt+W_opt(j)))^2)+0.0025*((out_states_p(6,j,l)-Z_opt(j))^2));   %Calculates path cost for each trajectory
                %{
                pc(1)=pc(1)+0.5*dt*(0.025*del_bi(1,k,l)^2+0.25*((out_states_p(1,j,l)-(U0_opt+U_opt(j)))^2)+(1e-06)*((out_states_p(4,j,l)-X_optdel(j))^2));   %Calculates path cost for each trajectory
                pc(2)=pc(2)+0.5*dt*(0.1*del_ai(1,k,l)^2+0.025*((out_states_p(2,j,l)-(V0_opt+V_opt(j)))^2)+(2.5e-05)*((out_states_p(5,j,l)-Y_opt(j))^2));   %Calculates path cost for each trajectory
                pc(3)=pc(3)+0.5*dt*(2.5*del_pi(1,k,l)^2);   %Calculates path cost for each trajectory
                pc(4)=pc(4)+0.5*dt*(0.025*del_ci(1,k,l)^2+0.25*((out_states_p(4,j,l)-(W0_opt+W_opt(j)))^2)+0.0025*((out_states_p(6,j,l)-Z_opt(j))^2));   %Calculates path cost for each trajectory
                %}
            end
            pathcost(:,j,l)=pc;        
            if abs(out_states_p(1,end,l)-(us0+us(end)))>band_u && abs(out_states_p(4,end,l)+Xeq(end)+X0_opt-(xseq(end)+xs(end)))>band_x && ...
                    abs(out_states_p(2,end,l)-vs(end))>band_v && abs(out_states_p(5,end,l)+Y0_opt-ys(end))>band_y && ...
                    abs(out_states_p(3,end,l)-ws(j))>band_w && abs(out_states_p(6,end,l)+Z0_opt-zs(j))>band_z
                Ji(:,j,l)=0;
                n=n+1;
            else
                Ji(1,j,l)=pathcost(1,j,l)+pathcost(2,j,l)+pathcost(3,j,l)+pathcost(4,j,l)+F(j)*((out_states_p(1,end,l)-(us0+us(end)))^2+(out_states_p(4,end,l)+Xeq(end)+X0_opt-(xseq(end)+xs(end)))^2); %Performance Index
                Ji(2,j,l)=pathcost(1,j,l)+pathcost(2,j,l)+pathcost(3,j,l)+pathcost(4,j,l)+F(j)*((out_states_p(2,end,l)-vs(end))^2+(out_states_p(5,end,l)+Y0_opt(end)-ys(end))^2);
                Ji(3,j,l)=pathcost(1,j,l)+pathcost(2,j,l)+pathcost(3,j,l)+pathcost(4,j,l);
                Ji(4,j,l)=pathcost(1,j,l)+pathcost(2,j,l)+pathcost(3,j,l)+pathcost(4,j,l)+F(j)*((out_states_p(3,end,l)-ws(j))^2+(out_states_p(6,end,l)+Z0_opt(end)-zs(j))^2); %Performance Index
            end
        end
        rb(j)=rb(j)+rb_inc;                    %Increments sensitivity
        ra(j)=ra(j)+ra_inc;
        rp(j)=rp(j)+rp_inc;
        rc(j)=rc(j)+rc_inc;
        m=m+1;
    end

    cycles(j)=m;
    np(j)=size(del_con,3)-n;                       %Number of usable trajectories
    J(:,1)=Ji(1,j,:);
    Ji_mod(1,j,:) = indexnorm2(J,np(j),tr);  %Normalizes the performance index to give more weight to lowest cost trajectory
    J(:,1)=Ji(2,j,:);
    Ji_mod(2,j,:) = indexnorm2z(J,np(j),tr);  %Normalizes the performance index to give more weight to lowest cost trajectory
    J(:,1)=Ji(3,j,:);
    Ji_mod(3,j,:) = indexnorm2z(J,np(j),tr);  %Normalizes the performance index to give more weight to lowest cost trajectory
    J(:,1)=Ji(4,j,:);
    Ji_mod(4,j,:) = indexnorm2z(J,np(j),tr);  %Normalizes the performance index to give more weight to lowest cost trajectory
    for l=1:size(del_con,3)                    %Performs exponential weighting of all trajectories
        if Ji_mod(:,j,l)==[0;0;0;0]
            for k=j:size(del_con,2)
                ddeli_e(:,k,l)=[0;0;0;0];
            end
        else
            for k=j:size(del_con,2)
                ddeli_e(1,k,l)=ddelb(1,k,l)*exp(-Ji_mod(1,j,l));
                ddeli_e(2,k,l)=ddela(1,k,l)*exp(-Ji_mod(2,j,l));
                ddeli_e(3,k,l)=ddelp(1,k,l)*exp(-Ji_mod(3,j,l));
                ddeli_e(4,k,l)=ddelc(1,k,l)*exp(-Ji_mod(4,j,l));
            end
        end
    end
    if n==tr                               %Avoids dividing by 0
        for k=j:size(del_con,2)                 %Uses previous optimal trajectory
            del_con(1,k,:)=del(1,k);
            del_con(2,k,:)=del(2,k);
            del_con(3,k,:)=del(3,k);
            del_con(4,k,:)=del(4,k);
        end
    else
        for k=j:size(del_con,2)                  %Calculates best trajectory at this time step
            del_con(1,k,:)=del_con(1,k,:)+ sum(ddeli_e(1,k,:))/(size(del_con,3)-n);
            del_con(2,k,:)=del_con(2,k,:)+ sum(ddeli_e(2,k,:))/(size(del_con,3)-n);
            del_con(3,k,:)=del_con(3,k,:)+ sum(ddeli_e(3,k,:))/(size(del_con,3)-n);
            del_con(4,k,:)=del_con(4,k,:)+ sum(ddeli_e(4,k,:))/(size(del_con,3)-n);
        end
    end
    [states,out_states,cost]=vehicle_dynamics_trapz(del_con,dt,states,out_states,j,Afull,Bfull,trim_val,U_opt,V_opt,W_opt,X_optdel,Y_opt,Z_opt,inert_val,cost);%Calculates actual vehicle state based on vehicle dynamics    
end
cost=cost+0.5*dt*(0.025*del_con(1,end,1)^2+0.025*del_con(2,end,1)^2+0.05*del_con(3,end,1)^2+0.025*del_con(4,end,1)^2 ...
    +0.25*((out_states(1,end)-(U0_opt+U_opt(end)))^2)+0.0167*((out_states(2,end)-(V0_opt+V_opt(end)))^2) ...
    +0.25*((out_states(3,end)-(W0_opt+W_opt(end)))^2)+(1e-06)*((out_states(4,end)-X_optdel(end))^2) ...
    +(2.5e-05)*((out_states(5,end)-Y_opt(end))^2)+0.0025*((out_states(6,end)-Z_opt(end))^2));
term_cost_x=F(end)*((out_states(1,end)-(us0+us(end)))^2+(out_states(4,end)+Xeq(end)+X0_opt-(xseq(end)+xs(end)))^2);
term_cost_y=F(end)*((out_states(2,end)-vs(end))^2+(out_states(5,end)+Y0_opt-ys(end))^2);
term_cost_z=F(end)*((out_states(3,end)-ws(end))^2+(out_states(6,end)+Z0_opt-zs(end))^2);

%% PLOTS

toc
figure (1)
subplot(3,1,1);
hold on
plot(tvec,X0_opt+X_opt,'c','LineWidth',1.5)
plot(tvec,Xeq+X0_opt+out_states(4,:),'r','LineWidth',1.5)
plot(tvec,xseq+xs,'--b','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax1=gca;
set(ax1,'GridLineStyle','-');
set(ax1,'MinorGridLineStyle',':');
title('Vehicle Inertial Position, X-axis','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Horizontal Position (ft)','FontSize',12);
leg=legend('Reference Horizontal Position (1st Order Model)','Vehicle Horizontal Position (MPPI)','Ship Horizontal Position','location','best');    %'Vehicle Horizontal Position (Optimal Control Theory)',
set(leg,'FontSize',10);
subplot(3,1,2);
hold on
plot(tvec,Y0_opt+Y_opt,'c','LineWidth',1.5)
plot(tvec,Y0_opt+out_states(5,:),'r','LineWidth',1.5)
plot(tvec,ys,'--b','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax1=gca;
set(ax1,'GridLineStyle','-');
set(ax1,'MinorGridLineStyle',':');
title('Vehicle Inertial Position, Y-axis','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Lateral Position (ft)','FontSize',12);
leg=legend('Reference Lateral Position (1st Order Model)','Vehicle Lateral Position (MPPI)','Ship Lateral Position','location','best');  %'Vehicle Vertical Position (Optimal Control Theory)',
set(leg,'FontSize',10);
subplot(3,1,3);
hold on
plot(tvec,Z0_opt+Z_opt,'c','LineWidth',1.5)
plot(tvec,Z0_opt+out_states(6,:),'r','LineWidth',1.5)
plot(tvec,zs,'--b','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax1=gca;
set(ax1,'GridLineStyle','-');
set(ax1,'MinorGridLineStyle',':');
title('Vehicle Inertial Position, Z-axis','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Altitude (ft)','FontSize',12);
leg=legend('Reference Vertical Position (1st Order Model)','Vehicle Vertical Position (MPPI)','Ship Vertical Position','location','best');  %'Vehicle Vertical Position (Optimal Control Theory)',
set(leg,'FontSize',10);

figure (2)
subplot(3,1,1);
hold on
plot(tvec,U0_opt+U_opt,'c','LineWidth',1.5)
plot(tvec,out_states(1,:),'r','LineWidth',1.5)
plot(tvec,us0+us,'--b','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax2=gca;
set(ax2,'GridLineStyle','-');
set(ax2,'MinorGridLineStyle',':');
title('Vehicle Inertial Velocity, X-axis','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Horizontal Velocity (ft/s)','FontSize',12);
leg=legend('Reference Horizontal Velocity (1st Order Model)','Vehicle X-Velocity (MPPI)','Ship X-Velocity','location','best');  %'Vehicle X-Velocity (Optimal Control Theory)',
set(leg,'FontSize',10);
subplot(3,1,2);
hold on
plot(tvec,V0_opt+V_opt,'c','LineWidth',1.5)
plot(tvec,out_states(2,:),'r','LineWidth',1.5)
plot(tvec,vs,'--b','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax2=gca;
set(ax2,'GridLineStyle','-');
set(ax2,'MinorGridLineStyle',':');
title('Vehicle Inertial Velocity, Y-axis','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Lateral Velocity (ft/s)','FontSize',12);
leg=legend('Reference Lateral Velocity (1st Order Model)','Vehicle Y-Velocity (MPPI)','Ship Y-Velocity','location','best');    %'Vehicle Z-Velocity (Optimal Control Theory)',
set(leg,'FontSize',10);
subplot(3,1,3);
hold on
plot(tvec,W0_opt+W_opt,'c','LineWidth',1.5)
plot(tvec,out_states(3,:),'r','LineWidth',1.5)
plot(tvec,ws,'--b','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax2=gca;
set(ax2,'GridLineStyle','-');
set(ax2,'MinorGridLineStyle',':');
title('Vehicle Inertial Velocity, Z-axis','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Vertical Velocity (ft/s)','FontSize',12);
leg=legend('Reference Vertical Velocity (1st Order Model)','Vehicle Z-Velocity (MPPI)','Ship Z-Velocity','location','best');    %'Vehicle Z-Velocity (Optimal Control Theory)',
set(leg,'FontSize',10);

figure (3)
subplot(3,1,1);
hold on
plot(tvec,(180/pi).*(phi0+states(4,:)),'r','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Vehicle Roll Attitude','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Roll Attitude (deg)','FontSize',12);
leg=legend('Vehicle Roll Attitude','location','best');  %'Vehicle Pitch Attitude (Optimal Control Theory)',
set(leg,'FontSize',10);
subplot(3,1,2);
hold on
plot(tvec,(180/pi).*(theta0+states(5,:)),'r','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Vehicle Pitch Attitude','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Pitch Attitude (deg)','FontSize',12);
leg=legend('Vehicle Pitch Attitude','location','best');  %'Vehicle Pitch Attitude (Optimal Control Theory)',
set(leg,'FontSize',10);
subplot(3,1,3);
hold on
plot(tvec,(180/pi).*(psi0+states(6,:)),'r','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Vehicle Yaw Attitude','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Yaw Attitude (deg)','FontSize',12);
leg=legend('Vehicle Yaw Attitude','location','best');
set(leg,'FontSize',10);

figure(4)
subplot(3,1,1);
hold on
plot(tvec,(180/pi).*states(10,:),'r','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Vehicle Roll Rate','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Pitch Roll (deg/sec)','FontSize',12);
leg=legend('Vehicle Roll Rate','location','best');
set(leg,'FontSize',10);
subplot(3,1,2);
hold on
plot(tvec,(180/pi).*states(11,:),'r','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Vehicle Pitch Rate','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Pitch Rate (deg/sec)','FontSize',12);
leg=legend('Vehicle Pitch Rate','location','best');
set(leg,'FontSize',10);
subplot(3,1,3);
hold on
plot(tvec,(180/pi).*states(12,:),'r','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Vehicle Yaw Rate','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Yaw Rate (deg/sec)','FontSize',12);
leg=legend('Vehicle Yaw Rate','location','best');
set(leg,'FontSize',10);

figure (5)
subplot(4,1,1);
hold on
plot(tvec,0.1.*(del_a0+del_con(2,:,1)),'r','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Lateral Cyclic','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Lateral Cyclic (inch)','FontSize',12);
leg=legend('Lateral Cyclic Stick Input','location','best');
set(leg,'FontSize',10);
subplot(4,1,2);
hold on
plot(tvec,0.1.*(del_b0+del_con(1,:,1)),'r','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Longitudinal Cyclic','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Longitudinal Cyclic (inch)','FontSize',12);
leg=legend('Longitudinal Cyclic Stick Input','location','best');
set(leg,'FontSize',10);
subplot(4,1,3);
hold on
plot(tvec,0.1.*(del_c0+del_con(4,:,1)),'r','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Collective','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Collective (inch)','FontSize',12);
leg=legend('Collective Stick Input','location','best');
set(leg,'FontSize',10);
subplot(4,1,4);
hold on
plot(tvec,0.0538.*(del_p0+del_con(3,:,1)),'r','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Pedal','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Pedal (inch)','FontSize',12);
leg=legend('Pedal Input','location','best');
set(leg,'FontSize',10);


figure (6)
plot(tvec,azs,'b','LineWidth',1.5)
grid on
grid minor
set(gca,'fontsize',12)
ax4=gca;
set(ax4,'GridLineStyle','-');
set(ax4,'MinorGridLineStyle',':');
title('Ship Vertical Acceleration','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Vertical Acceleration (ft/sec^2)','FontSize',12);

figure(7)
hold on
plot3(X0_opt+X_opt,Y0_opt+Y_opt,Z0_opt+Z_opt,'c','LineWidth',1.5)
plot3(Xeq+X0_opt+out_states(4,:),Y0_opt+out_states(5,:),Z0_opt+out_states(6,:),'r','LineWidth',1.5)
plot3(xseq+xs,ys,zs,'--b','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Helicopter and Ship Motion','FontSize',12,'FontWeight','Bold');
xlabel('X Inertial Position (ft)','FontSize',12);
ylabel('Y Inertial Position (ft)','FontSize',12);
zlabel('Z Inertial Position (ft)','FontSize',12);
leg=legend('Reference Trajectory','Helicopter Motion','Ship deck Motion','location','best');
set(leg,'FontSize',10);


figure (8)
subplot(3,1,1);
hold on
plot(tvec,Xeq+X0_opt+out_states(4,:),'r','LineWidth',1.5)
plot(tvec,Xeq+X0_opt+out_states_p(4,:,5),'--k','LineWidth',1.5)
plot(tvec,Xeq+X0_opt+out_states_p(4,:,24),'--g','LineWidth',1.5)
plot(tvec,Xeq+X0_opt+out_states_p(4,:,56),'--y','LineWidth',1.5)
plot(tvec,Xeq+X0_opt+out_states_p(4,:,78),'--m','LineWidth',1.5)
plot(tvec,Xeq+X0_opt+out_states_p(4,:,99),':k','LineWidth',1.5)
plot(tvec,xseq+xs,'--b','LineWidth',1.5)
plot(tvec,X0_opt+X_opt,'c','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax1=gca;
set(ax1,'GridLineStyle','-');
set(ax1,'MinorGridLineStyle',':');
title('Vehicle Relative Position, Inertial X-axis','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Horizontal Position (ft)','FontSize',12);
leg=legend('Vehicle Horizontal Position','Predicted Trajectory','Predicted Trajectory','Predicted Trajectory','Predicted Trajectory','Predicted Trajectory','Ship Horizontal Position','Initial Optimal X-Position','location','best');
set(leg,'FontSize',10);
subplot(3,1,2);
hold on
plot(tvec,Y0_opt+out_states(5,:),'r','LineWidth',1.5)
plot(tvec,Y0_opt+out_states_p(5,:,5),'--k','LineWidth',1.5)
plot(tvec,Y0_opt+out_states_p(5,:,24),'--g','LineWidth',1.5)
plot(tvec,Y0_opt+out_states_p(5,:,56),'--y','LineWidth',1.5)
plot(tvec,Y0_opt+out_states_p(5,:,78),'--m','LineWidth',1.5)
plot(tvec,Y0_opt+out_states_p(5,:,99),':k','LineWidth',1.5)
plot(tvec,ys,'--b','LineWidth',1.5)
plot(tvec,Y0_opt+Y_opt,'c','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax1=gca;
set(ax1,'GridLineStyle','-');
set(ax1,'MinorGridLineStyle',':');
title('Vehicle Relative Position, Inertial Y-axis','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Altitude (ft)','FontSize',12);
leg=legend('Vehicle Lateral Position','Predicted Trajectory','Predicted Trajectory','Predicted Trajectory','Predicted Trajectory','Predicted Trajectory','Ship Vertical Position','Initial Optimal Z-Position','location','best');
set(leg,'FontSize',10);
subplot(3,1,3);
hold on
plot(tvec,Z0_opt+out_states(6,:),'r','LineWidth',1.5)
plot(tvec,Z0_opt+out_states_p(6,:,5),'--k','LineWidth',1.5)
plot(tvec,Z0_opt+out_states_p(6,:,24),'--g','LineWidth',1.5)
plot(tvec,Z0_opt+out_states_p(6,:,56),'--y','LineWidth',1.5)
plot(tvec,Z0_opt+out_states_p(6,:,78),'--m','LineWidth',1.5)
plot(tvec,Z0_opt+out_states_p(6,:,99),':k','LineWidth',1.5)
plot(tvec,zs,'--b','LineWidth',1.5)
plot(tvec,Z0_opt+Z_opt,'c','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax1=gca;
set(ax1,'GridLineStyle','-');
set(ax1,'MinorGridLineStyle',':');
title('Vehicle Relative Position, Inertial Z-axis','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Altitude (ft)','FontSize',12);
leg=legend('Vehicle Vertical Position','Predicted Trajectory','Predicted Trajectory','Predicted Trajectory','Predicted Trajectory','Predicted Trajectory','Ship Vertical Position','Initial Optimal Z-Position','location','best');
set(leg,'FontSize',10);

figure (9)
subplot(4,1,1);
hold on
plot(tvec,0.1.*(del_a0+del_con(2,:,1)),'r','LineWidth',1.5)
plot(tvec,0.1.*(del_a0+del_ai(1,:,5)),'--k','LineWidth',1.5)
plot(tvec,0.1.*(del_a0+del_ai(1,:,24)),'--g','LineWidth',1.5)
plot(tvec,0.1.*(del_a0+del_ai(1,:,56)),'--y','LineWidth',1.5)
plot(tvec,0.1.*(del_a0+del_ai(1,:,78)),'--m','LineWidth',1.5)
plot(tvec,0.1.*(del_a0+del_ai(1,:,99)),':k','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Lateral Cyclic','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Lateral Cyclic (inch)','FontSize',12);
leg=legend('Lateral Cyclic Stick Input','location','best');
set(leg,'FontSize',10);
subplot(4,1,2);
hold on
plot(tvec,0.1.*(del_b0+del_con(1,:,1)),'r','LineWidth',1.5)
plot(tvec,0.1.*(del_b0+del_bi(1,:,5)),'--k','LineWidth',1.5)
plot(tvec,0.1.*(del_b0+del_bi(1,:,24)),'--g','LineWidth',1.5)
plot(tvec,0.1.*(del_b0+del_bi(1,:,56)),'--y','LineWidth',1.5)
plot(tvec,0.1.*(del_b0+del_bi(1,:,78)),'--m','LineWidth',1.5)
plot(tvec,0.1.*(del_b0+del_bi(1,:,99)),':k','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Longitudinal Cyclic','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Longitudinal Cyclic (inch)','FontSize',12);
leg=legend('Longitudinal Cyclic Stick Input','location','best');
set(leg,'FontSize',10);
subplot(4,1,3);
hold on
plot(tvec,0.1.*(del_c0+del_con(4,:,1)),'r','LineWidth',1.5)
plot(tvec,0.1.*(del_c0+del_ci(1,:,5)),'--k','LineWidth',1.5)
plot(tvec,0.1.*(del_c0+del_ci(1,:,24)),'--g','LineWidth',1.5)
plot(tvec,0.1.*(del_c0+del_ci(1,:,56)),'--y','LineWidth',1.5)
plot(tvec,0.1.*(del_c0+del_ci(1,:,78)),'--m','LineWidth',1.5)
plot(tvec,0.1.*(del_c0+del_ci(1,:,99)),':k','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Collective','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Collective (inch)','FontSize',12);
leg=legend('Collective Stick Input','location','best');
set(leg,'FontSize',10);
subplot(4,1,4);
hold on
plot(tvec,0.0538.*(del_p0+del_con(3,:,1)),'r','LineWidth',1.5)
plot(tvec,0.0538.*(del_p0+del_pi(1,:,5)),'--k','LineWidth',1.5)
plot(tvec,0.0538.*(del_p0+del_pi(1,:,24)),'--g','LineWidth',1.5)
plot(tvec,0.0538.*(del_p0+del_pi(1,:,56)),'--y','LineWidth',1.5)
plot(tvec,0.0538.*(del_p0+del_pi(1,:,78)),'--m','LineWidth',1.5)
plot(tvec,0.0538.*(del_p0+del_pi(1,:,99)),':k','LineWidth',1.5)
hold off
grid on
grid minor
set(gca,'fontsize',12)
ax3=gca;
set(ax3,'GridLineStyle','-');
set(ax3,'MinorGridLineStyle',':');
title('Pedal','FontSize',12,'FontWeight','Bold');
xlabel('Time (sec)','FontSize',12);
ylabel('Pedal (inch)','FontSize',12);
leg=legend('Pedal Input','location','best');
set(leg,'FontSize',10);


toc
%Parametric Study
%{
xie=out_states(4,end)-X_optdel(end)
uie=out_states(1,end)-(U0_opt+U_opt(end))
yie=out_states(5,end)-Y_opt(end)
vie=out_states(2,end)-(V0_opt+V_opt(end))
zie=out_states(6,end)-Z_opt(end)
wie=out_states(3,end)-(W0_opt+W_opt(end))
%}
xie=Xeq(end)+X0_opt+out_states(4,end)-(xseq(end)+xs(end))
uie=out_states(1,end)-(us0+us(end))
yie=Y0_opt+out_states(5,end)-ys(end)
vie=out_states(2,end)-vs(end)
zie=Z0_opt+out_states(6,end)-zs(end)
wie=out_states(3,end)-ws(end)
npmean=mean(np)
pmcostx=cost+term_cost_x
pmcosty=cost+term_cost_y
pmcostz=cost+term_cost_z
