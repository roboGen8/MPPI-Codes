function [states,out_states,cost]=vehicle_dynamics_trapz(del_con,dt,states,out_states,j,Afull,Bfull,trim_val,U_opt,V_opt,W_opt,X_optdel,Y_opt,Z_opt,inert_val,cost) 

u0 = trim_val(1);
v0 = trim_val(2);
w0 = trim_val(3);
phi0 = trim_val(4);
theta0 = trim_val(5);
psi0 = trim_val(6);

U0_opt = inert_val(1);
V0_opt = inert_val(2);
W0_opt = inert_val(3);


%Calculates actual vehicle state based on vehicle dynamics
states(:,j) = states(:,j-1) + dt*(Afull*states(:,j-1) + Bfull*0.5*(del_con(:,j-1,1)+del_con(:,j,1)));
out_states(1,j) = cos(theta0+states(5,j))*cos(psi0+states(6,j))*(u0+states(7,j)) + (sin(phi0+states(4,j))*sin(theta0+states(5,j))*cos(psi0+states(6,j)) ...
            - cos(phi0+states(4,j))*sin(psi0+states(6,j)))*(v0+states(8,j)) + (cos(phi0+states(4,j))*sin(theta0+states(5,j))*cos(psi0+states(6,j)) ...
            + sin(phi0+states(4,j))*sin(psi0+states(6,j)))*(w0+states(9,j));
out_states(2,j) = -(cos(theta0+states(5,j))*sin(psi0+states(6,j))*(u0+states(7,j)) + (sin(phi0+states(4,j))*sin(theta0+states(5,j))*sin(psi0+states(6,j)) ...
            + cos(phi0+states(4,j))*cos(psi0+states(6,j)))*(v0+states(8,j)) + (cos(phi0+states(4,j))*sin(theta0+states(5,j))*sin(psi0+states(6,j)) ...
            - sin(phi0+states(4,j))*cos(psi0+states(6,j)))*(w0+states(9,j)));
out_states(3,j) = -(-sin(theta0+states(5,j))*(u0+states(7,j)) + sin(phi0+states(4,j))*cos(theta0+states(5,j))*(v0+states(8,j)) ...
            + cos(phi0+states(4,j))*cos(theta0+states(5,j))*(w0+states(9,j)));
out_states(4,j) = states(1,j);
out_states(5,j) = states(2,j);
out_states(6,j) = states(3,j);
cost=cost+0.5*dt*(0.025*del_con(1,j-1,1)^2+0.025*del_con(2,j-1,1)^2+0.05*del_con(3,j-1,1)^2+0.025*del_con(4,j-1,1)^2 ...
    +0.25*((out_states(1,j-1)-(U0_opt+U_opt(j-1)))^2)+0.0167*((out_states(2,j-1)-(V0_opt+V_opt(j-1)))^2) ...
    +0.25*((out_states(3,j-1)-(W0_opt+W_opt(j-1)))^2)+(1e-06)*((out_states(4,j-1)-X_optdel(j-1))^2) ...
    +(2.5e-05)*((out_states(5,j-1)-Y_opt(j-1))^2)+0.0025*((out_states(6,j-1)-Z_opt(j-1))^2));
end
