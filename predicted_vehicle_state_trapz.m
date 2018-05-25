function [states_p,out_states_p]=predicted_vehicle_state_trapz(del_bi,del_ai,del_pi,del_ci,dt,states_p,out_states_p,j,Afull,Bfull,trim_val)

u0 = trim_val(1);
v0 = trim_val(2);
w0 = trim_val(3);
phi0 = trim_val(4);
theta0 = trim_val(5);
psi0 = trim_val(6);

%Calculates vehicle predicted state based on vehicle dynamics
for k=j:size(del_bi,2)                  
    for l=1:size(del_bi,3)
        states_p(:,k,l) = states_p(:,k-1,l) + dt*(Afull*states_p(:,k-1,l)+Bfull*0.5*([del_bi(1,k-1,l);del_ai(1,k-1,l);del_pi(1,k-1,l);del_ci(1,k-1,l)]+[del_bi(1,k,l);del_ai(1,k,l);del_pi(1,k,l);del_ci(1,k,l)]));
        out_states_p(1,k,l) = cos(theta0+states_p(5,k,l))*cos(psi0+states_p(6,k,l))*(u0+states_p(7,k,l)) + (sin(phi0+states_p(4,k,l))*sin(theta0+states_p(5,k,l))*cos(psi0+states_p(6,k,l)) ...
            - cos(phi0+states_p(4,k,l))*sin(psi0+states_p(6,k,l)))*(v0+states_p(8,k,l)) + (cos(phi0+states_p(4,k,l))*sin(theta0+states_p(5,k,l))*cos(psi0+states_p(6,k,l)) ...
            + sin(phi0+states_p(4,k,l))*sin(psi0+states_p(6,k,l)))*(w0+states_p(9,k,l));
        out_states_p(2,k,l) = -(cos(theta0+states_p(5,k,l))*sin(psi0+states_p(6,k,l))*(u0+states_p(7,k,l)) + (sin(phi0+states_p(4,k,l))*sin(theta0+states_p(5,k,l))*sin(psi0+states_p(6,k,l)) ...
            + cos(phi0+states_p(4,k,l))*cos(psi0+states_p(6,k,l)))*(v0+states_p(8,k,l)) + (cos(phi0+states_p(4,k,l))*sin(theta0+states_p(5,k,l))*sin(psi0+states_p(6,k,l)) ...
            - sin(phi0+states_p(4,k,l))*cos(psi0+states_p(6,k,l)))*(w0+states_p(9,k,l)));
        out_states_p(3,k,l) = -(-sin(theta0+states_p(5,k,l))*(u0+states_p(7,k,l)) + sin(phi0+states_p(4,k,l))*cos(theta0+states_p(5,k,l))*(v0+states_p(8,k,l)) ...
            + cos(phi0+states_p(4,k,l))*cos(theta0+states_p(5,k,l))*(w0+states_p(9,k,l)));
        out_states_p(4,k,l) = states_p(1,k,l);
        out_states_p(5,k,l) = states_p(2,k,l);
        out_states_p(6,k,l) = states_p(3,k,l);
    end
end
end
