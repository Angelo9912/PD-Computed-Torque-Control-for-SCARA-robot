syms m I X_u Y_v Z_w N_r N_r_r f_g Y_v_dot Y_r X_u_dot
syms u v w r u_dot v_dot w_dot r_dot tau_u tau_w tau_r tau_v
syms x y z psi x_dot y_dot z_dot psi_dot

M = diag([m m m I]);
C = [0 0 0 -m*v;
     0 0 0 m*u;
     0 0 0 0;
     m*v -m*u 0 0];
D = [-X_u 0 0 0;
      0 -Y_v 0 -Y_r;
      0 0 -Z_w 0;
      0 -N_v 0 -N_r];
tau = [tau_u;tau_v;tau_w;tau_r];
ni = [u;v;w;r];
%g = [0;0;f_g;0];
ni_dot = inv(M)*(-C*ni - D*ni + tau);

eta_dot = [cos(psi) -sin(psi) 0 0;
           sin(psi) cos(psi) 0 0;
           0 0 1 0;
           0 0 0 1]*[u;v;w;r];

xi_dot = [eta_dot;ni_dot];
xi = [x y z psi u v w r]';
H = [1 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0;
     0 0 0 1 0 0 0 0];

y_dot  = H*xi_dot