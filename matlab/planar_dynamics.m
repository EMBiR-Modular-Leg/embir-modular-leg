% kinematics definitions:
l1 = .125;
l2_pll = .025;
l2_perp = .140;
l3_pll = .020;
l3_perp = .135;

J_a_r = 1e-4;
J_a_a = 4e-4;
M_a = 0.5;

% Constants
r_b = 0.0; % m
 c_b = 0.0; % m centered mass
 beta_b = 0; % rad
 M_b = 2*M_a + 0.5; % kg two actuators + carriage
 J_b = 1e-3; % kg m^2
 gamma_b = 0; % rad

r_f = l1;
 c_f = 0.05;
 beta_f = pi;
 M_f = 0.2;
 J_f = 1e-3+J_a_a;
 gamma_f = 0;

r_t = (l2_pll^2 + l2_perp^2)^0.5;
 c_t = 0.03;
 beta_t = 0.5*pi;
 M_t = 0.11;
 J_t = 2e-4+J_a_a+J_a_r;
 gamma_t = atan2(l2_pll, l2_perp);

r_p = (l3_pll^2 + l3_perp^2)^0.5;
 c_p = 0.03;
 beta_p = 0.75*pi;
 M_p = 0.07;
 J_p = 1e-4;
 gamma_p = atan2(l3_pll, l3_perp);
 
B_f = 4.4e-3; % Nm s/rad
B_t = 4.4e-3;


% syms r_b c_b beta_b M_b J_b gamma_b;
% syms r_f c_f beta_f M_f J_f gamma_f;
% syms r_t c_t beta_t M_t J_t gamma_t;
% syms r_p c_p beta_p M_p J_p gamma_p;

syms t;

% Native coordinates
syms y_b(t) v_y_b p_y_b z_b(t) v_z_b p_z_b phi_b(t) v_phi_b p_phi_b T_b V_b;
syms y_f(t) v_y_f p_y_f z_f(t) v_z_f p_z_f phi_f(t) v_phi_f p_phi_f T_f V_f;
syms y_t(t) v_y_t p_y_t z_t(t) v_z_t p_z_t phi_t(t) v_phi_t p_phi_t T_t V_t;
syms y_p(t) v_y_p p_y_p z_p(t) v_z_p p_z_p phi_p(t) v_phi_p p_phi_p T_p V_p;

% Force extraction coordinates
syms alpha_f(t) alpha_t(t) v_alpha_f(t) v_alpha_t(t) p_alpha_f(t) p_alpha_t(t);
syms delta_y delta_z v_delta_y v_delta_z p_delta_y p_delta_z;

% Generalized coordinates and aliases
syms q_1(t) q_2(t) q_3(t) v_q_1 v_q_2 v_q_3 p_q_1 p_q_2 p_q_3;
syms q_4(t) q_5(t) v_q_4 v_q_5 p_q_4 p_q_5; % these are extra coordinates in order to extract GRF
q_dot_1 = diff(q_1, t); q_dot_2 = diff(q_2, t); q_dot_3 = diff(q_3, t);
q_dot_4 = diff(q_4, t); q_dot_5 = diff(q_5, t); 

% syms g;
g = 9.81;

syms L T V;

pi = sym(pi);

% Encode natives into generals
phi_p = q_1;
% phi_t = q_2;
% phi_b = q_3;
alpha_t = q_2;
alpha_f = q_3;
phi_f = q_1 + gamma_p; % femur-podial parallelism;
% y_p = q_4;
% z_p = q_5;
delta_y = q_4;
delta_z = q_5;

% alpha_f = phi_f - phi_b;
% alpha_t = phi_t - phi_f;
phi_b = phi_f - alpha_f;
phi_t = phi_f + (alpha_t - pi);

% Encode natives into generals -- kinematics

y_p = delta_y + (r_p*cos(phi_p) + c_p*cos(phi_p+beta_p));
z_p = delta_z + (r_p*sin(phi_p) + c_p*sin(phi_p+beta_p));

y_t = y_p - c_p*cos(phi_p+beta_p) + r_t*cos(phi_t) + c_t*cos(phi_t+beta_t);
z_t = z_p - c_p*sin(phi_p+beta_p) + r_t*sin(phi_t) + c_t*sin(phi_t+beta_t);

y_f = y_t - c_t*cos(phi_t+beta_t) + r_f*cos(phi_f) + c_f*cos(phi_f+beta_f);
z_f = z_t - c_t*sin(phi_t+beta_t) + r_f*sin(phi_f) + c_f*sin(phi_f+beta_f);

y_b = y_f - c_f*cos(phi_f+beta_f) + r_b*cos(phi_b) + c_b*cos(phi_b+beta_b);
z_b = z_f - c_f*sin(phi_f+beta_f) + r_b*sin(phi_b) + c_b*sin(phi_b+beta_b);

% velocities in E-L aliased coords
old = [q_dot_1, q_dot_2, q_dot_3, q_1, q_2, q_3, q_dot_4, q_dot_5, q_4, q_5];
new = [v_q_1, v_q_2, v_q_3, p_q_1, p_q_2, p_q_3, v_q_4, v_q_5, p_q_4, p_q_5];

y_dot_p = subs(diff(y_p, t),old, new);
z_dot_p = subs(diff(z_p, t),old, new);
phi_dot_p = subs(diff(phi_p, t),old, new);

y_dot_t = subs(diff(y_t, t),old, new);
z_dot_t = subs(diff(z_t, t),old, new);
phi_dot_t = subs(diff(phi_t, t),old, new);

y_dot_f = subs(diff(y_f, t),old, new);
z_dot_f = subs(diff(z_f, t),old, new);
phi_dot_f = subs(diff(phi_f, t),old, new);

y_dot_b = subs(diff(y_b, t),old, new);
z_dot_b = subs(diff(z_b, t),old, new);
phi_dot_b = subs(diff(phi_b, t),old, new);

alpha_dot_f = diff(alpha_f, t);
alpha_dot_t = diff(alpha_t, t);
delta_dot_y = diff(delta_y, t);
delta_dot_z = diff(delta_z, t);

% Energy terms in E-L aliased coords
T_b = (1/2) * ( M_b*(y_dot_b^2 + z_dot_b^2) + J_b*(phi_dot_b^2) );
V_b = M_b * g * z_b;
T_f = (1/2) * ( M_f*(y_dot_f^2 + z_dot_f^2) + J_f*(phi_dot_f^2) );
V_f = M_f * g * z_f;
T_t = (1/2) * ( M_t*(y_dot_t^2 + z_dot_t^2) + J_t*(phi_dot_t^2) );
V_t = M_t * g * z_t;
T_p = (1/2) * ( M_p*(y_dot_p^2 + z_dot_p^2) + J_p*(phi_dot_p^2) );
V_p = M_p * g * z_p;

T = T_b + T_f + T_t + T_p;
V = V_b + V_f + V_t + V_p;

% Write Lagrange energy
L = T - V;

syms F_y F_z tau_f tau_t;

% L = subs(L, [r_b, c_b, beta_b, M_b, J_b, r_f, c_f, beta_f, M_f, J_f, r_t, c_t, beta_t, M_t, J_t, r_p, c_p, beta_p, M_p, J_p],...
% [r_b_in, c_b_in, beta_b_in, M_b_in, J_b_in, r_f_in, c_f_in, beta_f_in, M_f_in, J_f_in, r_t_in, c_t_in, beta_t_in, M_t_in, J_t_in, r_p_in, c_p_in, beta_p_in, M_p_in, J_p_in]);

% de-alias
old = [v_q_1, v_q_2, v_q_3, p_q_1, p_q_2, p_q_3, v_q_4, v_q_5, p_q_4, p_q_5];
new = [q_dot_1, q_dot_2, q_dot_3, q_1, q_2, q_3, q_dot_4, q_dot_5, q_4, q_5];
L_sub = subs(L, old, new);

% derivatives
dLdq_1 = diff(L, v_q_1);
dLdq_1_sub = subs(dLdq_1, old, new);
EL1_lhs = diff(dLdq_1_sub, t);
EL1_rhs = functionalDerivative(L_sub,q_1)...
    + F_y*functionalDerivative(delta_y, q_1)...
    + F_z*functionalDerivative(delta_z, q_1);
EL1 = EL1_lhs - EL1_rhs == 0;

dLdq_2 = diff(L, v_q_2);
dLdq_2_sub = subs(dLdq_2, old, new);
EL2_lhs = diff(dLdq_2_sub, t);
EL2_rhs = functionalDerivative(L_sub,q_2)...
    + F_y*functionalDerivative(delta_y, q_2)...
    + F_z*functionalDerivative(delta_z, q_2);
EL2 = EL2_lhs - EL2_rhs == tau_t - B_t*diff(q_2,t);

dLdq_3 = diff(L, v_q_3);
dLdq_3_sub = subs(dLdq_3, old, new);
EL3_lhs = diff(dLdq_3_sub, t);
EL3_rhs = functionalDerivative(L_sub,q_3)...
    + F_y*functionalDerivative(delta_y, q_3)...
    + F_z*functionalDerivative(delta_z, q_3);
EL3 = EL3_lhs - EL3_rhs == tau_f - B_f*diff(q_3,t);

dLdq_4 = diff(L, v_q_4);
dLdq_4_sub = subs(dLdq_4, old, new);
EL4_lhs = diff(dLdq_4_sub, t);
EL4_rhs = functionalDerivative(L_sub,q_4)...
    + F_y*functionalDerivative(delta_y, q_4)...
    + F_z*functionalDerivative(delta_z, q_4);
EL4 = EL4_lhs - EL4_rhs == 0;

dLdq_5 = diff(L, v_q_5);
dLdq_5_sub = subs(dLdq_5, old, new);
EL5_lhs = diff(dLdq_5_sub, t);
EL5_rhs = functionalDerivative(L_sub,q_5)...
    + F_y*functionalDerivative(delta_y, q_5)...
    + F_z*functionalDerivative(delta_z, q_5);
EL5 = EL5_lhs - EL5_rhs == 0;

% apply constraint knowledge

old = {q_4, q_5,...
    q_dot_4, q_dot_5,...
    diff(q_dot_5, t), diff(q_dot_5, t)};
% new = {(r_p*cos(phi_p) + c_p*cos(phi_p+beta_p)), (r_p*sin(phi_p) + c_p*sin(phi_p+beta_p)),...
%     diff(r_p*cos(phi_p) + c_p*cos(phi_p+beta_p), t), diff(r_p*sin(phi_p) + c_p*sin(phi_p+beta_p), t),...
%     diff(diff(r_p*cos(phi_p) + c_p*cos(phi_p+beta_p), t), t), diff(diff(r_p*sin(phi_p) + c_p*sin(phi_p+beta_p), t), t)};
new = {0, 0, 0, 0, 0, 0};

EL1 = subs(EL1, old, new);
EL2 = subs(EL2, old, new);
EL3 = subs(EL3, old, new);
EL4 = subs(EL4, old, new);
EL5 = subs(EL5, old, new);

F_y_sol = isolate(EL4, F_y);
F_z_sol = isolate(EL5, F_z);

EL1 = subs(EL1, {lhs(F_y_sol), lhs(F_z_sol)}, {rhs(F_y_sol), rhs(F_z_sol)});
EL2 = subs(EL2, {lhs(F_y_sol), lhs(F_z_sol)}, {rhs(F_y_sol), rhs(F_z_sol)});
EL3 = subs(EL3, {lhs(F_y_sol), lhs(F_z_sol)}, {rhs(F_y_sol), rhs(F_z_sol)});

% EL1 = isolate(simplify(EL1), diff(diff(q_1, t), t))
% EL2 = isolate(simplify(EL2), diff(diff(q_2, t), t))
% EL3 = isolate(simplify(EL3), diff(diff(q_3, t), t))
% EL4 = isolate(simplify(EL4), diff(diff(q_4, t), t))
% EL5 = isolate(simplify(EL5), diff(diff(q_5, t), t))

EL1 = isolate((EL1), diff(diff(q_1, t), t));
EL2 = isolate((EL2), diff(diff(q_2, t), t));
EL3 = isolate((EL3), diff(diff(q_3, t), t));


% Create vector field
[VF,Ysubs] = odeToVectorField(EL1, EL2, EL3);

% Note: odeToVectorField does not preserve the order of the generalized
% coordinates (see documentation). Therefore,...
% Sort state vector to determine how re-ordering was done
[~,ind] = sort(Ysubs);

% Find indices of generalized coordinates and velocities
numcoor = 3;
indq    = ind(numcoor+1:2*numcoor);
indqqd  = [indq'; indq'+1];
% Un-do re-ordering of generalized coordinates and velocities
exp_str = ['[Y[kk] $ kk = 1..',num2str(2*numcoor),']'];
Y       = evalin(symengine,exp_str);
VF      = subs(VF,Y,Y(indqqd(:)));
% Un-do re-ordering of vector field equations
VF      = VF(indqqd(:));

VF(2) = simplify(VF(2));
VF(4) = simplify(VF(4));
VF(6) = simplify(VF(6));
% VF(4)
% VF(6)

%%
Q_vars = [tau_t, tau_f];
% 
% pars = {r_b, c_b, beta_b, M_b, J_b, gamma_b,...
% r_f, c_f, beta_f, M_f, J_f, gamma_f,...
% r_t, c_t, beta_t, M_t, J_t, gamma_t,...
% r_p, c_p, beta_p, M_p, J_p, gamma_p};

s_fileName = "planar_dynamics_block";
s_blockName = strcat(s_fileName,'/',"leg_planar");
sysName = "leg_planar";
% new_system  (s_fileName)
open_system  (s_fileName)
% add_block('built-in/SubSystem',s_fileName, 'MakeNameUnique', 'on')
matlabFunctionBlock(s_blockName,VF,...
    'functionName', sysName,...
    'vars',       [{'t','Y'}, Q_vars],...
    'outputs',     {'dYdt'});
save_system (s_fileName)
% close_system(s_fileName)