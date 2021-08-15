%% make data

config_data = {};
grf_data = {};

rhsF_y_sol = simplify(rhs(F_y_sol));
rhsF_z_sol = simplify(rhs(F_z_sol));

tic;     % start timing
for id = 1:length(out.tout)
    [y, z] = kinematics(out.Y_out(id,:));
    [fy, fz] = grf(out.Y_out(id,:), rhsF_y_sol, rhsF_z_sol);
    
    config_data{id} = [y;z];
    grf_data{id} = [fy;fz];
    
    if(mod(id, 200) == 0) 
        fprintf("\r id = %d", id)
    end
end
fprintf('kinematics and grf calculation: %0.2f sec\n', toc);

%%
figure;

% [y, z] = kinematics(out.Y_out(1,:));
% [fy, fz] = grf(out.Y_out(1,:), F_y_sol, F_z_sol);
hh1 = plot(config_data{1}(1,:), config_data{1}(2,:), ...
      '.-', 'MarkerSize', 20, 'LineWidth', 2);
hold on;
hh2 = plot(grf_data{1}(1,:), grf_data{1}(2,:), ...
      'r-', 'MarkerSize', 20, 'LineWidth', 2);
hold off;
axis equal
L = 0.15;
axis([-2*L 2*L -2*L 2*L]);
ht = title(sprintf('Time: %0.2f sec, GRF = %0.2f', out.tout(1), 0));

tic;     % start timing
old_id = 1;
f_id = 1;
loops = 1:5:length(out.tout);
F(length(loops)) = struct('cdata',[],'colormap',[]);

v = VideoWriter('animation.mp4');
open(v);

for ii = loops
    id = ii;
%     [y, z] = kinematics(out.Y_out(id,:));
%     [fy, fz] = grf(out.Y_out(id,:), F_y_sol, F_z_sol);
%     set(hh1(1), 'XData', y  , 'YData', z);
    
    set(hh1(1), 'XData', config_data{id}(1,:)  , 'YData', config_data{id}(2,:));
    
%     set(hh2(2), 'XData', x(id, :)       , 'YData', y(id, :));
%     set(hh2(1), 'XData', fy  , 'YData', fz);
    
    set(hh2(1), 'XData', grf_data{id}(1,:)  , 'YData', grf_data{id}(2,:));
    
    set(ht, 'String', sprintf('Time: %0.2f sec, GRF = %0.2f kN', out.tout(id), norm( grf_data{id}(:,2) )));
    tstep = out.tout(id) - out.tout(old_id);
    drawnow;
%     pause(1*tstep);
    old_id = id;
    
    frame = getframe(gcf);
    writeVideo(v, frame);
    f_id = f_id + 1;
end
fprintf('Animation (Smart update): %0.2f sec\n', toc);
close(v);

%%


open(v);
writeVideo(v, F);
close(v);

function [y, z] = kinematics(Y)

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
    r_b = 0.0;
    c_b = 0.0; % centered mass
    beta_b = 0;
    M_b = 2*M_a + 0.5; % two actuators + carriage
    J_b = 1e-3;
    gamma_b = 0;

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

    r_p = 0.1;
    c_p = 0.03;
    beta_p = 0.75*pi;
    M_p = 0.07;
    J_p = 1e-4;
    gamma_p = atan2(l3_pll, l3_perp);


    phi_p = Y(1);
    alpha_t = Y(3);
    alpha_f = Y(5);
    
    phi_f = phi_p + gamma_p; % femur-podial parallelism;
    phi_b = phi_f - alpha_f;
    phi_t = phi_f + (alpha_t - pi);

    % Encode natives into generals -- kinematics

    y_p_j = r_p*cos(phi_p);
    z_p_j = r_p*sin(phi_p);

    y_t_j = y_p_j + r_t*cos(phi_t);
    z_t_j = z_p_j + r_t*sin(phi_t);

    y_f_j = y_t_j + r_f*cos(phi_f);
    z_f_j = z_t_j + r_f*sin(phi_f);

    y_b_j = y_f_j + r_b*cos(phi_b);
    z_b_j = z_f_j + r_b*sin(phi_b);
    
    y = [0, y_p_j, y_t_j, y_f_j, y_b_j];
    z = [0, z_p_j, z_t_j, z_f_j, z_b_j];
end

function [fy, fz] = grf(Y, rhsF_y_sol, rhsF_z_sol)
    syms t q_1(t) q_2(t) q_3(t);
    old = {q_1, q_2, q_3, diff(q_1, t), diff(q_2, t), diff(q_3, t)};
    new = {Y(1), Y(3), Y(5), Y(2), Y(4), Y(6)};
    fy = vpa(subs(rhsF_y_sol, old, new));
    fz = vpa(subs(rhsF_z_sol, old, new));
    fy = [0, fy/1000];
    fz = [0, fz/1000];
end