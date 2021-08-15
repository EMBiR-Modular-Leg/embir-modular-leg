% RotaryPendulum_MLdemo    This demo illustrates an application of the
%                Euler-Lagrange equation. The system of differential
%                equations derived with the EulerLagrange tool is solved
%                using ode45. 
%                Note: If the Simulink 3D Animation Toolbox is available 
%                the dynamics of the system can be visualized in the VR
%                world.
%
% Copyright 2015-2016 The MathWorks, Inc.
clc;
% Solve DE numerically using ode45
% Set simulation time and initial state vector
tspan = [0 10];
X0    = [0 0 10 0]*pi/180;
% Set parameter values
I     = double(1);
m     = double(1);
l     = double(1);
R     = double(1);
g     = double(9.81);
tau   = double(0);    % No external input torque

% Use created .m file to solve DE
% fcn = @(t, Y) RotaryPendulem_ODE(t, Y, tau, I, m, l, R, g);
% [t, X]  = ode45(@RotaryPendulum_ODE,tspan,X0,[],tau,I,m,l,R,g);
[t, X]  = ode45(@(t, X)RotaryPendulum_ODE(t, X, tau,I,m,l,R,g),tspan,X0);

% dYdt = RotaryPendulum_ODE(t,Y,tau,I,m,l,Rl,g)

% Plot state vector
plot(t,X)
title('Rotary pendulum')
xlabel('t')
ylabel('X(t)')
legend('{\alpha}','d{\alpha}/dt','{\beta}','d{\beta}/dt')

% Start animation if VR toolbox is available
hasVR = license('test', 'virtual_reality_toolbox');
if ~hasVR
    msgbox('You do not seem to have the Simulink 3D Animation Toolbox.',...
           'Toolbox missing');
else
    % Open the VirtualReality world and display in figure
    world = vrworld('RotaryPendulum_VR.wrl');
    open(world);
    fig   = view(world, '-internal');
    
    % Define VR nodes to be able to rotate objects
    RotaryArm = vrnode(world, 'RotaryArm');
    Pendulum  = vrnode(world, 'Pendulum');
    
    % Loop through state vector and re-draw VR
    for ii = 1:size(X,1)
        RotaryArm.rotation = [0 1 0 X(ii,1)];
        Pendulum.rotation  = [0 0 1 X(ii,3)];
        vrdrawnow;
        pause(0.05);
    end
end % if statement
% End of script