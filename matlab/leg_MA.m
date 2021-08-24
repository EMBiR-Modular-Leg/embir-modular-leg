set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');

datafile = "MA_3L.csv";

ds = load_MA(datafile);
min = 0;
max = 0.5;

figure;
cb = colorbar();
pointsize = 30;
subplot(3,2,1)
scatter(ds.x_sample./1000, ds.y_sample./1000, pointsize, ds.Fmag/(sqrt(2)), 'filled');
daspect([1 1 1])
caxis([min max])
xlabel('y [m]'); ylabel('z [m]');
title("ratio of GRF magnitude to torque input for 3L leg")
colorbar;

subplot(3,2,3)
scatter(ds.x_sample./1000, ds.y_sample./1000, pointsize, abs(ds.Fx)/(sqrt(2)), 'filled');
daspect([1 1 1])
caxis([min max])
xlabel('y [m]'); ylabel('z [m]');
title("ratio of GRF hoirzontal component to torque input for 3L leg")
colorbar;

subplot(3,2,5)
scatter(ds.x_sample./1000, ds.y_sample./1000, pointsize, abs(ds.Fy)/(sqrt(2)), 'filled');
daspect([1 1 1])
caxis([min max])
xlabel('y [m]'); ylabel('z [m]');
title("ratio of GRF vertical component to torque input for 3L leg")
colorbar;

datafile = "MA_2L.csv";
ds = load_MA(datafile);

subplot(3,2,2)
scatter(ds.x_sample./1000, ds.y_sample./1000, pointsize, ds.Fmag/(sqrt(2)), 'filled');
daspect([1 1 1])
caxis([min max])
xlabel('y [m]'); ylabel('z [m]');
title("ratio of GRF magnitude to torque input for 2L leg")
colorbar;

subplot(3,2,4)
scatter(ds.x_sample./1000, ds.y_sample./1000, pointsize, abs(ds.Fx)/(sqrt(2)), 'filled');
daspect([1 1 1])
caxis([min max])
xlabel('y [m]'); ylabel('z [m]');
title("ratio of GRF hoirzontal component to torque input for 2L leg")
colorbar;

subplot(3,2,6)
scatter(ds.x_sample./1000, ds.y_sample./1000, pointsize, abs(ds.Fy)/(sqrt(2)), 'filled');
daspect([1 1 1])
caxis([min max])
xlabel('y [m]'); ylabel('z [m]');
title("ratio of GRF vertical component to torque input for 2L leg")
colorbar;

function ds = load_MA(datafile)
    ds = struct;
    data_table = readtable(datafile,'PreserveVariableNames',true,...
        'Delimiter',',');
    headers = data_table.Properties.VariableNames;

    x_sample_idx = find(ismember(headers,"x_sample [mm]"));
    y_sample_idx = find(ismember(headers,"y_sample [mm]"));
    theta1_idx = find(ismember(headers,"theta1 [rad]"));
    theta2_idx = find(ismember(headers,"theta2 [rad]"));
    Fx_idx = find(ismember(headers,"Fx [N]"));
    Fy_idx = find(ismember(headers,"Fy [N]"));

    ds.x_sample = table2array(data_table(1:end, x_sample_idx));  
    ds.y_sample = table2array(data_table(1:end, y_sample_idx));  
    ds.theta1 = table2array(data_table(1:end, theta1_idx));  
    ds.theta2 = table2array(data_table(1:end, theta2_idx));  
    ds.Fx = table2array(data_table(1:end, Fx_idx));  
    ds.Fy = table2array(data_table(1:end, Fy_idx));
    ds.Fmag = (ds.Fx.^2 + ds.Fy.^2).^0.5;
end