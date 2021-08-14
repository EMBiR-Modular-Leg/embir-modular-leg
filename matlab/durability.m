%% Load Data
addpath("\\engin-storage.m.storage.umich.edu\engin-storage\ursk\windat.v2\Documents\dynamometer-data");
datafiles = ["dynamometer_test_29_06_2021_15-12-17.csv",
    "dynamometer_test_29_06_2021_16-05-44.csv",
    "dynamometer_test_30_06_2021_14-50-50.csv",
    "dynamometer_test_08_07_2021_13-13-56.csv",
    "dynamometer_test_08_07_2021_14-39-44.csv"];

ii = 5;
datafile = datafiles(ii);
ds = load_durability(datafile);

%%

v_ampl_mask = abs(ds.a1_v) > 1;
v1 = ds.a1_v(v_ampl_mask);
v2 = ds.a2_v(v_ampl_mask);
figure; plot((v1+v2)./v1);

%%
addpath("\\engin-storage.m.storage.umich.edu\engin-storage\ursk\windat.v2\Documents\dynamometer-data");
plain_barrel = "dynamometer-data\dynamometer_test_15_07_2021_12-24-52.csv";
% [time_plain, temp_plain, tpeak_plain, ~] = thermal_step(plain_barrel, 1, 59);
plain_peaks = thermal_step(plain_barrel, 59);

slots_barrel = "dynamometer-data\dynamometer_test_15_07_2021_12-52-33.csv";
slots_peaks = thermal_step(slots_barrel, 59);

htsnk_barrel = "dynamometer-data\dynamometer_test_15_07_2021_14-47-08.csv";
htsnk_peaks = thermal_step(htsnk_barrel, 59);

minifan_barrel = "dynamometer-data\dynamometer_test_20_07_2021_17-20-05.csv";
minifan_peaks = thermal_step(minifan_barrel, 59);

minifanb_barrel = "dynamometer-data\dynamometer_test_21_07_2021_10-34-24.csv";
minifanb_peaks = thermal_step(minifanb_barrel, 59);

dblfan_barrel = "dynamometer-data\dynamometer_test_21_07_2021_11-11-17.csv";
dblfan_peaks = thermal_step(dblfan_barrel, 59);

dblfanb_barrel = "dynamometer-data\dynamometer_test_29_07_2021_00-28-56.csv";
dblfanb_peaks = thermal_step(dblfanb_barrel, 59);

dblfanc_barrel = "dynamometer-data\dynamometer_test_29_07_2021_00-42-02.csv";
dblfanc_peaks = thermal_step(dblfanc_barrel, 59);

dblfand_fanoff_barrel = "dynamometer-data\dynamometer_test_29_07_2021_08-30-40.csv";
dblfand_fanoff_peaks = thermal_step(dblfand_fanoff_barrel, 59);

%%
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');

figure;
hold on;

plot_peak_data(htsnk_peaks, 'k:', "htsnk")
plot_peak_data(dblfan_peaks, 'b:', "dblfan tape")

plot_peak_data(dblfanb_peaks, 'r', "dblfanb")
plot_peak_data(dblfanc_peaks, 'r:', "dblfanc")

plot_peak_data(dblfand_fanoff_peaks, 'm:', "dblfand fans off")

xlabel("time [s]");
ylabel("motor temp [$^O$C]");
xlim([-300, 150]);
title("torque step input thermal experiment");
legend();
hold off;

%%

% robocopy D:\dynamometer-data\ \\engin-storage.m.storage.umich.edu\engin-storage\ursk\windat.v2\Documents\dynamometer-data\ /s /z 
addpath("\\engin-storage.m.storage.umich.edu\engin-storage\ursk\windat.v2\Documents\dynamometer-data");

datafile = "dynamometer-data\dynamometer_test_16_07_2021_13-44-54.csv";

ds = load_durability(datafile);

%%
figure;
plot(ds.time, ds.a2_v_cmd, ds.time, ds.a2_v, ds.time, ds.ina1_v);
legend(["v cmd","v","voltage"]);

%%
alpha = 0.03;
figure;
hold on;
s3 = scatter((ds.a2_v_cmd), (ds.a1_t_cmd),'.');
s3.MarkerFaceAlpha = alpha;
s3.MarkerEdgeAlpha = alpha;

s1 = scatter((ds.a1_v), (ds.a1_t),'.');
s1.MarkerFaceAlpha = alpha;
s1.MarkerEdgeAlpha = alpha;

s2 = scatter((ds.a2_v), (ds.a2_t),'.');
s2.MarkerFaceAlpha = alpha;
s2.MarkerEdgeAlpha = alpha;

legend(["a1","a2"]);
xlabel("velocity (rad/s)");
ylabel("torque (Nm)");
hold off;

function ds = load_durability(datafile)
    ds = struct;
    data_table = readtable(datafile,'PreserveVariableNames',true,...
        'Delimiter',',');
    headers = data_table.Properties.VariableNames;

    time_idx = find(ismember(headers,"time [s]"));
    a1_p_cmd_idx = find(ismember(headers,"a1 position cmd [rad]"));
    a1_v_cmd_idx = find(ismember(headers,"a1 velocity cmd [rad/s]"));
    a1_t_cmd_idx = find(ismember(headers,"a1 ff torque cmd [Nm]"));
    a1_p_idx = find(ismember(headers,"a1 position [rad]"));
    a1_v_idx = find(ismember(headers,"a1 velocity [rad/s]"));
    a1_t_idx = find(ismember(headers,"a1 torque [Nm]"));
    c1_mode_idx = find(ismember(headers,"c1 mode"));
    c1_p_idx = find(ismember(headers,"c1 position [rev]"));
    c1_v_idx = find(ismember(headers,"c1 velocity [Hz]"));
    c1_t_idx = find(ismember(headers,"c1 torque [Nm]"));
    c1_temp_idx = find(ismember(headers,"c1 temp [C]"));
    c1_fault_idx = find(ismember(headers,"c1 fault"));
    a2_p_cmd_idx = find(ismember(headers,"a2 position cmd [rad]"));
    a2_v_cmd_idx = find(ismember(headers,"a2 velocity cmd [rad/s]"));
    a2_t_cmd_idx = find(ismember(headers,"a2 ff torque cmd [Nm]"));
    a2_p_idx = find(ismember(headers,"a2 position [rad]"));
    a2_v_idx = find(ismember(headers,"a2 velocity [rad/s]"));
    a2_t_idx = find(ismember(headers,"a2 torque [Nm]"));
    c2_mode_idx = find(ismember(headers,"c2 mode"));
    c2_p_idx = find(ismember(headers,"c2 position [rev]"));
    c2_v_idx = find(ismember(headers,"c2 velocity [Hz]"));
    c2_t_idx = find(ismember(headers,"c2 torque [Nm]"));
    c2_temp_idx = find(ismember(headers,"c2 temp [C]"));
    c2_fault_idx = find(ismember(headers,"c2 fault"));
    ts_idx = find(ismember(headers,"trs605-5 torque [Nm]"));
    m_temp_idx = find(ismember(headers,"motor temp [C]"));
    h_temp_idx = find(ismember(headers,"housing temp [C]"));
    ina1_v_idx = find(ismember(headers,"ina1 voltage [V]"));
    ina1_i_idx = find(ismember(headers,"ina1 current [A]"));
    ina1_p_idx = find(ismember(headers,"ina1 power [W]"));
    ina2_v_idx = find(ismember(headers,"ina2 voltage [V]"));
    ina2_i_idx = find(ismember(headers,"ina2 current [A]"));
    ina2_p_idx = find(ismember(headers,"ina2 power [W]"));
    a_id_idx = find(ismember(headers,"actuator_a_id"));
    dts_idx = find(ismember(headers,"durability test mode"));

    ds.time = table2array(data_table(1:end, time_idx));  
    ds.a1_p_cmd = table2array(data_table(1:end, a1_p_cmd_idx));  
    ds.a1_v_cmd = table2array(data_table(1:end, a1_v_cmd_idx));  
    ds.a1_t_cmd = table2array(data_table(1:end, a1_t_cmd_idx));  
    ds.a1_p = table2array(data_table(1:end, a1_p_idx));  
    ds.a1_v = table2array(data_table(1:end, a1_v_idx));  
    ds.a1_t = table2array(data_table(1:end, a1_t_idx));  
    ds.c1_mode = table2array(data_table(1:end, c1_mode_idx));  
    ds.c1_p = table2array(data_table(1:end, c1_p_idx));  
    ds.c1_v = table2array(data_table(1:end, c1_v_idx));  
    ds.c1_t = table2array(data_table(1:end, c1_t_idx));  
    ds.c1_temp = table2array(data_table(1:end, c1_temp_idx));  
    ds.c1_fault = table2array(data_table(1:end, c1_fault_idx));  
    ds.a2_p_cmd = table2array(data_table(1:end, a2_p_cmd_idx));  
    ds.a2_v_cmd = table2array(data_table(1:end, a2_v_cmd_idx));  
    ds.a2_t_cmd = table2array(data_table(1:end, a2_t_cmd_idx));  
    ds.a2_p = table2array(data_table(1:end, a2_p_idx));  
    ds.a2_v = table2array(data_table(1:end, a2_v_idx));  
    ds.a2_t = table2array(data_table(1:end, a2_t_idx));  
    ds.c2_mode = table2array(data_table(1:end, c2_mode_idx));  
    ds.c2_p = table2array(data_table(1:end, c2_p_idx));  
    ds.c2_v = table2array(data_table(1:end, c2_v_idx));  
    ds.c2_t = table2array(data_table(1:end, c2_t_idx));  
    ds.c2_temp = table2array(data_table(1:end, c2_temp_idx));  
    ds.c2_fault = table2array(data_table(1:end, c2_fault_idx));  
    ds.ts = table2array(data_table(1:end, ts_idx));  
    ds.m_temp = table2array(data_table(1:end, m_temp_idx));  
    ds.h_temp = table2array(data_table(1:end, h_temp_idx));  
    ds.ina1_v = table2array(data_table(1:end, ina1_v_idx));  
    ds.ina1_i = table2array(data_table(1:end, ina1_i_idx));  
    ds.ina1_p = table2array(data_table(1:end, ina1_p_idx));  
    ds.ina2_v = table2array(data_table(1:end, ina2_v_idx));  
    ds.ina2_i = table2array(data_table(1:end, ina2_i_idx));  
    ds.ina2_p = table2array(data_table(1:end, ina2_p_idx));  
    ds.a_id = table2array(data_table(1:end, a_id_idx));  
    ds.dts = table2array(data_table(1:end, dts_idx));  
end

function peak_data = thermal_step(filename, peakheight)

ds = load_durability(filename);
time = ds.time;
temp = ds.m_temp;

[pks, locs] = findpeaks(temp, 'MinPeakHeight',peakheight, 'MinPeakDistance', 3000);

peak_data = {};
for peak_num = 1:length(locs)
    center_idx = locs(peak_num);
    tpeak = time(center_idx);
    time = time-tpeak;
    temp = smoothdata(temp,'movmean',500);
    row = {time, temp, tpeak, center_idx};
    peak_data{peak_num} = row;
end

end

function plot_peak_data(peak_data, linspec, label_root)

for ii = 1:length(peak_data)
    row = peak_data{ii};
    time = row{1};
    temp = row{2};
    plot(time, temp,linspec, 'LineWidth',2,'DisplayName',label_root+ii);
end

end