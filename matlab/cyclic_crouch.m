%% Load Data
addpath("\\engin-storage.m.storage.umich.edu\engin-storage\ursk\windat.v2\Documents\dynamometer-data");
addpath("\\engin-storage.m.storage.umich.edu\engin-storage\ursk\windat.v2\Documents\cpp_api_demo_data");

leg_fname_0_2Hz_ground = [   "leg_test_07_08_2021_14-57-49.csv";
                "leg_test_07_08_2021_14-58-34.csv";
                "leg_test_07_08_2021_14-59-20.csv" ];
            
leg_fname_0_5Hz_ground = [   "leg_test_07_08_2021_15-28-45.csv";
                "leg_test_07_08_2021_15-32-00.csv";
                "leg_test_07_08_2021_15-39-09.csv" ];
            
leg_fname_0_5Hz_air = [   "leg_test_07_08_2021_15-43-40.csv";
                "leg_test_07_08_2021_15-44-31.csv";
                "leg_test_07_08_2021_15-45-35.csv";
                "leg_test_07_08_2021_15-47-26.csv" ];
            
leg_data_1 = load_multiple_data(leg_fname_0_2Hz_ground);
leg_data_2 = load_multiple_data(leg_fname_0_5Hz_ground);
leg_data_3 = load_multiple_data(leg_fname_0_5Hz_air);

%% Plotting
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');

figure;
hold on;
leg_data = leg_data_2;
d1 = designfilt('lowpassiir','FilterOrder',4, ...
    'HalfPowerFrequency',0.05,'DesignMethod','butter');
for ii = 1:length(leg_data)
    plot(leg_data{ii}.time, filtfilt(d1, leg_data{ii}.a1_t), 'DisplayName',"femur torque "+ii);
    plot(leg_data{ii}.time, filtfilt(d1, leg_data{ii}.a2_t), 'DisplayName',"tibia torque "+ii);
end
xlabel("time [s]");
ylabel("torque [Nm]");
legend();
hold off;

function data_cells = load_multiple_data(datafiles)

data_cells = {};
for ii = 1:length(datafiles)
    data_cells{ii} = load_leg_data(datafiles(ii));
end
end

function a = fill_nans(a)
    a = fillmissing(a, 'previous');
    a = fillmissing(a, 'next');
end

function ds = load_leg_data(datafile)
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
    lfs_idx = find(ismember(headers,"leg fsm state"));

    ds.time = fill_nans(table2array(data_table(1:end, time_idx)));  
    ds.a1_p_cmd = fill_nans(table2array(data_table(1:end, a1_p_cmd_idx)));  
    ds.a1_v_cmd = fill_nans(table2array(data_table(1:end, a1_v_cmd_idx)));  
    ds.a1_t_cmd = fill_nans(table2array(data_table(1:end, a1_t_cmd_idx)));  
    ds.a1_p = fill_nans(table2array(data_table(1:end, a1_p_idx)));  
    ds.a1_v = fill_nans(table2array(data_table(1:end, a1_v_idx)));  
    ds.a1_t = fill_nans(table2array(data_table(1:end, a1_t_idx)));  
    ds.c1_mode = fill_nans(table2array(data_table(1:end, c1_mode_idx)));  
    ds.c1_p = fill_nans(table2array(data_table(1:end, c1_p_idx)));  
    ds.c1_v = fill_nans(table2array(data_table(1:end, c1_v_idx)));  
    ds.c1_t = fill_nans(table2array(data_table(1:end, c1_t_idx)));  
    ds.c1_temp = fill_nans(table2array(data_table(1:end, c1_temp_idx)));  
    ds.c1_fault = fill_nans(table2array(data_table(1:end, c1_fault_idx)));  
    ds.a2_p_cmd = fill_nans(table2array(data_table(1:end, a2_p_cmd_idx)));  
    ds.a2_v_cmd = fill_nans(table2array(data_table(1:end, a2_v_cmd_idx)));  
    ds.a2_t_cmd = fill_nans(table2array(data_table(1:end, a2_t_cmd_idx)));  
    ds.a2_p = fill_nans(table2array(data_table(1:end, a2_p_idx)));  
    ds.a2_v = fill_nans(table2array(data_table(1:end, a2_v_idx)));  
    ds.a2_t = fill_nans(table2array(data_table(1:end, a2_t_idx)));  
    ds.c2_mode = fill_nans(table2array(data_table(1:end, c2_mode_idx)));  
    ds.c2_p = fill_nans(table2array(data_table(1:end, c2_p_idx)));  
    ds.c2_v = fill_nans(table2array(data_table(1:end, c2_v_idx)));  
    ds.c2_t = fill_nans(table2array(data_table(1:end, c2_t_idx)));  
    ds.c2_temp = fill_nans(table2array(data_table(1:end, c2_temp_idx)));  
    ds.c2_fault = fill_nans(table2array(data_table(1:end, c2_fault_idx)));
    ds.lfs = fill_nans(table2array(data_table(1:end, lfs_idx)));
    
    ds.filename = datafile;
end
