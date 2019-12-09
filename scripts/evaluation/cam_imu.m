clear all
close all
clc

%% Read csv
filename = '/home/spadmin/datasets/versavis/all_cameras/output.csv';
delimiter = ',';
formatSpec = '%s%s%s%s%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN, 'ReturnOnError', false);
fclose(fileID);

%% format:
sensors = {'versavis', 'vi-sensor', 'realsense'};
tests = {'cam0', 'cam1', 'stereo'};
datasets = {'dynamic_1', 'dynamic_2', 'dynamic_3', 'dynamic_4', 'dynamic_5', 'dynamic_filt2_1', 'dynamic_filt2_2', 'dynamic_filt2_3', 'dynamic_filt4_1', 'dynamic_filt4_2', 'dynamic_filt4_3', 'dynamic_filt4_1ms_1', 'dynamic_filt4_1ms_2', 'dynamic_filt4_1ms_3', 'dynamic_filt4_3ms_1', 'dynamic_filt4_3ms_2', 'dynamic_filt4_3ms_3', 'dynamic_filt4_5ms_1', 'dynamic_filt4_5ms_2', 'dynamic_filt4_5ms_3'};
metrics = {'Reprojection error (cam0) [px]','Reprojection error (cam1) [px]','Gyroscope error (imu0) [rad/s]','Accelerometer error (imu0) [m/s^2]'};
timeshifts = {'timeshift cam0 to imu0: [s] (t_imu = t_cam + shift)', 'timeshift cam1 to imu0: [s] (t_imu = t_cam + shift)'};

%%
data = NaN(length(sensors),length(tests),length(datasets),length(metrics)*3 + length(timeshifts));
for i = 1:size(dataArray{1},1)
   sensor_id = find(strcmp(sensors, dataArray{2}(i)));
   test_id = find(strcmp(tests, dataArray{3}(i)));
   dataset_id = find(strcmp(datasets, dataArray{1}(i)));
   metric_id = find(strcmp(metrics, dataArray{4}(i)));
   timeshift_id = find(strcmp(timeshifts, dataArray{4}(i)));
   if metric_id
       data(sensor_id, test_id, dataset_id, (metric_id-1)*3+1:(metric_id-1)*3+3) = [dataArray{5}(i), dataArray{6}(i),dataArray{7}(i)]; 
   elseif timeshift_id
       data(sensor_id, test_id, dataset_id, length(metrics)*3 + timeshift_id) = dataArray{5}(i); 
   else
       error('Neither metric nor timeshift');
   end
end


%% Evaluation
fig=figure;
hold on;


color_1ms = [1 0.8 0];
color_3ms = [0.8 1 0];
color_5ms = [0.6 0.6 0.4];
%% Reprojection mean
subplot(1,4,1);
idx = 1;
scatter((idx-0.15)*ones(1,5), data(1,1,1:5,1),'or') % cam0 mean versavis
hold on;
scatter((idx-0.1)*ones(1,3), data(1,1,6:8,1),'om') % cam0 mean versavis
scatter((idx-0.05)*ones(1,3), data(1,1,9:11,1),'oy') % cam0 mean versavis
scatter((idx)*ones(1,3), data(1,1,12:14,1),'o', 'MarkerEdgeColor', color_1ms) % cam0 mean versavis
scatter((idx+0.05)*ones(1,3), data(1,1,15:17,1),'o', 'MarkerEdgeColor', color_3ms) % cam0 mean versavis
scatter((idx+0.1)*ones(1,3), data(1,1,18:20,1),'o', 'MarkerEdgeColor', color_5ms) % cam0 mean versavis
scatter((idx+0.15)*ones(1,length(datasets)), data(2,1,:,1),'ob') % cam0 mean visensor
scatter((idx+0.2)*ones(1,length(datasets)), data(3,1,:,1),'og') % cam0 mean realsense
scatter([],[],'ok')
scatter([],[],'xk')
legend('VersaVIS 0 AE','VersaVIS 2 AE','VersaVIS 4 AE','VersaVIS 4 1ms','VersaVIS 4 3ms','VersaVIS 4 5ms','VI-Sensor AE','RealSense AE','Cam0','Cam1');

scatter((idx-0.15)*ones(1,5), data(1,2,1:5,1),'xr') % cam1 mean versavis
scatter((idx-0.1)*ones(1,3), data(1,2,6:8,1),'xm') % cam1 mean versavis
scatter((idx-0.05)*ones(1,3), data(1,2,9:11,1),'xy') % cam1 mean versavis
scatter((idx)*ones(1,3), data(1,2,12:14,1),'x', 'MarkerEdgeColor', color_1ms) % cam1 mean versavis
scatter((idx+0.05)*ones(1,3), data(1,2,15:17,1),'x', 'MarkerEdgeColor', color_3ms) % cam1 mean versavis
scatter((idx+0.1)*ones(1,3), data(1,2,18:20,1),'x', 'MarkerEdgeColor', color_5ms) % cam1 mean versavis
scatter((idx+0.15)*ones(1,length(datasets)), data(2,2,:,1),'xb') % cam1 mean visensor
scatter((idx+0.2)*ones(1,length(datasets)), data(3,2,:,1),'xg') % cam1 mean realsense

%% Reprojection std
idx = 2;
scatter((idx-0.15)*ones(1,5), data(1,1,1:5,3),'or') % cam0 std versavis
hold on;
scatter((idx-0.1)*ones(1,3), data(1,1,6:8,3),'om') % cam0 std versavis
scatter((idx-0.05)*ones(1,3), data(1,1,9:11,3),'oy') % cam0 std versavis
scatter((idx)*ones(1,3), data(1,1,12:14,3),'o', 'MarkerEdgeColor', color_1ms) % cam0 mean versavis
scatter((idx+0.05)*ones(1,3), data(1,1,15:17,3),'o', 'MarkerEdgeColor', color_3ms) % cam0 mean versavis
scatter((idx+0.1)*ones(1,3), data(1,1,18:20,3),'o', 'MarkerEdgeColor', color_5ms) % cam0 mean versavis
scatter((idx+0.15)*ones(1,length(datasets)), data(2,1,:,3),'ob') % cam0 std visensor
scatter((idx+0.2)*ones(1,length(datasets)), data(3,1,:,3),'og') % cam0 std realsense
scatter((idx-0.15)*ones(1,5), data(1,2,1:5,3),'xr') % cam1 std versavis
scatter((idx-0.1)*ones(1,3), data(1,2,6:8,3),'xm') % cam1 std versavis
scatter((idx-0.05)*ones(1,3), data(1,2,9:11,3),'xy') % cam1 std versavis
scatter((idx)*ones(1,3), data(1,2,12:14,3),'x', 'MarkerEdgeColor', color_1ms) % cam1 mean versavis
scatter((idx+0.05)*ones(1,3), data(1,2,15:17,3),'x', 'MarkerEdgeColor', color_3ms) % cam1 mean versavis
scatter((idx+0.1)*ones(1,3), data(1,2,18:20,3),'x', 'MarkerEdgeColor', color_5ms) % cam1 mean versavis
scatter((idx+0.15)*ones(1,length(datasets)), data(2,2,:,3),'xb') % cam1 std visensor
scatter((idx+0.2)*ones(1,length(datasets)), data(3,2,:,3),'xg') % cam1 std realsense
xticks([1,2]);
xticklabels({'Mean', 'Std'})
xlim([0.5, 2.5]);
ylim([0, 0.2]);
ylabel('[px]')
xlabel('Reprojection error')

%% Gyro mean
subplot(1,4,2);
idx = 0.85;
scatter(idx*ones(1,5), data(1,1,1:5,7),'or') % cam0 gyro mean versavis
idx = idx + 0.05;
hold on;
scatter(idx*ones(1,3), data(1,1,6:8,7),'om') % cam0 gyro mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,9:11,7),'oy') % cam0 gyro mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,12:14,7),'o', 'MarkerEdgeColor', color_1ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,15:17,7),'o', 'MarkerEdgeColor', color_3ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,18:20,7),'o', 'MarkerEdgeColor', color_5ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(2,1,:,7),'ob') % cam0 gyro mean visensor
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(3,1,:,7),'og') % cam0 gyro mean realsense
idx = 0.85;
scatter(idx*ones(1,5), data(1,2,1:5,7),'xr') % cam1 gyro mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,6:8,7),'xm') % cam1 gyro mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,9:11,7),'xy') % cam1 gyro mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,12:14,7),'x', 'MarkerEdgeColor', color_1ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,15:17,7),'x', 'MarkerEdgeColor', color_3ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,18:20,7),'x', 'MarkerEdgeColor', color_5ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(2,2,:,7),'xb') % cam1 gyro mean visensor
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(3,2,:,7),'xg') % cam1 gyro mean realsense

%% Gyro std
idx = 1.85;
scatter(idx*ones(1,5), data(1,1,1:5,9),'or') % cam0 gyro std versavis
hold on;
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,6:8,9),'om') % cam0 gyro std versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,9:11,9),'oy') % cam0 gyro std versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,12:14,9),'o', 'MarkerEdgeColor', color_1ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,15:17,9),'o', 'MarkerEdgeColor', color_3ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,18:20,9),'o', 'MarkerEdgeColor', color_5ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(2,1,:,9),'ob') % cam0 gyro std visensor
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(3,1,:,9),'og') % cam0 gyro std realsense
idx = 1.85;
scatter(idx*ones(1,5), data(1,2,1:5,9),'xr') % cam1 gyro std versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,6:8,9),'xm') % cam1 gyro std versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,9:11,9),'xy') % cam1 gyro std versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,12:14,9),'x', 'MarkerEdgeColor', color_1ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,15:17,9),'x', 'MarkerEdgeColor', color_3ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,18:20,9),'x', 'MarkerEdgeColor', color_5ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(2,2,:,9),'xb') % cam1 gyro std visensor
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(3,2,:,9),'xg') % cam1 gyro std realsense

xticks([1,2])
xticklabels({'Mean', 'Std'})
xlim([0.5, 2.5]);
ylabel('[rad/s]')
xlabel('Gyroscope error')

%% Accel mean
subplot(1,4,3);
idx = 0.85;
scatter(idx*ones(1,5), data(1,1,1:5,10),'or') % cam0 accel mean versavis
hold on;
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,6:8,10),'om') % cam0 accel mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,9:11,10),'oy') % cam0 accel mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,12:14,10),'o', 'MarkerEdgeColor', color_1ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,15:17,10),'o', 'MarkerEdgeColor', color_3ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,18:20,10),'o', 'MarkerEdgeColor', color_5ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(2,1,:,10),'ob') % cam0 accel mean visensor
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(3,1,:,10),'og') % cam0 accel mean realsense
idx = 0.85;
scatter(idx*ones(1,5), data(1,2,1:5,10),'xr') % cam1 accel mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,6:8,10),'xm') % cam1 accel mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,9:11,10),'xy') % cam1 accel mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,12:14,10),'x', 'MarkerEdgeColor', color_1ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,15:17,10),'x', 'MarkerEdgeColor', color_3ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,18:20,10),'x', 'MarkerEdgeColor', color_5ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(2,2,:,10),'xb') % cam1 accel mean visensor
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(3,2,:,10),'xg') % cam1 accel mean realsense

%% Accel std
idx = 1.85;
scatter(idx*ones(1,5), data(1,2,1:5,12),'or') % cam0 accel std versavis
hold on;
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,6:8,12),'om') % cam0 accel std versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,9:11,12),'oy') % cam0 accel std versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,12:14,12),'o', 'MarkerEdgeColor', color_1ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,15:17,12),'o', 'MarkerEdgeColor', color_3ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,1,18:20,12),'o', 'MarkerEdgeColor', color_5ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(2,2,:,12),'ob') % cam0 accel std visensor
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(3,2,:,12),'og') % cam0 accel std realsense
idx = 1.85;
scatter(idx*ones(1,5), data(1,2,1:5,12),'xr') % cam1 accel std versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,6:8,12),'xm') % cam1 accel std versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,9:11,12),'xy') % cam1 accel std versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,12:14,12),'x', 'MarkerEdgeColor', color_1ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,15:17,12),'x', 'MarkerEdgeColor', color_3ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), data(1,2,18:20,12),'x', 'MarkerEdgeColor', color_5ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(2,2,:,12),'xb') % cam1 accel std visensor
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), data(3,2,:,12),'xg') % cam1 accel std realsense
xticks([1,2]);
xticklabels({'Mean', 'Std'})
xlim([0.5, 2.5]);
ylabel('[m/s^2]');
xlabel('Acceleration error')

%% Time offset
subplot(1,4,4);
idx = 0.85;
scatter(idx*ones(1,5), 1000*data(1,1,1:5,13),'or') % cam0 time_off versavis
hold on;
idx = idx + 0.05;
scatter(idx*ones(1,3), 1000*data(1,1,6:8,13),'om') % cam0 time_off versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), 1000*data(1,1,9:11,13),'oy') % cam0 time_off versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), 1000*data(1,1,12:14,13),'o', 'MarkerEdgeColor', color_1ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), 1000*data(1,1,15:17,13),'o', 'MarkerEdgeColor', color_3ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), 1000*data(1,1,18:20,13),'o', 'MarkerEdgeColor', color_5ms) % cam0 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), 1000*data(2,1,:,13),'ob') % cam0 time_off visensor
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), 1000*data(3,1,:,13),'og') % cam0 time_off realsense
idx = 0.85;
scatter(idx*ones(1,5), 1000*data(1,2,1:5,13),'xr') % cam1 time_off versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), 1000*data(1,2,6:8,13),'xm') % cam1 time_off versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), 1000*data(1,2,9:11,13),'xy') % cam1 time_off versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), 1000*data(1,2,12:14,13),'x', 'MarkerEdgeColor', color_1ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), 1000*data(1,2,15:17,13),'x', 'MarkerEdgeColor', color_3ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,3), 1000*data(1,2,18:20,13),'x', 'MarkerEdgeColor', color_5ms) % cam1 mean versavis
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), 1000*data(2,2,:,13),'xb') % cam1 time_off visensor
idx = idx + 0.05;
scatter(idx*ones(1,length(datasets)), 1000*data(3,2,:,13),'xg') % cam1 time_off realsense
xticklabels('')
ylabel('[ms]')
xlabel('Time offset')

%% Output as table
fprintf('\t\t reproj_mean \t reproj_std \t gyro_mean \t gyro_std \t accel_mean \t accel_std \t time_offset \t time_offset_std \n');
camera = 1; index = 1:5; name = 'VersaVIS 0 ae';
fprintf('%s \t %d \t %f \t %f \t%f \t%f \t%f \t%f \t%f \t%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 1; index = 6:8; name = 'VersaVIS 2 ae';
fprintf('%s \t %d \t %f \t %f \t%f \t%f \t%f \t%f \t%f \t%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 1; index = 9:11; name = 'VersaVIS 4 ae';
fprintf('%s \t %d \t %f \t %f \t%f \t%f \t%f \t%f \t%f \t%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 1; index = 12:14; name = 'VersaVIS 4 1ms';
fprintf('%s \t %d \t %f \t %f \t%f \t%f \t%f \t%f \t%f \t%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 1; index = 15:17; name = 'VersaVIS 4 3ms';
fprintf('%s \t %d \t %f \t %f \t%f \t%f \t%f \t%f \t%f \t%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 1; index = 18:20; name = 'VersaVIS 4 5ms';
fprintf('%s \t %d \t %f \t %f \t%f \t%f \t%f \t%f \t%f \t%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 2; index = 1:20; name = 'VI-Sensor';
fprintf('%s \t %d \t %f \t %f \t%f \t%f \t%f \t%f \t%f \t%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 3; index = 1:20; name = 'Realsense';
fprintf('%s \t %d \t %f \t %f \t%f \t%f \t%f \t%f \t%f \t%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

%% Output for latex

fprintf('\n\n\n------------Output for latex ----------\n\n')
fprintf('&& reproj_mean & reproj_std & gyro_mean & gyro_std & accel_mean & accel_std & time_offset & time_offset_std \n');
camera = 1; index = 1:5; name = 'VersaVIS 0 ae';
fprintf('%s & %d & %f & %f &%f &%f &%f &%f &%f &%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 1; index = 6:8; name = 'VersaVIS 2 ae';
fprintf('%s & %d & %f & %f &%f &%f &%f &%f &%f &%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 1; index = 9:11; name = 'VersaVIS 4 ae';
fprintf('%s & %d & %f & %f &%f &%f &%f &%f &%f &%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 1; index = 12:14; name = 'VersaVIS 4 1ms';
fprintf('%s & %d & %f & %f &%f &%f &%f &%f &%f &%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 1; index = 15:17; name = 'VersaVIS 4 3ms';
fprintf('%s & %d & %f & %f &%f &%f &%f &%f &%f &%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 1; index = 18:20; name = 'VersaVIS 4 5ms';
fprintf('%s & %d & %f & %f &%f &%f &%f &%f &%f &%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 2; index = 1:20; name = 'VI-Sensor';
fprintf('%s & %d & %f & %f &%f &%f &%f &%f &%f &%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

camera = 3; index = 1:20; name = 'Realsense';
fprintf('%s & %d & %f & %f &%f &%f &%f &%f &%f &%f \n', name, length(reshape(data(camera,1:2,index,1),[],1)), ...
    mean(reshape(data(camera,1:2,index,1),[],1)), mean(reshape(data(camera,1:2,index,3),[],1)), ...
    mean(reshape(data(camera,1:2,index,7),[],1)), mean(reshape(data(camera,1:2,index,9),[],1)), ...
    mean(reshape(data(camera,1:2,index,10),[],1)), mean(reshape(data(camera,1:2,index,12),[],1)), ...
    mean(reshape(1000*data(camera,1:2,index,13),[],1)), std(reshape(1000*data(camera,1:2,index,13),[],1)) );

