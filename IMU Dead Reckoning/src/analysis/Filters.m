
bag = rosbag ('D:\Northeastern University\Sem 1\Robot Sensing and Navigation\LAB4\2022-10-27-19-39-11.bag');
% imu_data
bsel = select(bag,'Topic','/imu');
msgStructs = readMessages(bsel,'DataFormat','struct');
magneticField_x = cellfun(@(m) double(m.MagField.MagneticField_.X),msgStructs);
magneticField_y = cellfun(@(m) double(m.MagField.MagneticField_.Y),msgStructs);
magneticField_z = cellfun(@(m) double(m.MagField.MagneticField_.Z),msgStructs);
omega_x = cellfun(@(m) double(m.IMU.AngularVelocity.X),msgStructs);
omega_y = cellfun(@(m) double(m.IMU.AngularVelocity.Y),msgStructs);
omega_z = cellfun(@(m) double(m.IMU.AngularVelocity.Z),msgStructs);
orientation_x = cellfun(@(m) double(m.IMU.Orientation.X),msgStructs);
orientation_y = cellfun(@(m) double(m.IMU.Orientation.Y),msgStructs);
orientation_z = cellfun(@(m) double(m.IMU.Orientation.Z),msgStructs);
orientation_w = cellfun(@(m) double(m.IMU.Orientation.W),msgStructs);
acc_x = cellfun(@(m) double(m.IMU.LinearAcceleration.X),msgStructs);
acc_y = cellfun(@(m) double(m.IMU.LinearAcceleration.Y),msgStructs);
acc_z = cellfun(@(m) double(m.IMU.LinearAcceleration.Z),msgStructs);
imu_timePoints_sec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs);
imu_timePoints_nanosec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs);
imu_timePoints = double(imu_timePoints_sec + ( imu_timePoints_nanosec * 10^(-9)));
imu_time = imu_timePoints - imu_timePoints(1);

quat = [orientation_w orientation_x orientation_y orientation_z];
eulZYX_rad = quat2eul(quat);
yaw = eulZYX_rad (:,1);
pitch = eulZYX_rad (:,2);
roll = eulZYX_rad (:,3);


scale_matrix = [0.9982 -0.0132;-0.0132 0.9010];
offset_magx = -0.3393;
offset_magy = -0.0663;
offseted_magx = magneticField_x - offset_magx;
offseted_magy = magneticField_y - offset_magy;
calibrated_mag =  (scale_matrix*[offseted_magx,offseted_magy]')';

% yaw_from_magnetometer
mag_yaw= (atan2(-calibrated_mag(:,2),calibrated_mag(:,1)));
raw_mag_yaw = atan2(-magneticField_y,magneticField_x);
unwrapped_mag_yaw = unwrap(mag_yaw);
figure;
plot(imu_time,mag_yaw,"DisplayName","Corrected Magnetometer yaw");
hold on;
plot(imu_time,raw_mag_yaw,"DisplayName","Raw Magnetometer yaw");
xlabel('time in s')
ylabel('yaw in rad')
title('Yaw Angles from Magnetometer')
legend;

% yaw_from_gyroscope
gyro_yaw = cumtrapz(imu_time,omega_z)+ mag_yaw(1);
wrapped_gyro_yaw = wrapToPi(gyro_yaw);
figure;
plot(imu_time,mag_yaw,"DisplayName","Magnetometer yaw");
hold on;
plot(imu_time,wrapped_gyro_yaw,"DisplayName","Gyroscope yaw");
xlabel('time in s')
ylabel('yaw in rad')
title('Yaw from Gyroscope & Magnetometer plotted against time')
legend;

LPF_mag= lowpass(unwrapped_mag_yaw,1,40);
HPF_gyro = highpass(gyro_yaw,0.01,40);
%  
% figure;
% plot(imu_time,unwrapped_mag_yaw,"DisplayName","mag_yaw");
% hold on;
% plot(imu_time,(LPF_mag),"DisplayName","LPF_mag");
% hold on;
% legend;
% 
% plot(imu_time,(gyro_yaw),"DisplayName","gyro_yaw");
% hold on;
% plot(imu_time,(HPF_gyro),"DisplayName","HPF_gyro");
% hold on;
% legend;

% complementery filter_equation
a_c = 0.4;
filtered_yaw = a_c*LPF_mag + (1-a_c)*HPF_gyro;


figure;
plot(imu_time,(HPF_gyro),"DisplayName","HPF Gyro");
hold on;
plot(imu_time,(LPF_mag),"DisplayName","LPF Mag");
hold on;
plot(imu_time,(filtered_yaw),"DisplayName","Complimentary Filtered Yaw");
xlabel('time in s')
ylabel('yaw in rad')
title('Filtered Yaw YAW from Gyroscope and Magnetometer')
legend;

figure;
plot(imu_time,yaw,"DisplayName","IMU Yaw");
hold on;
plot(imu_time,(filtered_yaw),"DisplayName","Complimentary Filtered Yaw");
xlabel('time in s')
ylabel('yaw in rad')
title('IMU Yaw & Complimentary Filtered Yaw vs Time')
legend;