clc
clear
close all

CPU_freq = 16e6; % of Arduino UNO %% 48e6 for Arudino Zero
Cam_rate_vec = [];
IMU_rate_vec = [];
Cam_compare_vec = [];
IMU_compare_vec = [];
Cam_prescale_vec = [];
IMU_prescale_vec = [];

for Cam_rate=1:300
    for IMU_rate=1:500
        for Cam_prescale = [1 8 64 256 1024] % timer1 of Arduino UNO
            for IMU_prescale = [1 8 32 64 128 256 1024] % timer2 of Arduino UNO
                IMU_compare = 1.0/IMU_rate*CPU_freq/IMU_prescale - 1;
                Cam_compare = 1.0/Cam_rate*CPU_freq/Cam_prescale - 1;
                if mod(IMU_compare,1)==0 && mod(Cam_compare,1) == 0 && IMU_compare < 2^8 && Cam_compare < 2^16 %&& (mod(IMU_rate / Cam_rate,1) == 0 || mod(Cam_rate / IMU_rate,1) == 0)
                    Cam_rate_vec(end+1) = Cam_rate;
                    IMU_rate_vec(end+1) = IMU_rate;
                    Cam_compare_vec(end+1) = Cam_compare;
                    IMU_compare_vec(end+1) = IMU_compare;
                    Cam_prescale_vec(end+1) = Cam_prescale;
                    IMU_prescale_vec(end+1) = IMU_prescale;
                end
            end
        end
    end
end

T = table(Cam_rate_vec',IMU_rate_vec',Cam_compare_vec',IMU_compare_vec',Cam_prescale_vec',IMU_prescale_vec','VariableNames',{'Cam_rate','IMU_rate','Cam_compare','IMU_compare','Cam_prescale','IMU_prescale'})
