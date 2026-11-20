%% decode_ZD_mode6.m
% 用于对MT_IMU产品的mode6数据进行解析(81bytes)
% ============================================================================
%  Copyright (C) 2026 Wu Boyu. All rights reserved.
%  Version: 1.0.0 | Date: 2026-11-20
% ============================================================================

clear;
clc;

filename = 'txt文件路径';
fid = fopen(filename, 'r');
save_name = [filename(1:end-4),'.mat'];

n = 10000000; % max number of samples
device=zeros(n,29);
count = 0;
k = 1;

try
for i = 1:n
    h1 = 0;
    h2 = 0;
    while h2 ~= 83             % while 0x53 not receioved
        h1 = h2;    
        while h1 ~= 89             % while 0x59 not encountered
            [A, count] = fread(fid, 1, 'uint8');
            h1 = A(1);
        end
        [A, count] = fread(fid, 1, 'uint8');
        h2 = A(1);
    end
    [packet,count] = fread(fid,79,'uint8');
    fseek(fid,-79,'cof');
    [ck1_out,ck2_out] = double_checksum_func(packet, 77);

    if packet(3)==74 && ck1_out==packet(78) && ck2_out==packet(79)
        % count
        device(k,1) = fread(fid,1,'uint16');
        % main length
        device(k,2) = fread(fid,1,'uint8');

        % temp
        % temp-id
        device(k,3) = fread(fid,1,'uint8');
        % temp-length
        device(k,4) = fread(fid,1,'uint8');
        % temp-data(℃)
        device(k,5)=fread(fid,1,'int16')*0.01;     

        % acc
        % acc-id
        device(k,6) = fread(fid,1,'uint8');
        % acc-length
        device(k,7) = fread(fid,1,'uint8');
        % acc-data(m/s2)
        device(k,8:10)=fread(fid,3,'int32')*0.000001;

        % gyro
        % gyro-id
        device(k,11) = fread(fid,1,'uint8');
        % gyro-length
        device(k,12) = fread(fid,1,'uint8');
        % gyro-data(deg/s)
        device(k,13:15)=fread(fid,3,'int32')*0.000001;   

        % euler
        % euler-id
        device(k,16) = fread(fid,1,'uint8');
        % euler-length
        device(k,17) = fread(fid,1,'uint8');
        % euler-data(°) pitch roll yaw
        device(k,18:20)=fread(fid,3,'int32')*0.000001;   

        % quaternion
        % quaternion-id
        device(k,21) = fread(fid,1,'uint8');
        % quaternion-length
        device(k,22) = fread(fid,1,'uint8');
        % quaternion-data
        device(k,23:26)=fread(fid,4,'int32')*0.000001;   

        % time stamp
        % time-id
        device(k,27) = fread(fid,1,'uint8');
        % time-length
        device(k,28) = fread(fid,1,'uint8');
        % time-data
        device(k,29)=fread(fid,1,'uint64')*0.000001;  % us->s  

        k = k + 1;  % current sample number
    else
        continue;
    end
end
catch
end
k = k - 1;
data_out=device(1:k,:);
fclose(fid);



