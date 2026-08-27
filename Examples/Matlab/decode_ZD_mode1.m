%% decode_ZD_mode1.m
% 用于对MT_IMU产品的mode1数据进行解析(39bytes)
% ============================================================================
%  Copyright (C) 2026 Wu Boyu. All rights reserved.
%  Version: 1.0.0 | Date: 2026-08-27
% ============================================================================

clear;
clc;

filename = 'txt文件路径';
fid = fopen(filename, 'r');

n = 3600000; % max number of samples
device=zeros(n,15);
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
    [packet,count] = fread(fid,37,'uint8');
    fseek(fid,-37,'cof');
    [ck1_out,ck2_out] = double_checksum_func(packet, 35);

    if packet(3)==32 && ck1_out==packet(36) && ck2_out==packet(37)
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




