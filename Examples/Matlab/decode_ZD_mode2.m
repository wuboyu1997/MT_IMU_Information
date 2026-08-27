%% decode_ZD_mode2.m
% 用于对MT_IMU产品的mode2数据进行解析(35bytes)
% ============================================================================
%  Copyright (C) 2026 Wu Boyu. All rights reserved.
%  Version: 1.0.0 | Date: 2026-08-27
% ============================================================================

clear;
clc;

filename = 'txt文件路径';
fid = fopen(filename, 'r');

n = 3600000; % max number of samples
device=zeros(n,12);
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
    [packet,count] = fread(fid,33,'uint8');
    fseek(fid,-33,'cof');
    [ck1_out,ck2_out] = double_checksum_func(packet, 31);

    if packet(3)==28 && ck1_out==packet(32) && ck2_out==packet(33)
        % count
        device(k,1) = fread(fid,1,'uint16');
        % main length
        device(k,2) = fread(fid,1,'uint8');

        % acc
        % acc-id
        device(k,3) = fread(fid,1,'uint8');
        % acc-length
        device(k,4) = fread(fid,1,'uint8');
        % acc-data(m/s2)
        device(k,5:7)=fread(fid,3,'int32')*0.000001;

        % gyro
        % gyro-id
        device(k,8) = fread(fid,1,'uint8');
        % gyro-length
        device(k,9) = fread(fid,1,'uint8');
        % gyro-data(deg/s)
        device(k,10:12)=fread(fid,3,'int32')*0.000001;   

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




