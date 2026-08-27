function [ck1_out,ck2_out] = double_checksum_func(data_in, check_length)
    ck1 = 0;
    ck2 = 0;
    for i=1:check_length
        ck1 = ck1 + data_in(i);
        ck2 = ck2 + ck1;
    end
    ck1_out = bitand(ck1,255);
    ck2_out = bitand(ck2,255);
end

