function Fullc = checkFull_1(DistMap, Selmode)

[M,N,ll] = size(DistMap);



if Selmode == 1
    Fullc = 1;
    for kk = 1 : M
        if length(find(DistMap(kk,:,end-1)~=0)) == 0
            Fullc = Fullc * 0;
        else
            Fullc = Fullc * 1;
        end
    end
else
    Fullc = 0;
    for kk = 1 : M
        if length(find(DistMap(kk,:,end)~=0)) == 0
            Fullc = Fullc;
        else
            Fullc = Fullc + 1;
        end
    end
end
