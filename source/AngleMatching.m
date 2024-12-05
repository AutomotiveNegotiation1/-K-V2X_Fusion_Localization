function TargetAng = AngleMatching(RefAng, TargetAng)

DiffTemp = RefAng-TargetAng;
DiffInt = sign(DiffTemp)*floor(abs(DiffTemp)/(2*pi));
TargetAng = TargetAng+DiffInt*2*pi;
if TargetAng - RefAng > pi
    TargetAng = TargetAng -2*pi;
elseif TargetAng - RefAng < -pi
    TargetAng = TargetAng +2*pi;
end
