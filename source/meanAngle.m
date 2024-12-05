function HeadingH = meanAngle(RefHeading, Heading)
L = length(Heading);

for kk = 1 : L
    Heading(kk) = AngleMatching(RefHeading,Heading(kk));
end
HeadingH = mean(Heading);

