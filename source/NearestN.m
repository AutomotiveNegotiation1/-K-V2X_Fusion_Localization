function Pos1 = NearestN(Pos2)
lent_p = 0;
lent_i = 1;
outl = zeros(length(Pos2),length(Pos2));
indsort = zeros(length(Pos2),length(Pos2));

for dfg = 1 : length(Pos2)
    [distTwo] = abs(Pos2-Pos2(dfg));
    [outl(dfg,:), indsort(dfg,:)] = sort(distTwo);
    lent = length(find(outl(dfg,:)<1));
    if lent>lent_p
        lent_p = lent;
        lent_i = indsort(dfg,:);
    end
    % MeanLoutl(dfg) = mean(distTwo(outl).^2);
end

Pos1 = mean(Pos2(lent_i(1:lent_p)));
