%mean(dat)
%std(dat)


%size(dat,1)


amps = calc_amps(dat);

rad2deg(mean(amps))
rad2deg(std(amps))

function amps = calc_amps(spins)
    n = size(spins,1);
    amps = [];
    for i = 2:n
        val = abs(spins(i) - spins(i-1))/2;
        amps = [amps,val];
    end
end
