%mean(dat)
%std(dat)


%size(dat,1)
%load('amp25.mat');

%plot(1:73,dat);

amps = calc_amps(dat);

amps_deg = rad2deg(amps);

% amps_filtered = filter_outlier(amps_deg,10);

rad2deg(mean(amps))
rad2deg(std(amps))

function amps = calc_amps(spins)
    n = size(spins,1);
    amps = [];
    for i = 2:n
        val = abs(spins(i) - spins(i-1))/2;
        amps = [amps,val];
    end
    amps = amps';
end


function amps = filter_outlier(src,m)
    n = size(src,1);
    amps = [];
    for i=1:n
        v = src(i);
        if v > m
            amps = [amps,v];
        end
    end
end