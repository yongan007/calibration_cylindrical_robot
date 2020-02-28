function m = configuration(n)
    m = [2*mod(floor(n/4),2)-1,2*mod(floor(n/2),2)-1,2*mod(n,2)-1];
end