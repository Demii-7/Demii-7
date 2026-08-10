%The following function finds fear index usings 10 features stored in a vector, passed as 
%an argument and returns a scalar of the featr index.

function FearIndex = calculateFearIndex(F)
    %Calculate Fear 
    FearIndex = F(1) + (2*F(2)) + F(3) + (0.5*F(4)) + F(5) + (2*F(6)) + F(7) + (5*F(8)) + (0.001*F(9)) + (0.5*F(10)); 
end