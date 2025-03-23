
function [lb,ub,dim,fobj] = Get_F_details(F)

    fobj = @F1;
    dim=30;
    ub=100;
    lb=-100;
end
% F1 (sphere)
function o = F1(x)
o=sum(x.^2);
end



