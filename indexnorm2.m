function [Ji_mod] = indexnorm2(Ji,p,tr)
[J,i]=sort(Ji);
if p<tr
    logscale=[zeros(1,tr-p),logspace(-3,1,p)];
else
    logscale=logspace(-3,1,p);
end
Ji_mod=J.*logscale';
Ji_mod=[i,Ji_mod];
Ji_mod=sortrows(Ji_mod,1);
Ji_mod=Ji_mod(:,2);
end