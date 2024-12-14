clear
clc
trials = 10;
p = [repelem([3:3:18],trials) 21*ones(1,30)];
p = p(randperm(length(p)));
z = zeros(1,length(p));
vec = [z; p];
vec = (-vec(:)/0.1 + 450)
allOneString = sprintf('%.0f,' , vec)