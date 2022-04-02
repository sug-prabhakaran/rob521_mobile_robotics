function [prob] = ProbFromLogOdds(logodds)
%PROBFROMLOGODDS - Calculate probability between [0,1] given a log-odd

prob = exp(logodds)./(1.+exp(logodds));
return