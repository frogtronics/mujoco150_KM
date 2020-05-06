femLen = 0.018143;
tibLen = 0.019438;
totalLen = femLen + tibLen;
%#ok<*NOPTS>

%%
femScale = 1.2;
tibScale = (totalLen - femLen * femScale)/tibLen %#ok<NASGU>

%%
tibScale = 0.8:0.1:1.2;
femScale = (totalLen - tibLen * tibScale)/femLen