m = importdata('../output/KM04_HOP_07_muscledata.txt');

% length data is saved as:
%   <tendonname>_len
% moment arm data is saved as:
%   <tendonname>_mmt_<jointname>[_<axis>]
%   (for ball joints, axis = x, y or z)
%   (for free joints, axis = 1, 2 or 3 indicates translational x, y, z);

% use colheaders to find the column index for the data required
% example: 
%   SM.hip.x = find(contains(m.colheaders,'t_left_SM_mmt_j_hipL_x'));

% then, the values are:
%   m.data(:,SM.hip.x)

% Joint information
% (ID)	DoF	TYPE		NAME
% (0)	0	mjJNT_FREE	j_body
% (1)	6	mjJNT_HINGE	j_urostyleX
% (2)	7	mjJNT_HINGE	j_urostyleY
% (3)	8	mjJNT_HINGE	j_ISX
% (4)	9	mjJNT_HINGE	j_ISY
% (5)	10	mjJNT_BALL	j_hipL
% (6)	13	mjJNT_BALL	j_kneeL
% (7)	16	mjJNT_BALL	j_ankleL
% (8)	19	mjJNT_BALL	j_hipL_mir
% (9)	22	mjJNT_BALL	j_kneeL_mir
% (10)	25	mjJNT_BALL	j_ankleL_mir

% Tendon (id) and names:
% (0) t_left_LD
% (1) t_right_LD
% (2) t_left_IL_0
% (3) t_left_IL_1
% (4) t_left_IL_2
% (5) t_left_IL_3
% (6) t_right_IL_0
% (7) t_right_IL_1
% (8) t_right_IL_2
% (9) t_right_IL_3
% (10) t_left_CS_lat
% (11) t_left_CS_med
% (12) t_right_CS_lat
% (13) t_right_CS_med
% (14) t_left_CI_prox
% (15) t_left_CI_mid
% (16) t_left_CI_dist
% (17) t_right_CI_prox
% (18) t_right_CI_mid
% (19) t_right_CI_dist
% (20) t_left_II_lat
% (21) t_left_II_med
% (22) t_left_IE_prox
% (23) t_left_IE_dist
% (24) t_left_CR_dors
% (25) t_left_CR_vent
% (26) t_left_SM
% (27) t_left_ST_vent
% (28) t_left_ST_dors
% (29) t_left_GR
% (30) t_left_GL
% (31) t_left_AM_crv
% (32) t_left_AM_str
% (33) t_left_AL
% (34) t_left_SA
% (35) t_left_IFB
% (36) t_left_IFM
% (37) t_left_PY
% (38) t_left_OE
% (39) t_left_PE
% (40) t_left_PL


IE = find(contains(m.colheaders,'t_right_LD_mmt_j_ankleLy'));

CRdrslen = find(contains(m.colheaders,'t_left_CR_dors_len'),1);
CRdrsmmt = find(contains(m.colheaders,'t_left_CR_dors_mmt15'),1);
CRvntlen = find(contains(m.colheaders,'t_left_CR_vent_len'),1);
CRvntmmt = find(contains(m.colheaders,'t_left_CR_vent_mmt15'),1);

SM = find(contains(m.colheaders,'t_left_SM_len'));
ST = find(contains(m.colheaders,'t_left_ST_len'));
GR = find(contains(m.colheaders,'t_left_GR_len'));
GL = find(contains(m.colheaders,'t_left_GL_len'));
AM = find(contains(m.colheaders,'t_left_AM_str_len'));
AL = find(contains(m.colheaders,'t_left_AL_len'));
SA = find(contains(m.colheaders,'t_left_SA_len'));

x = 1:41;
% plot(x,tend.data(x,IE),...
%      x,tend.data(x,SM),...
%      x,tend.data(x,ST),...
%      x,tend.data(x,GR),...
%      x,tend.data(x,GL),...
%      x,tend.data(x,AM),...
%      x,tend.data(x,AL),...
%      x,tend.data(x,SA));
figure(3)
subplot(2,1,1); hold on;
plot(x,m.data(x,CRdrslen))%,...
     %x,tend.data(x,CRvntlen));
subplot(2,1,2); hold on;
plot(x,m.data(x,CRdrsmmt))%,...
     %x,tend.data(x,CRvntmmt));



%% Plot qfrc body force vector
 

Fmag = sqrt(sum(qfrc(:,1:3).^2,2));

figure(1); ax = gca;
axis equal; grid on; view(3);
lin = line([0 1],[0 1],[0 1],'LineWidth',3);
ax.XLim = [-1 1]; ax.YLim = [-1 1]; ax.ZLim = [-1 1];

for fr = 1:length(Fmag)
    lin.XData = [0 qfrc(fr,1)];
    lin.YData = [0 qfrc(fr,2)];
    lin.ZData = [0 qfrc(fr,3)];
    drawnow;
    pause(0.1);
end

 
 
