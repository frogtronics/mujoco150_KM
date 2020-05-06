qpos.t80 = importdata('../output/qpos80.txt');
qpos.t90 = importdata('../output/qpos90.txt');
qpos.t100 = importdata('../output/qpos100.txt');
qpos.t110 = importdata('../output/qpos110.txt');
qpos.t120 = importdata('../output/qpos120.txt');
qpos.time = importdata('../output/qposTime.txt');


clearvars -except qpos*
%%
%figure(1)
%scatter3(data(:,2),data(:,3),data(:,4))

[X,Y,Z] = sphere(100);

figure(2), grid on, colormap gray; hold off;
srf = surf(X,Y,Z,zeros(size(Z))); hold on;
srf.EdgeAlpha = 0;
srf.FaceAlpha = 0.2; axis equal

start = 10000;
cut = 13000;

trials = {'t80','t90','t100','t110','t120'};

cols = generateColor(length(trials) + 4);

for trial = trials

    ind = find(strcmp(trial,trials)) + 3;
    
    data = qpos.(trial{1})(start:cut,:);
    len = length(data);

    [vhip,vknee,vank] = deal(zeros(len,3));
    for ii = 1:len
        vhip(ii,:) = quatMultVec(data(ii,1:4),[0 0 1]);
        vknee(ii,:) = quatMultVec(data(ii,5:8),[0 0 1]);
        vank(ii,:) = quatMultVec(data(ii,9:12),[0 0 1]);
    end

    figure(1) %'Color',cols(ind,:),
    subplot(3,1,1), hold on
    plot(qpos.time(1:len,13)./10,pi-acos(dot(repmat([0 0 1],[len 1]),vhip,2)),...
        'LineWidth',2);
    xlim([0 0.15]);
    subplot(3,1,2), hold on
    plot(qpos.time(1:len,13)./10,pi-acos(dot(vhip,vknee,2)),...
        'LineWidth',2)
    xlim([0 0.15])
    subplot(3,1,3), hold on
    plot(qpos.time(1:len,13)./10,pi-acos(dot(vknee,vank,2)),...
        'LineWidth',2)
    xlim([0 0.15])

    figure(2)
%     scatter3(vhip(:,1),vhip(:,2),vhip(:,3),3,(1:len)./len);
%     scatter3(vknee(:,1),vknee(:,2),vknee(:,3),3,(1:len)./len);
%     scatter3(vank(:,1),vank(:,2),vank(:,3),3,(1:len)./len);

    scatter3(vhip(:,1),vhip(:,2),vhip(:,3),3,repmat(cols(ind,:),[len 1]));
    scatter3(vknee(:,1),vknee(:,2),vknee(:,3),3,repmat(cols(ind,:),[len 1]));
    scatter3(vank(:,1),vank(:,2),vank(:,3),3,cols(ind,:));
    
    axis equal%, hold off;
    axis([-1 0 -1 0 -0.2 1])

end


%% format figure 1
figure(1)
subplot(3,1,1)
ax = gca;

ax.FontSize = 12;
ax.FontWeight = 'bold';
ax.LabelFontSizeMultiplier = 1.1667;

ylabel('Hip')
legend('80%','90%','100%','110%','120% tibia length',...
    'Orientation','horizontal','Location','best')



subplot(3,1,2)
ax = gca;

ax.FontSize = 12;
ax.FontWeight = 'bold';
ax.LabelFontSizeMultiplier = 1.1667;


ylabel(sprintf('<- Flexion           Joint angles (rad)         Extension ->\nKnee'))

subplot(3,1,3)
ax = gca;

ax.FontSize = 12;
ax.FontWeight = 'bold';
ax.LabelFontSizeMultiplier = 1.1667;

ylabel('Ankle')
ylim([0.9 3.4])
xlabel('Time (s)')

%%
hipRange = importdata('../output/qpos_hiprange_1.txt');


ind = 1;
    
data = hipRange;
len = length(data);

vhip = zeros(len,3);
for ii = 1:len
    vhip(ii,:) = quatMultVec(data(ii,1:4),[0 0 1]);
end

%transform by -90 degrees around X

vhip = vhip(vhip(:,1)<-0.56 & (vhip(:,3) < 0.8),:);

vhip = vhip*rotMatX(-pi/2);

[X,Y,Z] = sphere(100);


fig = figure(3); grid off, colormap gray; hold off;
srf = surf(X*0.95,Y*0.95,Z*0.95,zeros(size(Z))); hold on;
srf.EdgeAlpha = 0;
srf.FaceAlpha = 0.2; axis equal

sct = scatter3(vhip(:,1),vhip(:,2),vhip(:,3),15,'blue');
sct.MarkerFaceColor = 'flat';

axis equal%, hold off;
%axis([-1 0 -1 0 -0.2 1])


%now comes some math

%find the mean of all points
% meanPoint = mean(vhip);
% scatter3(meanPoint(1), meanPoint(2), meanPoint(3),'MarkerFaceColor','flat')

P = vhip;
k = boundary(P,0);
trs = trisurf(k,P(:,1),P(:,2),P(:,3),'Facecolor','blue','FaceAlpha',0.05,...
                'EdgeAlpha',0);
            
ax = gca;

ax.FontSize = 12;
ax.FontWeight = 'bold';
ax.LabelFontSizeMultiplier = 1.1667*1.5;

xlabel('Lateral')
ylabel('Proximodistal')
zlabel('Vertical')



