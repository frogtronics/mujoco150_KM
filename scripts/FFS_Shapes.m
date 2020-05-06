nV = 9;
%off-centred circle
P = [cos(linspace(0,2*pi,nV+1))+1; 1.5*sin(linspace(0,2*pi,nV+1))]*10/nV;
V = vectorspace(P(:,1:end-1));
z = zeros(1,length(P));

figure(1)
quiver(z,z,P(1,:),P(2,:)); 
hold on; plot(V(1,:),V(2,:)); hold off; axis equal;

xlim([-1 11]); ylim([-10 10]);

%%
nV = 9;
%misleading off-centred directional ellipse
P = [cos(linspace(0,2*pi,nV+1))+1; 2.5*sin(linspace(0,2*pi,nV+1))]*10/nV;
V = vectorspace(P(:,1:end-1));
z = zeros(1,length(P));

figure(2)
quiver(z,z,P(1,:),P(2,:)); 
hold on; plot(V(1,:),V(2,:)); hold off; axis equal;

xlim([-1 11]); ylim([-10 10]);



%% circular comparisons

nV = 9;
%centred circle
P = [1.56*cos(linspace(0,2*pi,nV+1)); 1.56*sin(linspace(0,2*pi,nV+1))]*10/nV;
V = vectorspace(P(:,1:end-1));
z = zeros(1,length(P));

figure(3);
subplot(1,2,1);
quiver(z,z,P(1,:),P(2,:),'Color',thesisThemeColor(1)); 
hold on; plot(V(1,:),V(2,:),'Color',thesisThemeColor(2)); hold off; axis equal;
t(1) = title('Centred cicular FFS');
xlim([-6 6]); ylim([-6 6]);

%off-centre it
P = [cos(linspace(0,2*pi,nV+1))+1; 1.5*sin(linspace(0,2*pi,nV+1))]*10/nV;
V = vectorspace(P(:,1:end-1));

subplot(1,2,2);
quiver(z,z,P(1,:)+10/nV,P(2,:),'Color',thesisThemeColor(1));
hold on; plot(V(1,:),V(2,:),'Color',thesisThemeColor(2)); hold off; axis equal;
t(2) = title('Off-centre cicular FFS');
xlim([-1 11]); ylim([-6 6]);


formatFigure(figure(3),'a7','Thesis',1.25)
%t(1).Position = t(1).Position - [0.5 0 0];
%   t(2).Position = t(2).Position + [0.5 0 0];
exportThesisFigure(figure(3),4,'FFS_circle_centredness');

%% elliptical comparisons

nV = 9;
%centred directional ellipse
P = [1.56*cos(linspace(0,2*pi,nV+1)); 2.5*sin(linspace(0,2*pi,nV+1))]*10/nV;
V = vectorspace(P(:,1:end-1));
z = zeros(1,length(P));

figure(4);
subplot(1,2,1);
quiver(z,z,P(1,:),P(2,:),'Color',thesisThemeColor(1)); 
hold on; plot(V(1,:),V(2,:),'Color',thesisThemeColor(2)); hold off; axis equal;

t(1) = title('Centred directional FFS'); 
xlim([-6 6]); ylim([-9 9]);

%off-centre it
P = [cos(linspace(0,2*pi,nV+1))+1; 2.5*sin(linspace(0,2*pi,nV+1))]*10/nV;
V = vectorspace(P(:,1:end-1));

subplot(1,2,2);
quiver(z,z,P(1,:)+10/nV,P(2,:),'Color',thesisThemeColor(1));
hold on; plot(V(1,:),V(2,:),'Color',thesisThemeColor(2)); hold off; axis equal;

t(2) = title('Off-centre directional FFS'); 
xlim([-1 11]); ylim([-9 9]);



formatFigure(figure(4),'a7','Thesis',1.25)
t(1).Position = t(1).Position - [0.5 0 0];
t(2).Position = t(2).Position + [0.5 0 0];
exportThesisFigure(figure(4),4,'FFS_ellipse_centredness');
