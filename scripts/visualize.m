%plot orientations


%% before
femur.X = quatMultVec(qFemur,[1 0 0]).*0.5;
femur.Y = quatMultVec(qFemur,[0 1 0]).*0.5;
femur.Z = quatMultVec(qFemur,[0 0 1]).*1;

tibFib.X = femur.Z + quatMultVec(qTibFib,[1 0 0]).*0.5;
tibFib.Y = femur.Z + quatMultVec(qTibFib,[0 1 0]).*0.5;
tibFib.Z = femur.Z + quatMultVec(qTibFib,[0 0 1]).*1;

foot.X = tibFib.Z + quatMultVec(qFoot,[1 0 0]).*0.5;
foot.Y = tibFib.Z + quatMultVec(qFoot,[0 1 0]).*0.5;
foot.Z = tibFib.Z + quatMultVec(qFoot,[0 0 1]).*1;



figure(1); clf(1); view(-150,-87); axis equal; grid on;

line([0 femur.X(1)],[0 femur.X(2)],[0 femur.X(3)],'Color','r');
line([0 femur.Y(1)],[0 femur.Y(2)],[0 femur.Y(3)],'Color','g');
line([0 femur.Z(1)],[0 femur.Z(2)],[0 femur.Z(3)],'Color','b','LineWidth',2);

line([femur.Z(1) tibFib.X(1)],[femur.Z(2) tibFib.X(2)],[femur.Z(3) tibFib.X(3)],'Color','r');
line([femur.Z(1) tibFib.Y(1)],[femur.Z(2) tibFib.Y(2)],[femur.Z(3) tibFib.Y(3)],'Color','g');
line([femur.Z(1) tibFib.Z(1)],[femur.Z(2) tibFib.Z(2)],[femur.Z(3) tibFib.Z(3)],'Color','b','LineWidth',2);

line([tibFib.Z(1) foot.X(1)],[tibFib.Z(2) foot.X(2)],[tibFib.Z(3) foot.X(3)],'Color','r');
line([tibFib.Z(1) foot.Y(1)],[tibFib.Z(2) foot.Y(2)],[tibFib.Z(3) foot.Y(3)],'Color','g');
line([tibFib.Z(1) foot.Z(1)],[tibFib.Z(2) foot.Z(2)],[tibFib.Z(3) foot.Z(3)],'Color','b','LineWidth',2);



%% after

line([femur.Z(1) femur.Z(1)+kneeAx(1)],[femur.Z(2) femur.Z(2)+kneeAx(2)],[femur.Z(3) femur.Z(3)+kneeAx(3)]);
line([tibFib.Z(1) tibFib.Z(1)+tmtAx(1)],[tibFib.Z(2) tibFib.Z(2)+tmtAx(2)],[tibFib.Z(3) tibFib.Z(3)+tmtAx(3)]);



femur.X = quatMultVec(qFemur,[1 0 0]).*0.5;
femur.Y = quatMultVec(qFemur,[0 1 0]).*0.5;
femur.Z = quatMultVec(qFemur,[0 0 1]).*1;

tibFib.X = femur.Z + quatMultVec(qTibFib,[1 0 0]).*0.5;
tibFib.Y = femur.Z + quatMultVec(qTibFib,[0 1 0]).*0.5;
tibFib.Z = femur.Z + quatMultVec(qTibFib,[0 0 1]).*1;

foot.X = tibFib.Z + quatMultVec(qFoot,[1 0 0]).*0.5;
foot.Y = tibFib.Z + quatMultVec(qFoot,[0 1 0]).*0.5;
foot.Z = tibFib.Z + quatMultVec(qFoot,[0 0 1]).*1;



figure(2); clf(2); view(-150,-87); axis equal; grid on;

line([0 femur.X(1)],[0 femur.X(2)],[0 femur.X(3)],'Color','r');
line([0 femur.Y(1)],[0 femur.Y(2)],[0 femur.Y(3)],'Color','g');
line([0 femur.Z(1)],[0 femur.Z(2)],[0 femur.Z(3)],'Color','b','LineWidth',2);

line([femur.Z(1) tibFib.X(1)],[femur.Z(2) tibFib.X(2)],[femur.Z(3) tibFib.X(3)],'Color','r');
line([femur.Z(1) tibFib.Y(1)],[femur.Z(2) tibFib.Y(2)],[femur.Z(3) tibFib.Y(3)],'Color','g');
line([femur.Z(1) tibFib.Z(1)],[femur.Z(2) tibFib.Z(2)],[femur.Z(3) tibFib.Z(3)],'Color','b','LineWidth',2);

line([tibFib.Z(1) foot.X(1)],[tibFib.Z(2) foot.X(2)],[tibFib.Z(3) foot.X(3)],'Color','r');
line([tibFib.Z(1) foot.Y(1)],[tibFib.Z(2) foot.Y(2)],[tibFib.Z(3) foot.Y(3)],'Color','g');
line([tibFib.Z(1) foot.Z(1)],[tibFib.Z(2) foot.Z(2)],[tibFib.Z(3) foot.Z(3)],'Color','b','LineWidth',2);


line([femur.Z(1) femur.Z(1)+kneeAx(1)],[femur.Z(2) femur.Z(2)+kneeAx(2)],[femur.Z(3) femur.Z(3)+kneeAx(3)]);
line([tibFib.Z(1) tibFib.Z(1)+tmtAx(1)],[tibFib.Z(2) tibFib.Z(2)+tmtAx(2)],[tibFib.Z(3) tibFib.Z(3)+tmtAx(3)]);


