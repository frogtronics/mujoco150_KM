%Find principal components of FFS

load('KM04_HOP_07_EE_FFS.mat');

%make mesh manifold by removing duplicate points


P = convexHull(space(1).DT);
TR = triangulation(P,space(1).DT.Points);

RBP=RigidBodyParams(TR); 
VisualizeLocalFrame(TR,false);


%%
sph = spheretri(100);

P = convexHull(delaunayTriangulation(sph));

TR = triangulation(P,sph);
VisualizeLocalFrame(TR,false);

