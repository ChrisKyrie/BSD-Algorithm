  coder.extrinsic('FCT_LCDAS');

em_PerfDegr_data = zeros(1,60);
em_Road = zeros(1,60);
em_MovingObjectTraces = zeros(1,60);
em_GenObjectList = zeros(1,60);
em_ARSObjectList = zeros(1,60);
vdy_VehPar = zeros(1,60);
vdy_VehDyn = zeros(1,60);

longitudinalPos = zeros(1,100);
lateralPos = zeros(1,100);
count = zeros(1,1);
LeftWarningResult = zeros(1,1);

vdy_VehPar(1,1) = 1;
vdy_VehPar(1,2) = 3.5;
vdy_VehPar(1,3) = 2.8;
vdy_VehPar(1,4) = 0;

vdy_VehDyn(1,1) = 10; %VX本车向前为正，向后为负
vdy_VehDyn(1,2) = 0.5;%VY本车向左为正，向右为负

em_GenObjectList(1,1) = 1;
em_GenObjectList(1,2) = 5;%目标车X
em_GenObjectList(1,3) = (-1.0)*vdy_VehPar(1,3)/2-1.5;
em_GenObjectList(1,4) = 10;
em_GenObjectList(1,5) = 0.9;
em_GenObjectList(1,6) = 5;
em_GenObjectList(1,7) = (-1.0)*vdy_VehPar(1,3)/2-1.5;
em_GenObjectList(1,8) = 10;

em_ARSObjectList(1,1) = 0;
em_ARSObjectList(1,2) = 0;

longitudinalPos(1,1) = em_GenObjectList(1,6);
lateralPos(1,1) = em_GenObjectList(1,7);
count = 1;
LeftWarningResult = FCT_LCDAS(em_PerfDegr_data,em_Road,em_MovingObjectTraces,em_GenObjectList,em_ARSObjectList,vdy_VehPar,vdy_VehDyn);
