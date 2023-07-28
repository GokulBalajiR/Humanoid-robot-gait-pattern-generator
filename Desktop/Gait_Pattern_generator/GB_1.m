animateOn = false; 

xTorso = 0.12; % x distance from body to foot 
g = 9.807; 
Ts = 0.005; %sample time (s)


zRobot = 0.78;
zModel = 0.68;  


swingHeight = 0.1; 

useParametricICs = true; 

if useParametricICs
    stepLength = 0.2; 
    dy_mid = 0.06;  % y velocity when y = 0 this is the minimum y velocity
    x0 = 0.12;      % want x0 to be around xTorso 
    [dx0, y0, dy0, tSingleSupport] = findInitialConditions(stepLength, dy_mid, x0, zModel, g, Ts);
    display('the initial coordinates are\n')
    display([x0,y0])
else
    tSingleSupport = 1.335;
    x0 = 0.12; 
    dx0 = -0.4501;
    y0 = -0.1; 
    dy0 = 0.3845;
end 

A = [0   1 0    0
    g/zModel 0 0    0
    0    0 0    1
    0    0 g/zModel 0]; 
B = [0 0
     1 0
     0 0
     0 1]; 
C = [1 0 0 0
     0 0 1 0]; 
D = [0 0;0 0]; 

lipm = ss(A,B,C,D); 


lipmD = c2d(lipm, Ts); 

Ad = lipmD.A; 
Bd = lipmD.B;
Cd = lipmD.C;
Dd = lipmD.D;
display(Ad)

state0 = [x0; dx0; y0; dy0]; 

u0 = [0; 0];


modelVars.g = g; 
modelVars.Ts = Ts; 
modelVars.zModel = zModel;
modelVars.Ad = Ad; 
modelVars.Bd = Bd; 
modelVars.Cd = Cd; 
modelVars.Dd = Dd; 
modelVars.tSingleSupport = tSingleSupport; 

simStates = struct('bodyPos',[],'footPos',[],'timeVec',[]);


robotpos0 = [0; 0; zRobot]; 

% Store left and right foothold position (left:odd, right:even)
simStates.footPos(:,end+1) = [-xTorso; 0; 0]; % left foot
simStates.footPos(:,end+1) = [ xTorso; 0; 0]; % right foot

% Set the total number of steps to take
numSteps = 6;

hFig = figure; 
hFig.Visible = 'on'; % for livescript 
hFig.Units = 'Normalized'; 
hFig.OuterPosition = [0 0 1 1]; 
hAx  = axes(hFig); 

% create handles for plots 
hold(hAx, 'on'); 
hLeg  = plot3(hAx,[0,robotpos0(1)],[0,robotpos0(2)],[0,robotpos0(3)],...
                '-o','MarkerIndices',2,'LineWidth',2,'MarkerSize',8); % represents com + leg 
hCom = plot3(hAx,robotpos0(1),robotpos0(2),robotpos0(3)); % represents com traj
hLineFootHold = plot3(hAx, simStates.footPos(1,:), simStates.footPos(2,:), simStates.footPos(3,:)); % represents foothold traj
hLineLeftFoot = plot3(hAx, simStates.footPos(1,1), simStates.footPos(2,1), simStates.footPos(3,1));
hLineRightFoot = plot3(hAx, simStates.footPos(1,2), simStates.footPos(2,2), simStates.footPos(3,2));
hold(hAx, 'off'); 

grid(hAx, 'on'); 
view(hAx,3)
axis equal 
hAx.XLim = [-3*zRobot 3*zRobot];
hAx.YLim = [-3*zRobot 3*zRobot]; 
legend(hAx, 'Virtual Leg and COM of LIP','Trajectory of COM', 'Foot Hold', 'Left Foot', 'Right Foot', ...
       'Location', 'northeast');



stepinfos = cell(1,numSteps+2);



walkRdyPoints = [0      0.67*x0; 
                 0      0      ; 
                 zRobot zModel];  
% Note: We are moving COM to be partly over the right foot

timepoints = [0 1]; % Hard-coded 1 second to move down
timevec = timepoints(1):Ts:timepoints(end)-Ts; 
[q, qd] = cubicpolytraj(walkRdyPoints, timepoints, timevec); 
simStates.bodyPos = [simStates.bodyPos q]; 
simStates.timeVec = [simStates.timeVec timevec]; 

% save data
fhold_x = simStates.footPos(1,1); 
fhold_y = simStates.footPos(2,1); 
stepinfo.index = 1; 
bodystate = [q(1,:); qd(1,:); q(2,:); qd(2,:); q(3,:); qd(3,:)]; % pos wrt world  
relstate = bodystate - [fhold_x; 0; fhold_y; 0; 0; 0]; % save pos wrt foot  
stepinfo.state = relstate;
stepinfo.timevec = timevec; 
stepinfo.mode = 'doublesupport'; 
stepinfo.footplant = [fhold_x; fhold_y; 0]; % state is in absolute position 
stepinfos{1} = stepinfo; 

% update graphics
for idx = 1:size(simStates.bodyPos,2)
    points = [zeros(3,1) simStates.bodyPos(:,idx)]; 
    updateLine(hLeg, points); 
    appendLine(hCom, simStates.bodyPos(:,idx)); 
    if animateOn
        pause(0.005);
    end
end

walkRdyPoints(:,end+1) = [0     ; 
                          0.1   ; 
                          zModel];  

yoff = walkRdyPoints(2,3); 
% initial walking footPosition
fhold_x = -x0; 
fhold_y = -y0 + yoff; 
simStates.footPos(:,end+1) = [fhold_x; fhold_y; 0]; % left

% make trajectory for the stance foot to walk-initial state
rdyPos1 = walkRdyPoints(:,2); 
rdypos2 = walkRdyPoints(:,3); 
rdyvel1 = [0; 0; 0]; 
rdyvel2 = [dx0; dy0; 0]; 
waypoints = [rdyPos1 rdypos2];
velpoints = [rdyvel1 rdyvel2]; 

timepoints = [1 1.5]; % Hard-coded 0.5 seconds to take a half step
timevec = timepoints(1):Ts:timepoints(end)-Ts; 
[q, qd, qdd] = cubicpolytraj(waypoints, timepoints, timevec, ...
                  'VelocityBoundaryCondition', velpoints); 
simStates.bodyPos = [simStates.bodyPos q]; 
simStates.timeVec = [simStates.timeVec timevec]; 

% make trajectory for the swing foot
swingfootpos0 = simStates.footPos(:,1); 
swingfootpos1 = simStates.footPos(:,3); 
[qswing, dqswing, ddqswing] = getSwingFootTraj(swingfootpos0, swingfootpos1, swingHeight, ...
                                timepoints(1), timepoints(end),Ts); 

% save data
stepinfo.index = 2; 
bodystate = [q(1,:); qd(1,:); q(2,:); qd(2,:); q(3,:); qd(3,:)]; % com info 
relstate = bodystate - [simStates.footPos(1,2); 0; simStates.footPos(2,2); 0; 0; 0]; % relative position  
stepinfo.state = relstate;
stepinfo.timevec = timevec; 
stepinfo.mode = 'singlesupportright'; 
stepinfo.swing = qswing; 
stepinfo.footplant = [simStates.footPos(1,2); simStates.footPos(2,2); 0]; 
stepinfos{2} = stepinfo; 

% update graphics
for idx = 1:size(q,2)
    points = [zeros(3,1) q(:,idx)]; 
    updateLine(hLeg, points); 
    appendLine(hCom, q(:,idx)); 
    % add points to swing foot 
    appendLine(hLineLeftFoot, qswing(:,idx)); 
    if animateOn
        pause(0.005);
    end

end
appendLine(hLineFootHold, [fhold_x; fhold_y; 0])


fhold_x1 = fhold_x; 
fhold_y1 = fhold_y; 
 
for stp = 1:numSteps  
    % Update foot hold postiions
    fhold_x0 = fhold_x1;
    fhold_y0 = fhold_y1;
    
    % Simulate stance LIPM for single support time duration
    [states, timevec, simStates] = stanceSim(fhold_x0, fhold_y0, state0, u0, modelVars, simStates);
    
    % Save Step Info at each step 
    index = stp + 2; 
    stepinfo.index = index; 
    savestate = [states; repmat([zModel; 0],[1 size(states,2)])]; % Add Z and dZ states
    stepinfo.state = savestate; 
    stepinfo.footplant = [fhold_x0; fhold_y0; 0]; 
    stepinfo.timevec = timevec; 
    if rem(stp,2) > 0
        stepinfo.mode = 'singlesupportleft';
    else
        stepinfo.mode = 'singlesupportright';
    end     
    
    % change leg according to control law 
    [state0, fhold_x1, fhold_y1] = changeLeg(states(:,end), simStates);
    simStates.footPos(:,end+1) = [fhold_x1; fhold_y1; 0];
    
    % define swing foot trajectory with new leg position 
    [q, dq, ddq] = getSwingFootTraj(simStates.footPos(:,end-2), simStates.footPos(:,end), ...
        swingHeight,timevec(1), timevec(end), Ts); 
    stepinfo.swing = q; 
    
    % simulate stance
    for idx = 1:size(states,2)
        state1 = states(:,idx); 
        meas1 = Cd*state1 + Dd*u0; 
        
        bodypos = [fhold_x0 + meas1(1);
                   fhold_y0 + meas1(2);
                   zModel]; 
        % animate stance leg and COM traj 
        points = [fhold_x0, bodypos(1);
                  fhold_y0, bodypos(2);
                  0,        bodypos(3)]; 
        updateLine(hLeg, points)
        appendLine(hCom, bodypos); 
        
        % animate swing leg 
        if rem(stp,2) > 0
            appendLine(hLineRightFoot, q(:,idx))
        else
            appendLine(hLineLeftFoot, q(:,idx))
        end
        if animateOn
            pause(0.005);
        end
    end 
    % update graphics: change leg
    % animate body/foot with new foothold
    points = [fhold_x1, bodypos(1);
              fhold_y1, bodypos(2);
              0,        bodypos(3)]; 
    updateLine(hLeg, points)
    appendLine(hLineFootHold, [fhold_x1; fhold_y1; 0])
    if animateOn
        pause(0.005); 
    end
    
    % save the data 
    stepinfos{stp+2} = stepinfo;  
end

function [dx0, y0, dy0, tsinglesupport] = findInitialConditions(stepLength, dy_mid, x0, zModel, g, Ts)

    Tc = sqrt(zModel/g); 
    % Desired midstance y state
    y_mid = 0;
    % Corresponding orbital energy is
    E = -g/(2*zModel)*y_mid^2 + 0.5*dy_mid^2; 
    
    y0 = -stepLength/2;
    % finding dy0 from midstance energy level 
    dy0 = sqrt(2*(E+g/(2*zModel)*y0^2));
    
    % using relationship between final body state and initial body state,
    % we can find time it will take to reach midstance given final velocity
    % (dy = dy_mid) and final position (which is y = 0 at midstance) 
    tsinglesupport = 2*asinh(stepLength/2/(Tc*dy_mid))*Tc;
    tsinglesupport = floor(tsinglesupport/Ts)*Ts; % should be divisible by sample time 

    % also using relationship between final body state and initial body
    % state. We already know time it will take to reach midstance. 
    tf = tsinglesupport/2; 
    dx0 = -x0/Tc * sinh(tf/Tc) / cosh(tf/Tc);
end

function [states, timeVec, simStates] = stanceSim(fhold_x, fhold_y, state0, u0, modelVars, simStates)

    % Unpack variables
    Ts = modelVars.Ts;
    Ad = modelVars.Ad;
    Bd = modelVars.Bd;
    
    % Define time vector
    tInitial = simStates.timeVec(end) + modelVars.Ts; % don't let time overlap
    tFinal   = tInitial + modelVars.tSingleSupport; 
    timeVec  = tInitial:Ts:tFinal; 

    % Simulation loop
    nSteps = numel(timeVec); 
    states = zeros(size(state0,1),nSteps);  
    states(:,1) = state0; 
    for idx = 1:nSteps-1
        states(:,idx+1) = Ad*states(:,idx) + Bd*u0;
    end
    
    % Update the simStates structure
    simStates.timeVec = [simStates.timeVec timeVec(1:end-1)];
    newStates = [states(1,2:end)+fhold_x; states(3,2:end)+fhold_y; modelVars.zModel*ones([1 nSteps-1])];
    simStates.bodyPos = [simStates.bodyPos newStates];
end

function [state0, fhold_x, fhold_y] = changeLeg(state1,simStates)  
    % previous final states 
    xf  = state1(1); 
    dxf = state1(2); 
    yf  = state1(3); 
    dyf = state1(4); 

    % use simple control law (mirroring)
    x0 = -xf; 
    y0 = -yf; 
    dx0 = dxf; 
    dy0 = dyf; 
    state0 = [x0; dx0; y0; dy0]; 

    % new foothold
    fhold_x = simStates.bodyPos(1,end) - x0; 
    fhold_y = simStates.bodyPos(2,end) - y0;
end 

function appendLine(gHandle, points)
    gHandle.XData(end+1) = points(1); 
    gHandle.YData(end+1) = points(2); 
    gHandle.ZData(end+1) = points(3); 
end

function xi = findFootHold(xd, vd, vi, mvar)
    g = mvar.g; zModel = mvar.zModel; tSingleSupport = mvar.tSingleSupport; 
    a = 1; % weight for distance
    b = 1; % weight for velocity
    
    Tc = sqrt(zModel/g); 
    CT = cosh(tSingleSupport/Tc); 
    ST = sinh(tSingleSupport/Tc); 
    DT = a*CT^2 + b*(ST/Tc)^2; 
    
    xi = (a*CT*(xd - ST*Tc*vd) + b*ST/Tc*(vd-CT*vi))/DT; 
end




function [q, qd, qdd] = getSwingFootTraj(footpos0, footpos1, swingheight, timestp0, timestpf, Ts)
% Returns cubic polynomial trajectory for the robot's swing foot
%
% Copyright 2019 The MathWorks, Inc.

% Trajectory for X and Y
waypointsXY = [footpos0(1:2) footpos1(1:2)];
timestampsXY = [timestp0 timestpf];
timevecswingXY = timestampsXY(1):Ts:timestampsXY(end);
[XYq, XYqd, XYqdd, ~] = cubicpolytraj(waypointsXY, timestampsXY, timevecswingXY);

% Trajectory for Z
waypointsZ = [footpos0(3) footpos0(3)+swingheight footpos0(3)];
timestpMid = (timestp0+timestpf)/2; % Top of swing at midpoint
timestampsZ = [timestp0 timestpMid timestpf];
timevecswingZ = timestampsZ(1):Ts:timestampsZ(end);
[Zq, Zqd, Zqdd, ~] = cubicpolytraj(waypointsZ, timestampsZ, timevecswingZ);

% combine xy and z trajectory
q = [XYq; Zq];
qd = [XYqd; Zqd];
qdd = [XYqdd; Zqdd];

end



