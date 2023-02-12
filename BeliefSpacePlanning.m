%% INIT1
clear all
close all
F     = [1.0 0.0; 0.0 1.0];
a     = [1.0, 0.0];%[1.0, 0.0];
xgt0  = [0.5, 0.2];
T     = 10;
xgt   = zeros(T,2);
z     = zeros(T,2);
probB = zeros(T,2);
PropUpB = zeros(T,2);
sigW  =  0.1^2*[1.0 0.0; 0.0 1.0];
sigV  =  0.01^2*[1.0 0.0; 0.0 1.0];
muB0   =  [0,0];
sigB0  =  [1,0;0,1];

B     = struct;
B.muPr(1,:)  = muB0;
B.sigPr(:,:,1) = sigB0;

B.muUp(1,:)  = muB0;
B.sigUp(:,:,1) = sigB0;

P   = struct;
P.F = F;
P.sigV = sigV;
P.sigW = sigW;
xgt(1,:) = xgt0; 



%% MAIN1
for ii = 1:T-1
   xgt(ii+1,:) =  SampleMotionModel(P,xgt(ii,:),a);
end

for zz = 1:T
    [z(zz,:)] = GenerateObservation(P,xgt(zz,:));
end

for bb = 1:T-1
    [probB(bb+1,:),B.muPr(:,:,bb+1),B.sigPr(:,:,bb+1)] = PropagateBelief(B.muPr(:,:,bb),B.sigPr(:,:,bb),P,a);
    
    [PropUpB(bb+1,:),B.muUp(:,:,bb+1),B.sigUp(:,:,bb+1)] = PropagateUpdateBelief(B.muUp(:,:,bb),B.sigUp(:,:,bb),P,a,z(bb,:));
end



figure(1)
plot(xgt(:,1),xgt(:,2),'-x')
hold on
grid on
plot(z(:,1),z(:,2),'-o')
title('Motion Model')
xlabel('X')
ylabel('Y')
plot(probB(:,1),probB(:,2),'--*')
plot(PropUpB(:,1),PropUpB(:,2),'--*')
legend('Model','Observation','Belief','Updated Belief')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% INIT2
clear all
Rmin = 0.1;
d    = 1;
BeacPos = [0,0;1,1;2,2;3,3;4 4;5,5;6,6;7,7;8,8;9,9];
T = 100;
xgt0  = [-0.5, -0.2];
a     = [0.1,0.1];
F   = eye(2);
P.F = F;
sigW  =  0.1^2*[1.0 0.0; 0.0 1.0];
sigV  =  0.01^2*[1.0 0.0; 0.0 1.0];
P.sigV = sigV;
P.sigW = sigW;
xgt(1,:) = xgt0;

muB0   =  [0,0];
sigB0  =  [1,0;0,1];


B.mu(1,:)  = muB0;
B.sig(:,:,1) = sigB0;

%% MAIN2
for ii = 1:T-1
   xgt(ii+1,:) =  SampleMotionModel(P,xgt(ii,:),a);
   [Zbeac(ii+1,:),~,~] = GenerateObservationFromBeacons(xgt(ii,:),BeacPos,Rmin,d);
end

for bb = 1:T
    if Zbeac(bb,1) == -99
        [proUpBeacon(bb+1,:),B.mu(:,:,bb+1),B.sig(:,:,bb+1)] = PropagateBelief(B.mu(:,:,bb),B.sig(:,:,bb),P,a);
    else
        [proUpBeacon(bb+1,:),B.mu(:,:,bb+1),B.sig(:,:,bb+1)] = PropagateUpdateBelief(B.mu(:,:,bb),B.sig(:,:,bb),P,a,Zbeac(bb,:));
    end
end

figure(2)
plot(xgt(:,1),xgt(:,2),'-x')
hold on
plot(Zbeac(Zbeac(:,1)~=-99,1),Zbeac(Zbeac(:,1)~=-99,2),'o','MarkerSize',3,'LineWidth',2)
plot(BeacPos(:,1),BeacPos(:,2),'*','MarkerSize',10,'LineWidth',2)
grid on
plot(proUpBeacon(:,1),proUpBeacon(:,2),'gs')
legend('Motion','Observation','Beacons','Belief Propogation')
%% INIT 3
clear all
Rmin = 0.1;
d    = 1;
BeacPos = [0,0;1,1;2,2;3,3;4 4;5,5;6,6;7,7;8,8;9,9];
T = 100;
xgt0  = [-0.5, -0.2];
a     = [0.1,0.1];
F   = eye(2);
P.F = F;
sigW  =  0.1^2*[1.0 0.0; 0.0 1.0];
sigV  =  0.01^2*[1.0 0.0; 0.0 1.0];
P.sigV = sigV;
P.sigW = sigW;
xgt(1,:) = xgt0;

muB0   =  [0,0];
sigB0  =  [1,0;0,1];


B.mu(1,:)  = muB0;
B.sig(:,:,1) = sigB0;
N = 10;
%% MAIN 3
for aa = 1:N
    a = [0.1,0.1*(aa/5)];
    for ii = 1:T-1
        xgt(ii+1,:) =  SampleMotionModel(P,xgt(ii,:),a);
        [Zbeac(ii+1,:),~,~] = GenerateObservationFromBeacons(xgt(ii,:),BeacPos,Rmin,d);
    end
    
    for bb = 1:T
        if Zbeac(bb,1) == -99
            [proUpBeacon(bb+1,:),B.mu(:,:,bb+1),B.sig(:,:,bb+1)] = PropagateBelief(B.mu(:,:,bb),B.sig(:,:,bb),P,a);
        else
            [proUpBeacon(bb+1,:),B.mu(:,:,bb+1),B.sig(:,:,bb+1)] = PropagateUpdateBelief(B.mu(:,:,bb),B.sig(:,:,bb),P,a,Zbeac(bb,:));
        end
    end
    %The final covariance should be minimal
    CostFunc(aa) = det(B.sig(end));
    [BestPolicyValue,BestPolicyIdx] = min(CostFunc);
    figure(4)
    plot(xgt(:,1),xgt(:,2),'-x')
    hold on
    %plot(Zbeac(Zbeac(:,1)~=-99,1),Zbeac(Zbeac(:,1)~=-99,2),'o','MarkerSize',3,'LineWidth',2)
    grid on
    %plot(proUpBeacon(:,1),proUpBeacon(:,2),'gs')
    title('Beacons Trajectory')
    Legend{aa}=strcat('Trajectory: ', num2str(aa),' Cost: ',num2str(CostFunc(aa)));

end
plot(BeacPos(:,1),BeacPos(:,2),'*','MarkerSize',10,'LineWidth',2)
legend(Legend);
disp(['Best action sequence idx: ',num2str(BestPolicyIdx),' With actions: ',num2str([0.1,0.1*(BestPolicyIdx/5)])])
%% FUNCTIONS

function[PropUpB,muB,sigB]=PropagateUpdateBelief(Bmu,Bsig,P,a,z)
% %predict:
muP = Bmu*P.F+a; 
sigP = P.F*Bsig*P.F'+P.sigW;
%update:
K = sigP*inv(sigP+P.sigV);
muB  =  muP+(z-muP)*K;
sigB = (eye(2)-K)*sigP;
PropUpB = mvnrnd(muB,sigB);
end

function[PropB,muP,sigP]=PropagateBelief(Bmu,Bsig,P,a)
%Predict:
muP = Bmu*P.F+a;
sigP = P.F*Bsig*P.F'+P.sigW;
PropB = mvnrnd(muP,sigP);
end

function[upX] = SampleMotionModel(P,X,a)
%UpX = X*F+a+w
mu =  X*P.F+a;
sig =  P.sigW;
upX = mvnrnd(mu, sig);
end


function[z] = GenerateObservation(P,X)
%z = X+V
mu = X;  
sig  =  P.sigV;
z = mvnrnd(mu,sig);
end

function [z,mu,sig]= GenerateObservationFromBeacons(RobPos,BeconPos,Rmin,d)
%z = X+mvnrnd([0,0.01*max(r,rmin)*eye(2)]);
dist = zeros(1,9);
for i = 1:length(BeconPos)
    dist(i) = norm(BeconPos(i) - RobPos);
end

dist = dist(dist<d);
if ~isempty(dist)
[r,RelBecon] = min(dist);
else
    z = -99;
    sig = -99*eye(2);
    mu  = RobPos;
    return
end
sig = 0.01*max(r,Rmin)*eye(2);
mu  = RobPos;
z = mvnrnd(mu,sig);
end
    


    
    
  


