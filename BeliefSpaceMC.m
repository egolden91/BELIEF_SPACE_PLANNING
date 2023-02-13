%% INIT 4
clear all
global count
count = 0;
Xgoal = [9,9];
Lambda = 1;
Rmin = 0.1;
d    = 1;
BeacPos = [0,0;1,1;2,2;3,3;4 4;5,5;6,6;7,7;8,8;9,9];
T = 11;
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
%% MAIN 4
depth = 5;
A = [1 0;-1 0;0,1;0 -1; 1/sqrt(2) 1/sqrt(2);-1/sqrt(2) 1/sqrt(2);1/sqrt(2) -1/sqrt(2);-1/sqrt(2) -1/sqrt(2); 0 0];
[BestA,BestU] = CalcSparseSample(B.mu(1,:),B.sig(:,:,1),BeacPos,P,A,Rmin,d,Xgoal,depth);





% figure(2)
% plot(xgt(:,1),xgt(:,2),'-x')
% hold on
% plot(Zbeac(Zbeac(:,1)~=-99,1),Zbeac(Zbeac(:,1)~=-99,2),'o','MarkerSize',3,'LineWidth',2)
% plot(BeacPos(:,1),BeacPos(:,2),'y*','MarkerSize',10,'LineWidth',2)
% grid on
% plot(proUpBeacon(:,1),proUpBeacon(:,2),'--gs')
% %% PLOTTING2
% xgt = xgt';
% BeacPos = BeacPos';
% % B.mu = B.mu';
% % B.sig = B.sig'; 
% figure(2)
% %H(1) = plot(xgt(1,:),xgt(2,:),'-x');
% hold on; grid on;
% % plot(BeacPos(1,:),BeacPos(2,:),'*','MarkerSize',10,'LineWidth',2)
% xlabel('X')
% ylabel('Y')
% H(2) = plot(B.mu(:,1),B.mu(:,1),'k'); %bel mean loc
% ColArr = {'red';'green';'magenta';'cyan'};
% for ii=1:length(B.mu)
%     ColArr_ii = ColArr{double(mod(ii,1)+1)}; 
%     drawCovarianceEllipse(B.mu(:,:,ii),B.sig(:,:,ii),ColArr_ii, '--')
%     plot(B.mu(1,1,ii),B.mu(1,2,ii),ColArr_ii,'Marker','*')    
% end
% legend('Observation','Beacons','Belief Propogation')
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
    z = [-99 -99];
    sig = -99*eye(2);
    mu  = RobPos;
    return
end
sig = 0.01*max(r,Rmin)*eye(2);
mu  = RobPos;
z = mvnrnd(mu,sig);
end
    
function [Cost]= CalcCostFunction(Xgoal,Mu,sigma,Lambda)
Cost = norm(Mu - Xgoal)+Lambda*det(sigma);
end


function [BeleifRnd,mu,sig] = TransitBeliefMDP(muPrev,sigPrev,P,a,BeacPos,Rmin,d)
%xgt =  SampleMotionModel(P,xgtPrev,A(ii,:));
[BeleifRnd,mu,sig] = PropagateBelief(muPrev,sigPrev,P,a); %PREDICT
[Zbeac,~,~] = GenerateObservationFromBeacons(BeleifRnd,BeacPos,Rmin,d); %CHECK FOR OBSERVATION
if Zbeac ~= -99
    [BeleifRnd,mu,sig] = PropagateUpdateBelief(muPrev,sigPrev,P,a,Zbeac); %IF OBSEREVED - KALMAN
end

end
    
function [BestA,BestU] = CalcSparseSample(muPrev,sigPrev,BeacPos,P,A,Rmin,d,Xgoal,depth)
global count 
if depth <= 0
    count = count + 1
    BestU = 0;
    BestA = 0;
    return
end
BestA = [];
BestU = inf;
for ii = 1:length(A)
    u = 0;
    [~,muUpdated,sigUpdated] = TransitBeliefMDP(muPrev,sigPrev,P,A(ii),BeacPos,Rmin,d);
    Cost = CalcCostFunction(Xgoal,muUpdated,sigUpdated,1);
    [BestANew,uNew] = CalcSparseSample(muUpdated,sigUpdated,BeacPos,P,A,Rmin,d,Xgoal,depth-1);
    u = u+Cost+uNew;
    if u <= BestU
        BestU = u;
        BestA = A(ii,:);
    end
end
end

  


