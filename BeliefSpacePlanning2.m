%% INIT1
clear all
close all
F     = [1.0 0.0; 0.0 1.0]; %X coeffs
h     = [1.0 0.0; 0.0 1.0]; %Z coeffs
a     = [1.0, 0.0].';%[1.0, 0.0]; %motion command
xgt0  = [0.5, 0.2].'; % % X initial location
T     = 11; %duration - changed to 11 because 0 is also an index
xgt   = zeros(2,T); %init locations
z     = zeros(2,T); %init observations
probB = zeros(2,T); %belifeUpdate init 
PropUpB = zeros(2,T); %
sigW  =  0.1^2*[1.0 0.0; 0.0 1.0]; %X uncertainty  
sigV  =  0.01^2*[1.0 0.0; 0.0 1.0]; %Z uncertainty
muB0   =  [0,0].'; % first belife
sigB0  =  [1,0;0,1]; %first belife variance

B     = struct;
B.muPr(:,1)  = muB0;
B.sigPr(:,:,1) = sigB0;

B.muUp(:,1)  = muB0;
B.sigUp(:,:,1) = sigB0;

P   = struct;
P.F = F;
P.sigV = sigV;
P.sigW = sigW;
P.h = h;
xgt(:,1) = xgt0; 



%% MAIN1
for ii = 1:T-1
   xgt(:,ii+1) =  SampleMotionModel(P,xgt(:,ii),a);
end

for zz = 1:T
    [z(:,zz)] = GenerateObservation(P,xgt(:,zz));
end

for bb = 1:T-1
    [probB(:,bb+1),B.muPr(:,bb+1),B.sigPr(:,:,bb+1)] = PropagateBelief(B.muPr(:,bb),B.sigPr(:,:,bb),P,a);
    
    [PropUpB(:,bb+1),B.muUp(:,bb+1),B.sigUp(:,:,bb+1)] = PropagateUpdateBelief(B.muUp(:,bb),B.sigUp(:,:,bb),P,a,z(:,bb));
end



%% PLOTTING1
figure(1)
H = zeros(1,2);
H(1) = plot(xgt(1,:),xgt(2,:),'b-x'); %actual loc
hold on
grid on
% plot(z(1,:),z(2,:),'-o')
% title('Motion Model')
xlabel('X')
ylabel('Y')
% plot(probB(1,:),probB(2,:),'k--*'); %bel loc
H(2) = plot(B.muPr(1,:),B.muPr(2,:),'k'); %bel mean loc
% legend('Actual','Bel Mu')
ColArr = {'red';'green';'magenta';'cyan'};
for ii=1:length(B.muPr)
    ColArr_ii = ColArr{double(mod(ii,4)+1)}; 
    drawCovarianceEllipse(B.muPr(:,ii),B.sigPr(:,:,ii),ColArr_ii, '--')
    plot(B.muPr(1,ii),B.muPr(2,ii),ColArr_ii,'Marker','*')
%     plot([probB(1,ii) xgt(1,ii)],[probB(2,ii) xgt(2,ii)],'y')
    
end
title('q1 section D iii')
legend(H([1 2]),'real Pos','Bel mean');
hold off;
figure(2)
H(1) = plot(xgt(1,:),xgt(2,:),'b-x'); %actual loc
hold on
grid on
% plot(z(1,:),z(2,:),'-o')
% title('Motion Model')
xlabel('X')
ylabel('Y')
% plot(PropUpB(1,:),PropUpB(2,:),'k--*') %bel loc
H(2) = plot(B.muUp(1,:),B.muUp(2,:),'k'); %bel loc
% legend('Actual','Bel Mu')
ColArr = {'red';'green';'magenta';'cyan'};
for ii=1:length(B.muUp)
    ColArr_ii = ColArr{double(mod(ii,4)+1)}; 
    drawCovarianceEllipse(B.muUp(:,ii),B.sigUp(:,:,ii),ColArr_ii, '--')
    plot(B.muUp(1,ii),B.muUp(2,ii),ColArr_ii,'Marker','*')
%     plot([PropUpB(1,ii) xgt(1,ii)],[PropUpB(2,ii) xgt(2,ii)],'y')
    
end
title('q1 section D iv')
legend(H([1 2]),'real Pos','Bel mean');
hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% INIT2
Rmin = 0.1;
d    = 1;
BeacPos = [0,0;1.5,1.5;3,3;4.5,4.5;6 6;7.5,7.5;9,9;2.5,5;6.5,4].';
T = 100; %duration - changed to 101 because 0 is also an index
xgt0  = [-0.5, -0.2].';
a     = [0.1,0.1].';
F   = eye(2);
h   = eye(2);
P.F = F;
P.h = h;
sigW  =  0.1^2*[1.0 0.0; 0.0 1.0];
sigV  =  0.01^2*[1.0 0.0; 0.0 1.0];
P.sigV = sigV;
P.sigW = sigW;
xgt(:,1) = xgt0;

muB0   =  [0,0].';
sigB0  =  [1,0;0,1];


B.mu(:,1)  = muB0;
B.sig(:,:,1) = sigB0;

%% MAIN2
ZB.mu(:,1) = [0 0].';ZB.sig(:,:,1) = ((norm([0.5 0.2])*0.01)^2)*eye(2);
for ii = 1:T-1
   [Zbeac(:,ii+1),ZB.mu(:,ii+1),ZB.sig(:,:,ii+1)] = GenerateObservationFromBeacons(xgt(:,ii),BeacPos,Rmin,d);
    xgt(:,ii+1) =  SampleMotionModel(P,xgt(:,ii),a);
end
% for no observation
for bb = 1:T-1
    [probB(:,bb+1),B.muPr(:,bb+1),B.sigPr(:,:,bb+1)] = PropagateBelief(B.muPr(:,bb),B.sigPr(:,:,bb),P,a);
end

figure(3)
H = zeros(1,2);
H(1) = plot(xgt(1,:),xgt(2,:),'b-x'); %actual loc
hold on
grid on
% plot(z(1,:),z(2,:),'-o')
% title('Motion Model')
xlabel('X')
ylabel('Y')
% plot(probB(1,:),probB(2,:),'k--*'); %bel loc
H(2) = plot(B.muPr(1,:),B.muPr(2,:),'k'); %bel mean loc
% legend('Actual','Bel Mu')
ColArr = {'red';'green';'magenta';'cyan'};
for ii=1:length(B.muPr)
    ColArr_ii = ColArr{double(mod(ii,4)+1)}; 
    drawCovarianceEllipse(B.muPr(:,ii),B.sigPr(:,:,ii),ColArr_ii, '--')
    plot(B.muPr(1,ii),B.muPr(2,ii),ColArr_ii,'Marker','*')
%     plot([probB(1,ii) xgt(1,ii)],[probB(2,ii) xgt(2,ii)],'y')
    
end
title('q2 section c i -no observations')
H(3) = plot(BeacPos(1,:),BeacPos(2,:),'y*','MarkerSize',10,'LineWidth',2);
legend(H([1 2 3]),'real Pos','Bel mean','Beacons');
hold off;

% for the case with observation
B.mu(:,1)  = muB0;
B.sig(:,:,1) = sigB0;
for bb = 1:T-1
    if Zbeac(1,bb) == -99
        [proUpBeacon(:,bb+1),B.mu(:,bb+1),B.sig(:,:,bb+1)] = PropagateBelief(B.mu(:,bb),B.sig(:,:,bb),P,a);
    else
        [proUpBeacon(:,bb+1),B.mu(:,bb+1),B.sig(:,:,bb+1)] = PropagateUpdateBelief(B.mu(:,bb),B.sig(:,:,bb),P,a,Zbeac(:,bb),ZB.sig(:,:,bb));
    end
end

%% PLOTTING2
figure(4)
H(1) = plot(xgt(1,:),xgt(2,:),'-x');
hold on; grid on;
% plot(Zbeac(1,(Zbeac(1,:)~=-99)),Zbeac(2,(Zbeac(1,:)~=-99)),'o','MarkerSize',3,'LineWidth',2)
plot(BeacPos(1,:),BeacPos(2,:),'*','MarkerSize',10,'LineWidth',2)
xlabel('X')
ylabel('Y')
% plot(probB(1,:),probB(2,:),'k--*'); %bel loc
H(2) = plot(B.mu(1,:),B.mu(2,:),'k'); %bel mean loc
% legend('Actual','Bel Mu')
ColArr = {'red';'green';'magenta';'cyan'};
for ii=1:length(B.mu)
    ColArr_ii = ColArr{double(mod(ii,4)+1)}; 
    drawCovarianceEllipse(B.mu(:,ii),B.sig(:,:,ii),ColArr_ii, '--')
    plot(B.mu(1,ii),B.mu(2,ii),ColArr_ii,'Marker','*')
%     plot([probB(1,ii) xgt(1,ii)],[probB(2,ii) xgt(2,ii)],'y')
    
end
title('q2 section c i -with observations')
H(3) = plot(BeacPos(1,:),BeacPos(2,:),'y*','MarkerSize',10,'LineWidth',2);
legend(H([1 2 3]),'real Pos','Bel mean','Beacons');
viscircles(BeacPos.',ones(1,9),'Color','y')
hold off;

figure(5)
hold on
for ii=1:(T-1)
    esr_err(ii) = norm(xgt(:,ii)-B.mu(:,ii));
end
plot((1:(T-1)),esr_err);
title('est err vs time')
xlabel('time [steps]') ; ylabel('est err [norm of differance]')
hold off

figure(6)
hold on
for ii=1:(T-1)
    tr_cov(ii) = trace(B.sig(:,:,ii));
end
plot((1:(T-1)),tr_cov);
title('trace of cov matrix')
xlabel('time [steps]') ; ylabel('trace of cov matrix')
hold off
%% FUNCTIONS

function[PropUpB,muB,sigB]=PropagateUpdateBelief(Bmu,Bsig,P,a,z,S_V)
if nargin==6
    P.sigV = S_V;
end
% %predict:
muP = P.F*Bmu+a; 
sigP = P.F*Bsig*P.F'+P.sigW;
%update:
K = sigP*P.h'/(P.h*sigP*P.h'+P.sigV);
muB  =  muP+K*(z-P.h*muP);
sigB = (eye(2)-K*P.h)*sigP;
PropUpB = (mvnrnd(muB,sigB)).';
end

function[PropB,muP,sigP]=PropagateBelief(Bmu,Bsig,P,a)
%Predict:
muP = P.F*Bmu+a;
sigP = P.F*Bsig*P.F'+P.sigW;
PropB = (mvnrnd(muP,sigP)).';
end

function[upX] = SampleMotionModel(P,X,a)
%UpX = X*F+a+w
mu =  P.F*X+P.h*a;
sig =  P.sigW;
upX = (mvnrnd(mu, sig)).';
end


function[z] = GenerateObservation(P,X)
%z = X+V
mu = X;  
sig  =  P.sigV;
z = (mvnrnd(mu,sig)).';
end

function [z,mu,sig]= GenerateObservationFromBeacons(RobPos,BeconPos,Rmin,d)
%z = X+mvnrnd([0,0.01*max(r,rmin)*eye(2)]);
dist = zeros(1,9);
for i = 1:length(BeconPos)
    dist(i) = norm(BeconPos(:,i) - RobPos);
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
% sig = ((0.01*max(r,Rmin))^2)*eye(2); % q2 sec  c i
sig = (0.01^2)*eye(2); % q2 sec  c ii
mu  = RobPos;
z = (mvnrnd(mu,sig)).';
end
    


    
    
  


