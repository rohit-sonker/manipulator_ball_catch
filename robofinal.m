clear all;
syms a1 a2 a3;
d1=1;
l1=1;
l2=1;
a4=0;
%dh table
% D = [0 0 d1 a1;0 -pi/2 0 a2;l1 0 0 a3;0 pi/2 l2 a4];
D = [0 0 d1 a1;0 pi/2 0 a2;l1 0 0 a3;0 pi/2 l2 a4];
%transformation matrices
for i=1:4
    T{i}=[cos(D(i,4)) -sin(D(i,4)) 0 D(i,1);
          sin(D(i,4))*cos(D(i,2)) cos(D(i,4))*cos(D(i,2)) -sin(D(i,2)) -sin(D(i,2))*D(i,3);
          sin(D(i,4))*sin(D(i,2)) cos(D(i,4))*sin(D(i,2)) cos(D(i,2)) cos(D(i,2))*D(i,3);
          0 0 0 1];
end 
Tf = T{1}*T{2}*T{3}*T{4};
a=[a1 a2 a3];

theta1 = [0 0 0]; %initial position
point1=[1 0 0];
%point2 = [ 1 0.6 0.3];

%%%-------------------ball straight line trajectory----------------%%%%%%%
pta = [4 ,3 ,3];
ptb = [0,0,0];
pt_d = ptb-pta;
speed = 0.3;
ballstepno = norm(pt_d)/speed; %scalar
ballstepno = ceil(ballstepno); 
ballpath = zeros(ballstepno+1,3);
for k = 0:ballstepno
    ballpath(k+1,:) = pta + pt_d.*(k/ballstepno); 
end;

%lets calculate pt of intersection
%we take the point on trajectory nearest to start point
for k = 1:size(ballpath,1)
    distances(k) = norm(ballpath(k,:)-point1);
end;

[~,index] = min(distances);

point2 = ballpath(index,:);

%index gives the time !
tsteps = index;


%%%%%------solve for the point----------%%%%%
P=Tf(1:3,4);

e1 = P(1)==point2(1);
e2 = P(2)==point2(2);
e3 = P(3)==point2(3);
res = solve(e1,e2,e3);
%req res
for i = 1:size(res.a1,1)
    if res.a1(i)>-pi && res.a1(i)<pi
        if res.a2(i)>-pi/2 && res.a2(i)<pi/2
            if res.a3(i)>-pi/2 && res.a3(i)<pi/2
                indexr = i;
                break;
            end;
        end;
    end;
end;
temp = [res.a1 res.a2 res.a3];
theta2 = temp(indexr,:);


% 
% %%%------DH Table--------------------%%%
% Dhval = [0 0 d1 theta2(1);
%         0 pi/2 0 theta2(2);
%         l1 0 0 theta2(3);
%         0 pi/2 l2 0];
% %%%------General Transform Matrix----%%%
% for i=1:4
%     Tval{i}=[cos(Dhval(i,4)) -sin(Dhval(i,4)) 0 Dhval(i,1);
%           sin(Dhval(i,4))*cos(Dhval(i,2)) cos(Dhval(i,4))*cos(Dhval(i,2)) -sin(Dhval(i,2)) -sin(Dhval(i,2))*Dhval(i,3);
%           sin(Dhval(i,4))*sin(Dhval(i,2)) cos(Dhval(i,4))*sin(Dhval(i,2)) cos(Dhval(i,2)) cos(Dhval(i,2))*Dhval(i,3);
%           0 0 0 1];
% end
% %%%------Transform matrix to global frame-------%%%%%
% T01 = Tval{1};
% T02 = Tval{1}*Tval{2};
% T03 = T02 * Tval{3};
% T04 = T03 * Tval{4};
% %%%-------getting global coordinates-----------%%%%%
% link1 = T01 * [0;0;0;1];
% link2 = T02 * [l1;0;0;1];
% link3 = T03 * [0;-l2;0;1];
% 
% for i = 1:3
%     if link1(i) < 0.0001
%         link1(i) = 0;
%     end;
%     if link2(i) < 0.0001
%         link2(i) = 0;
%     end;
%     if link3(i) < 0.0001
%         link3(i) = 0;
%     end;
% end;
% 
%  %%%---- combining coordinates for display--------%%%%
% robo_x = [0 link1(1) link2(1) link3(1)];
% robo_y = [ 0 link1(2) link2(2) link3(2)];
% robo_z = [0 link1(3) link2(3) link3(3)];
% 
% figure(1);
% ax = gca;
% ax.XLim = [0 +2];
% ax.YLim = [0 +2];
% ax.ZLim = [0 +2];
% plot3(robo_x,robo_y,robo_z);
% ax.XLim = [-2 +2];
% ax.YLim = [-2 +2];
% ax.ZLim = [0 +2];
% grid on;
% xlabel('X axis');
% ylabel('Y axis');
% zlabel('Z axis');
% hold on;
% scatter3(point2(1),point2(2),point2(3));

%%%-----------------drawing trajectory-----------------------%%%%%%%%%%%%%%
figure(1);

scatter3(point1(1),point1(2),point1(3),'x');
hold on;
scatter3(point2(1),point2(2),point2(3),'x');
hold on;
ax = gca;
ax.XLim = [-2 +2];
ax.YLim = [-2 +2];
ax.ZLim = [-2 +2];


%%%-----motion-----------------------%%%
%tsteps =20;
for t = 0:1:tsteps
    %theta = theta1 + (t/tsteps)*(theta2-theta1);
    
    %need a cubic equation not linear.
    
    
    a1 = theta1;
    a2 = 0;
    a3 = (3/(tsteps^2)) * (theta2 - theta1);
    a4 =  (-2/(tsteps^3)) * (theta2 - theta1);
    theta = a1+ a2*t + a3*(t^2) + a4*(t^3);
%%%------DH Table--------------------%%%
Dhval = [0 0 d1 theta(1);
        0 pi/2 0 theta(2);
        l1 0 0 theta(3);
        0 pi/2 l2 0];
%%%------General Transform Matrix----%%%
for i=1:4
    Tval{i}=[cos(Dhval(i,4)) -sin(Dhval(i,4)) 0 Dhval(i,1);
          sin(Dhval(i,4))*cos(Dhval(i,2)) cos(Dhval(i,4))*cos(Dhval(i,2)) -sin(Dhval(i,2)) -sin(Dhval(i,2))*Dhval(i,3);
          sin(Dhval(i,4))*sin(Dhval(i,2)) cos(Dhval(i,4))*sin(Dhval(i,2)) cos(Dhval(i,2)) cos(Dhval(i,2))*Dhval(i,3);
          0 0 0 1];
end
%%%------Transform matrix to global frame-------%%%%%
T01 = Tval{1};
T02 = Tval{1}*Tval{2};
T03 = T02 * Tval{3};
T04 = T03 * Tval{4};
%%%-------getting global coordinates-----------%%%%%
link1 = T01 * [0;0;0;1];
link2 = T02 * [l1;0;0;1];
link3 = T03 * [0;-l2;0;1];

for i = 1:3
    if link1(i) < 0.0001
        link1(i) = 0;
    end;
    if link2(i) < 0.0001
        link2(i) = 0;
    end;
    if link3(i) < 0.0001
        link3(i) = 0;
    end;
end;

 %%%---- combining coordinates for display--------%%%%
robo_x = [0 link1(1) link2(1) link3(1)];
robo_y = [ 0 link1(2) link2(2) link3(2)];
robo_z = [0 link1(3) link2(3) link3(3)];

figure(1);
ax = gca;
ax.XLim = [0 +2];
ax.YLim = [0 +2];
ax.ZLim = [0 +2];
plot3(robo_x,robo_y,robo_z);
ax.XLim = [-2 +2];
ax.YLim = [-2 +2];
ax.ZLim = [0 +2];
grid on;
scatter3(ballpath(t+1,1),ballpath(t+1,2),ballpath(t+1,3));
hold on;
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
hold on;

pause(0.2);

end;