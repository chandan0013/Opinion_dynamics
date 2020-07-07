clear all

%% LOAD DATA

load('G_dolph');
load('M_dolph');
%load('x0_dolph');
load('A_dolph');
M = M_dolph;
G = G_dolph;
N = length(A);

%% SETUP

%%%% VARIABLES

jerk_goal = 0.7;               % control target for the jerk
jerk_node = 15;                % jerk's node in network
capped = 1;                    % boolean for capping jerk's opinion by [0-1]
random50 = 0;                  % boolean for random noise every 50 iterations
noise_factor = 0.01;           % scales random noise

controller = 0;                % controller type: proportional = 0, PI = 1;
kp = 1.5;                        % proportional gain

k = 1;l=1;                         % discrete time counter
max_k = 200;                   % max number of iterations
err = ones(k,3);               % errors of [Neighbor; Global; time step error]
converge = 0;                  % if converged = 1

Neighbors = neighbors(G,jerk_node);   % neighbors of jerk
num_N = length(Neighbors);     % number of neighbors to jerk

N_avg = zeros(k,1);            % Series of neighbor averages



avg = zeros(k,1);              % Series of global averages

a = 0.2;
b = 0.9;
x0 = (b-a).*rand(N,1) + a;      % intitial state between (a,b)open interval
avg(1) = mean(x0);             % average of initial conditions, for convergence criteria
x = zeros(N,k);                % series of state vector states
x_1 = zeros(N,l); 
%%%% State Matrices

% initializing state vector
x(:,1) = x0;                   
%x(jerk_node,1) = a;
temp_sum = 0;
    for iii = 1:num_N
        temp_sum = temp_sum + x(Neighbors(iii),k);
    end
    
% Averages/Error Update for initial state
N_avg(k) = temp_sum/num_N;

x_1(:,1) = x0;                   
%x_1(jerk_node,1) = a;
% Initialize B Matrix
B = zeros(1,N)';
B(jerk_node,1) = 1;

% Initialize C Matrix
C = zeros(1,N);
for i=1:num_N
    index = Neighbors(i);
    C(1,index) = 1/num_N;
end

% Initialize A Matrix
Ap = kp*B*C; % Proportional feedback to A matrix
A_pert = A-Ap;    % A with feedback


%% Opinion Dispersion
while (~converge) 
    
    %State transition
    x_new = A_pert*x(:,k)+kp*B*jerk_goal; 
    k=k+1;
    x(:,k) = x_new;
     
     
    % Neighbor Average
    temp_sum = 0;
    for iii = 1:num_N
        temp_sum = temp_sum + x(Neighbors(iii),k);
    end
    
    % Averages/Error Update
    N_avg(k) = temp_sum/num_N;
    avg(k) = mean(x(:,k));
    err(k,1) = jerk_goal - N_avg(k); %Neighbor Error
    err(k,2) = jerk_goal - avg(k);  %Global Error    

%     temp_jerk = x(jerk_node,k);
%     rand_factor = 2*rand()-1;
%     
%     for iii = 1:N
% %        rand_factor = 2*rand()-1;
%         if rand_factor > 0
%             x(iii,k) = min(x(iii,k)+noise_factor*rand_factor,1);
%         else
%             x(iii,k) = max(x(iii,k)+noise_factor*rand_factor,0);
%         end
%     end
%     
%     x(jerk_node,k) = temp_jerk;
    
    
    % Apply Capping
    if (capped)
        if x(jerk_node,k)>1
            x(jerk_node,k) = 1;
        end
        if x(jerk_node,k)<0
            x(jerk_node,k) = 0;
        end
    end
    
% Check converge
%      if k > max_k
%          converge = 1;
%      end
     if abs(avg(k-1)-avg(k))<0.0001 && k > 10
         converge = 1;
     end
     
% Code scripted on June 10, to verify the boundness of the error terms
%  x[k+1] = [each node's avg at kth ieration) + k_p * (avg error at kth ieration) * [B]
% 
     x_new0(:,l) = A*x_1(:,l);
     diff(l) = 1- x_1(jerk_node,l);
     temp_sum = 0;
     for iii = 1:num_N
         temp_sum = temp_sum + x_new0(Neighbors(iii),l);
     end
     
     % Averages/Error Update
     N_avg1(l) = temp_sum/num_N;
     avg1(l) = mean(x_1(:,l));
     err(l,3) = kp*(jerk_goal - N_avg1(l));
     x_new1 = x_new0(:,l)+kp*B*err(l,3); 
     l=l+1;
     x_1(:,l) = x_new1;
     
      
      % Neighbor Average
     if (capped)
        if x_1(jerk_node,l)>1
            x_1(jerk_node,l) = 1;
        end
        if x_1(jerk_node,l)<0
            x_1(jerk_node,l) = 0;
        end
    end
end

%% FIGURES
figure(1);
plot(G);
title('Dolphin 62');

figure(2);
x_ax = 1:1:k;
goal = jerk_goal*ones(length(x_ax));
plot(x_ax, avg,'r', x_ax, x(jerk_node,:), 'b', x_ax,N_avg, 'g', x_ax, goal, 'k--')
xlim([-5 k+k/10])
limyu = max(x(jerk_node,:))+0.25;
limyl = min(min(x(jerk_node,:))-0.15,0);
ylim([limyl limyu])
legend('Global Average','Jerk','Neighbors Average','Jerk Goal')
title1 = strcat('P: ',num2str(kp),', Capped = ',num2str(capped));
title(title1)
xlabel('iterations')
ylabel('continuous opinion (0-1)')

figure(3);
x_ax1 = 1:1:l-1;
goal1 = jerk_goal*ones(length(x_ax1));
plot(x_ax1, avg1,'r', x_ax, x_1(jerk_node,:), 'b' ,x_ax1,N_avg1, 'g', x_ax1, goal1, 'k--')
xlim([-5 k+k/10])
limyu = max(x_1(jerk_node,:))+0.25;
limyl = min(min(x(jerk_node,:))-0.15,0);
ylim([limyl limyu])
legend('Global Average1','Jerk','Neighbors Average1','Jerk Goal')
title1 = strcat('P: ',num2str(kp),', Capped = ',num2str(capped));
title(title1)
xlabel('iterations')
ylabel('continuous opinion (0-1)')


% error term boundedness plots
figure(4)
plot(x_ax, kp*err(:,3)','r', x_ax, x_1(jerk_node,:), 'b',x_ax1, x_new0(jerk_node,:),'k--')
xlim([-5 k+k/10])
limyu = max(x_1(jerk_node,:))+0.25;
limyl = min(min(x(jerk_node,:))-0.15,0);
ylim([limyl limyu])
legend('Control input','Jerk','Jerk Neighbor average with no error')
title1 = strcat('P: ',num2str(kp),', Capped = ',num2str(capped));
title(title1)
xlabel('iterations')
ylabel('continuous opinion (0-1)')

figure(5)
plot(x_ax1,diff,'b',x_ax, kp*err(:,3)','r')
xlim([-5 k+k/10])
limyu = max(x_1(jerk_node,:))+0.25;
limyl = min(min(x(jerk_node,:))-0.15,0);
ylim([limyl limyu])
legend('Limit for capping','Control input')
title1 = strcat('P: ',num2str(kp),', Capped = ',num2str(capped));
title(title1)
xlabel('iterations')
ylabel('continuous opinion (0-1)')

figure(8)
x_ax = 1:1:k;
bound1 = a*ones(length(x_ax));
bound2 = b*ones(length(x_ax));
plot(x_ax, N_avg,'r', x_ax, x(jerk_node,:), 'b', x_ax,bound1, 'k--', x_ax, bound2, 'k--')
xlim([-5 k+k/10])
limyu = max(x(jerk_node,:))+0.25;
limyl = min(min(x(jerk_node,:))-0.15,0);
ylim([limyl limyu])
legend('Myopic Average','Jerk','lower bound','upper bound')
title1 = strcat('P: ',num2str(kp),', Capped = ',num2str(capped));
title(title1)
xlabel('iterations')
ylabel('continuous opinion (0-1)')

% each agents opinion over time
% figure(9);
% x_ax = 1:1:k;
% cap = jerk_goal*ones(length(x_ax));
% for i = 1:N
%     hold on
%     plot(x_ax, x(i,:),'r', x_ax, cap, 'k--')
%     xlim([-5 k+k/10])
%     ylim([0 2])
% end
% hold off
% legend('Agents opinion dynamics','Jerk Goal')
% title1 = strcat('P: ',num2str(kp),', Capped = ',num2str(capped));
% title(title1)
% xlabel('iterations')
% ylabel('continuous opinion (0-1)')

% Maximum opinion of all agents in each time step
 for i = 1:k
     cap_max(i) = max(x(:,i));
 end
figure(10)
plot(x_ax, N_avg,'r', x_ax, cap_max, 'b', x_ax,bound1, 'k--', x_ax, bound2, 'k--')
xlim([-5 k+k/10])
limyu = max(x(jerk_node,:))+0.25;
limyl = min(min(x(jerk_node,:))-0.15,0);
ylim([limyl limyu])
legend('Myopic Average','Max opinion','lower bound','upper bound')
title1 = strcat('P: ',num2str(kp),', Capped = ',num2str(capped));
title(title1)
xlabel('iterations')
ylabel('continuous opinion (0-1)')
 
