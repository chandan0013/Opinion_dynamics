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

jerk_goal = 0.56;               % control target for the jerk
jerk_node = 15;                % jerk's node in network
test_node1 = 21;
test_node2 = 30;
capped = 1;                    % boolean for capping jerk's opinion by [0-1]
global_vision = 0;
controller = 0;                % controller type: proportional = 0, PI = 1;
kp = 5;                        % proportional gain

k = 1;l=1;                         % discrete time counter
max_k = 150;                   % max number of iterations
err = zeros(k,2);               % errors of [Neighbor; Global]
converge = 0;                  % if converged = 1

Neighbors = neighbors(G,jerk_node);   % neighbors of jerk
num_N = length(Neighbors);     % number of neighbors to jerk

N_avg = zeros(k,1);            % Series of neighbor averages
avg = zeros(k,1);              % Series of global averages
m = zeros(k,1);

a = 0.2;
b = 0.8;
x0 = (b-a).*rand(N,1) + a;      % intitial state between (a,b)open interval
x = zeros(N,k);                % series of state vector states

%%%% State Matrices

% initializing state vector
x(:,1) = x0;     
x(test_node1,1) = 0.99;
x(test_node2,1) = 0.15;
avg(k) = mean(x);             % average of initial conditions, for convergence criteria
m(k) = max(x);
temp_sum = 0;
    for iii = 1:num_N
        temp_sum = temp_sum + x(Neighbors(iii),k);
    end    
% Averages/Error Update for initial state
N_avg(k) = temp_sum/num_N;
err(k,1) = jerk_goal - N_avg(k); %Neighbor Error
err(k,2) = jerk_goal - avg(k);  %Global Error  
                  
%
% Initialize B Matrix
B = zeros(1,N)';
B(jerk_node,1) = 1;

% Initialize C Matrix
C = zeros(1,N);
if global_vision == 0
    for i=1:num_N
        index = Neighbors(i);
        C(1,index) = 1/num_N;
    end
else
    C(1:N) = (1/N)*ones(1,N);
end


% Initialize A Matrix
Ap = kp*B*C; % Proportional feedback to A matrix
A_pert = A-Ap;    % A with feedback
A_last = zeros(1,N+1);A_last(N+1) = 1;
A = [A_pert kp*B]; 
A = [A; A_last];
x_hat = zeros(N+1,k);
x_hat(:,k) = [x ; jerk_goal];
%% Opinion Dispersion
while (~converge) 
    
    %State transition
    x_new = A*x_hat(:,k);
    k=k+1;
    x_hat(:,k) = x_new;
     
     
    % Neighbor Average
    temp_sum = 0;
    for iii = 1:num_N
        temp_sum = temp_sum + x_hat(Neighbors(iii),k);
    end
    
    % Averages/Error Update
    N_avg(k) = temp_sum/num_N;
    avg(k) = mean(x_hat(1:N,k));
    m(k) = max(x_hat(1:N,k));
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
        if x_hat(jerk_node,k)>1
            x_hat(jerk_node,k) = 1;
        end
        if x_hat(jerk_node,k)<0
            x_hat(jerk_node,k) = 0;
        end
    end
    

    
% Check converge
      if k > max_k
          converge = 1;
      end
     if abs(avg(k-1)-avg(k))<0.00001 && k > 10
         converge = 1;
     end
end


figure(1);
x_ax = 1:1:k;
lb = a*ones(1,length(x_ax));
ub = b*ones(1,length(x_ax));
goal = jerk_goal*ones(1,length(x_ax));
p = plot(x_ax, avg, x_ax, x_hat(jerk_node,:),x_ax, x_hat(test_node1,:),x_ax, x_hat(test_node2,:),x_ax, lb, 'k--',x_ax, ub, 'k--');
xlim([1 k+5]);
limyu = max(x_hat(jerk_node,:))+0.5;
limyl = min(min(x_hat(jerk_node,:))-0.25,0);
ylim([limyl limyu])
legend('Network Average','P Selfish Agent','Test Node 1','Test Node 2','Capping bounds')
set(p, 'LineWidth',1)
set(gca,'FontSize',20, 'LineWidth',1)
%title1 = strcat('P Manipulator with myopic vision, k_P: ',num2str(kp),', Capped = ',num2str(capped),', Vision = Global');
%title(title1)
xlabel('Time (k)')
ylabel('Aggregate Opinions')

figure(2);
x_ax = 1:1:k;
p = plot(x_ax, err(:,1), x_ax, err(:,2));
xlim([1 k+5]);
ylim([limyl limyu])
legend('Neighbor Avg Error','Global Avg Error')
set(p, 'LineWidth',3)
set(gca,'FontSize',40, 'LineWidth',3)
%title1 = strcat('P Manipulator with myopic vision, k_P: ',num2str(kp),', Capped = ',num2str(capped),', Vision = Global');
%title(title1)
xlabel('Time (k)')
ylabel('Error')