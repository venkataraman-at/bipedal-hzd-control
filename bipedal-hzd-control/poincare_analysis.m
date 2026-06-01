function poincare_analysis(x_plus0, a, s_params, n_steps)
% Poincaré analysis for 3-link biped full dynamics

q1_min = s_params(1);
q1_max = s_params(2);
options = odeset('Events',@event_fn,'AbsTol',1e-6,'RelTol',1e-6);

%---- Record Poincaré section ----
z_poincare = zeros(n_steps, 2);
x_ic = x_plus0;
for i = 1:n_steps
    [~, x_sol] = ode45(@(t,x) func_full_dynamics(t,x,a,s_params),[0,5],x_ic,options);
    if isempty(x_sol); break; end
    z_poincare(i,:) = x_sol(end,[1,4]);
    [x_plus_i,~] = func_impact_map(x_sol(end,:));
    x_ic = x_plus_i(end,:);
end

%---- Plot Poincaré section ----
figure
plot(z_poincare(:,1),z_poincare(:,2),'bo-','LineWidth',1.5,'MarkerSize',8); hold on
plot(z_poincare(1,1),z_poincare(1,2),'g*','MarkerSize',14,'LineWidth',2)
plot(z_poincare(end,1),z_poincare(end,2),'r*','MarkerSize',14,'LineWidth',2)
xlabel('q_1 (rads)'); ylabel('dq_1 (rads/s)')
title('Poincaré Section: q_1 vs dq_1 at Impact')
legend('Steps','Start','End'); grid on

%---- Numerical Poincaré map Jacobian (2x2) ----
eps_p = 1e-5;
A_p = zeros(2,2);
[~,x_nom] = ode45(@(t,x) func_full_dynamics(t,x,a,s_params),[0,5],x_plus0,options);
[x_n_plus,~] = func_impact_map(x_nom(end,:));
z_nom_next = x_n_plus(end,[1,4]);

state_idx = [1,4];
for j = 1:2
    x_pert = x_plus0;
    x_pert(state_idx(j)) = x_pert(state_idx(j)) + eps_p;
    [~,x_p] = ode45(@(t,x) func_full_dynamics(t,x,a,s_params),[0,5],x_pert,options);
    [x_p_plus,~] = func_impact_map(x_p(end,:));
    z_p_next = x_p_plus(end,[1,4]);
    A_p(:,j) = (z_p_next - z_nom_next)'/eps_p;
end

eigs_Ap = eig(A_p);
fprintf('\n=== Poincare Map Analysis ===\n');
for i = 1:length(eigs_Ap)
    fprintf('  lambda_%d = %.4f + %.4fi  (|lambda| = %.4f)\n',...
        i,real(eigs_Ap(i)),imag(eigs_Ap(i)),abs(eigs_Ap(i)));
end
if all(abs(eigs_Ap) < 1)
    fprintf('System is STABLE: all eigenvalues inside unit circle\n');
else
    fprintf('System is UNSTABLE: eigenvalue(s) outside unit circle\n');
end

%---- Eigenvalue plot ----
figure
theta_c = linspace(0,2*pi,200);
plot(cos(theta_c),sin(theta_c),'k--','LineWidth',1.5); hold on
plot(real(eigs_Ap),imag(eigs_Ap),'rx','MarkerSize',15,'LineWidth',3)
xlabel('Real Part'); ylabel('Imaginary Part')
title('Poincare Map Eigenvalues')
legend('Unit Circle','Eigenvalues')
axis equal; grid on
lim_v = max(2, max(abs(eigs_Ap))*1.5);
xlim([-lim_v,lim_v]); ylim([-lim_v,lim_v])

%---- Event function ----
function [lim,isterminal,dir] = event_fn(~,x)
    s = func_gait_timing(x(1),q1_min,q1_max);
    if s>=1; s=1; end
    lim = s-1; isterminal=1; dir=[];
end

end