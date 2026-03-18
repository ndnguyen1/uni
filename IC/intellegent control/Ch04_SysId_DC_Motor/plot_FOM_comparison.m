%% plot comparison
function plot_FOM_comparison(data,data_verif,ts)
plot_font = 18;
N = length(data);
if (length(data_verif)<N)
    N = length(data_verif);
end
time = 1:N;
time = (time-1)*ts;

subplot(411)
plot(time,data(1:N,1),'LineWidth',1.5)
grid
%title('LS-Based SysId','FontSize',plot_font)
ylabel('$v_m(t)~(v)$','Interpreter','latex','FontSize',plot_font)
subplot(412)
plot(time,data(1:N,2),'b',time,data_verif(1:N,2),'r')
ylabel('$\theta(t)~(^o)$','Interpreter','latex','FontSize',plot_font)
grid
subplot(413)
plot(time,data(1:N,3),'b',time,data_verif(1:N,3),'r')
ylabel('$\omega(t)~(rad/s)$','Interpreter','latex','FontSize',plot_font)
grid
subplot(414)
plot(time,data(1:N,4),'b',time,data_verif(1:N,4),'r')
ylabel('$i(t)~(A)$','Interpreter','latex','FontSize',plot_font)
grid
xlabel('t (s)','Interpreter','latex','FontSize',plot_font)