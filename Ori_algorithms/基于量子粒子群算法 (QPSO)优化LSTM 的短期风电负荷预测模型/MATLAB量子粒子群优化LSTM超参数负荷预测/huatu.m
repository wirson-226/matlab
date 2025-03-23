function huatu(fitness,process,type)
figure
plot(fitness)
grid on
title([type,'的适应度曲线'])
xlabel('迭代次数/次')
ylabel('适应度值/MSE')

figure
subplot(2,2,1)
plot(process(:,1))
grid on
xlabel('迭代次数/次')
ylabel('L1/个')

subplot(2,2,2)
plot(process(:,2))
grid on
xlabel('迭代次数/次')
ylabel('L2/个')

subplot(2,2,3)
plot(process(:,3))
grid on
xlabel('迭代次数/次')
ylabel('K/次')

subplot(2,2,4)
plot(process(:,4))
grid on
xlabel('迭代次数/次')
ylabel('lr')
title([type,'的超参数随迭代次数的变化'])