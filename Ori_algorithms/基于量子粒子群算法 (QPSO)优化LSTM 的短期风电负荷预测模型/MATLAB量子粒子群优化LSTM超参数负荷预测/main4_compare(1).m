clc;clear;close
%%
lstm=load('result/lstm.mat');
result(lstm.true_value,lstm.predict_value,'LSTM');

psolstm=load('result/pso_lstm.mat');
result(psolstm.true_value,psolstm.predict_value,'PSO-LSTM');

qpsolstm=load('result/qpso_lstm.mat');
result(qpsolstm.true_value,qpsolstm.predict_value,'QPSO-LSTM');

figure
plot(lstm.true_value(end,:),'-r')
hold on;grid on
plot(lstm.predict_value(end,:),'-b')
plot(psolstm.predict_value(end,:),'-k')
plot(qpsolstm.predict_value(end,:),'-','color',[4 157 107]/255)
legend('��ʵֵ','LSTMԤ��','PSO-LSTMԤ��','QPSO-LSTMԤ��')
title('���㷨���')
xlabel('���Լ�����')
ylabel('����')
