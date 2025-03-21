%% PSO�Ż�LSTMʱ������Ԥ��
clc;clear;close all;format compact
%%
load fuhe
n=10;m=2;
[x,y]=data_process(unnamed2,n,m);%ǰn��ʱ�� Ԥ���m��ʱ��
method=@mapminmax;%��һ��
% method=@mapstd;%��׼��
[xs,mappingx]=method(x');x=xs';
[ys,mappingy]=method(y');y=ys';

%��������
n=size(x,1);
m=round(n*0.7);%ǰ70%ѵ�� ��30%����
XTrain=x(1:m,:)';
XTest=x(m+1:end,:)';
YTrain=y(1:m,:)';
YTest=y(m+1:end,:)';
%% ����PSO�Ż�
optimization=1;%�Ƿ������Ż�
if optimization==1
    [x ,fit_gen,process]=psoforlstm(XTrain,YTrain,XTest,YTest);%�ֱ��������ڵ� ѵ��������ѧϰ��Ѱ��
    save result/pso_para_result x fit_gen process
else
    load result/pso_para_result
end
%% ����Ӧ��������4�������ı仯����
huatu(fit_gen,process,'PSO')
disp('�Ż��ĳ�����Ϊ��')
disp('L1:'),x(1)
disp('L2:'),x(2)
disp('K:'),x(3)
disp('lr:'),x(4)

%% �����Ż��õ��Ĳ�������ѵ��
train=1;%�Ƿ�����ѵ��
if train==1
    rng(0)
    numFeatures = size(XTrain,1);%����ڵ���
    numResponses = size(YTrain,1);%����ڵ���
    miniBatchSize = 16; %batchsize ��fitness.m�б���һ��
    numHiddenUnits1 = x(1);
    numHiddenUnits2 = x(2);
    maxEpochs=x(3);
    learning_rate=x(4);
    layers = [ ...
        sequenceInputLayer(numFeatures)
        lstmLayer(numHiddenUnits1)
        lstmLayer(numHiddenUnits2)
        fullyConnectedLayer(numResponses)
        regressionLayer];
    options = trainingOptions('adam', ...
        'MaxEpochs',maxEpochs, ...
        'MiniBatchSize',miniBatchSize, ...
        'InitialLearnRate',learning_rate, ...
        'GradientThreshold',1, ...
        'Shuffle','every-epoch', ...
        'Verbose',true,...
        'Plots','training-progress');

    net = trainNetwork(XTrain,YTrain,layers,options);
    save model/psolstm net
else
    load model/psolstm
end
% Ԥ��
YPred = predict(net,XTest,'MiniBatchSize',1);YPred=double(YPred);
% ����һ��
predict_value=method('reverse',YPred,mappingy);
true_value=method('reverse',YTest,mappingy);
save result/pso_lstm predict_value true_value
%%
load result/pso_lstm
disp('�������')
result(true_value,predict_value,'PSO-LSTM')

fprintf('\n')

%
figure
plot(true_value(end,:))
hold on
plot(predict_value(end,:))
grid on
title('PSO-LSTM')
legend('��ʵֵ','Ԥ��ֵ')
xlabel('Ԥ������������')
ylabel('ֵ')


