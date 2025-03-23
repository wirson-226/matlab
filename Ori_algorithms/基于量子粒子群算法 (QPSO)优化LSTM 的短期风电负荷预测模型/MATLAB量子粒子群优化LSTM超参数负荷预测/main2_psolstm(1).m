%% PSO优化LSTM时间序列预测
clc;clear;close all;format compact
%%
load fuhe
n=10;m=2;
[x,y]=data_process(unnamed2,n,m);%前n个时刻 预测后m个时刻
method=@mapminmax;%归一化
% method=@mapstd;%标准化
[xs,mappingx]=method(x');x=xs';
[ys,mappingy]=method(y');y=ys';

%划分数据
n=size(x,1);
m=round(n*0.7);%前70%训练 后30%测试
XTrain=x(1:m,:)';
XTest=x(m+1:end,:)';
YTrain=y(1:m,:)';
YTest=y(m+1:end,:)';
%% 采用PSO优化
optimization=1;%是否重新优化
if optimization==1
    [x ,fit_gen,process]=psoforlstm(XTrain,YTrain,XTest,YTest);%分别对隐含层节点 训练次数与学习率寻优
    save result/pso_para_result x fit_gen process
else
    load result/pso_para_result
end
%% 画适应度曲线与4个参数的变化曲线
huatu(fit_gen,process,'PSO')
disp('优化的超参数为：')
disp('L1:'),x(1)
disp('L2:'),x(2)
disp('K:'),x(3)
disp('lr:'),x(4)

%% 利用优化得到的参数重新训练
train=1;%是否重新训练
if train==1
    rng(0)
    numFeatures = size(XTrain,1);%输入节点数
    numResponses = size(YTrain,1);%输出节点数
    miniBatchSize = 16; %batchsize 与fitness.m中保持一致
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
% 预测
YPred = predict(net,XTest,'MiniBatchSize',1);YPred=double(YPred);
% 反归一化
predict_value=method('reverse',YPred,mappingy);
true_value=method('reverse',YTest,mappingy);
save result/pso_lstm predict_value true_value
%%
load result/pso_lstm
disp('结果分析')
result(true_value,predict_value,'PSO-LSTM')

fprintf('\n')

%
figure
plot(true_value(end,:))
hold on
plot(predict_value(end,:))
grid on
title('PSO-LSTM')
legend('真实值','预测值')
xlabel('预测样本点坐标')
ylabel('值')


