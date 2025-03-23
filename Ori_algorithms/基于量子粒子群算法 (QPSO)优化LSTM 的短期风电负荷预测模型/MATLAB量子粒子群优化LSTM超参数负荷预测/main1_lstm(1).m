%% LSTM时间序列预测
clc;clear;close all
%%
load fuhe 
n=10;m=2;
[x,y]=data_process(unnamed2,n,m);%前n个时刻 预测后m个时刻

method=@mapminmax;%归一化
%method=@mapstd;%标准化
[xs,mappingx]=method(x');x=xs';
[ys,mappingy]=method(y');y=ys';

%划分数据
n=size(x,1);
m=round(n*0.7);%前70%训练 后30%测试
XTrain=x(1:m,:)';
XTest=x(m+1:end,:)';
YTrain=y(1:m,:)';
YTest=y(m+1:end,:)';
%% 参数设置
train=1;%为1就重新训练，否则加载训练好的模型进行预测
if train==1
    rng(0)
    numFeatures = size(XTrain,1);%输入节点数
    numResponses = size(YTrain,1);%输出节点数
    miniBatchSize = 16; %batchsize
    numHiddenUnits1 = 20;
    numHiddenUnits2 = 20;
    maxEpochs=20;
    learning_rate=0.005;
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
    save model/lstm net
else
    load model/lstm
end
YPred = predict(net,XTest,'MiniBatchSize',1);YPred=double(YPred);
% 反归一化
predict_value=method('reverse',YPred,mappingy);
true_value=method('reverse',YTest,mappingy);
save result/lstm predict_value true_value
%%
load result/lstm
disp('结果分析')
result(true_value,predict_value,'LSTM')

%
figure
plot(true_value(end,:))
hold on
plot(predict_value(end,:))
grid on
title('LSTM')
legend('真实值','预测值')
xlabel('预测样本点坐标')
ylabel('值')

ae= abs(predict_value - true_value);
rmse = (mean(ae.^2)).^0.5;
mse = mean(ae.^2);
mae = mean(ae);
mape = mean(ae./true_value);
R2 = 1 - (sum((predict_value- true_value).^2) / sum((true_value - mean(true_value)).^2))
Smape = mean(abs(predict_value - true_value)./((true_value+predict_value)./2));
disp('机组预测结果评价指标：')
disp(['RMSE = ', num2str(rmse)])
disp(['MSE  = ', num2str(mse)])
disp(['MAE  = ', num2str(mae)])
disp(['MAPE = ', num2str(mape)])
disp(['SMAPE = ', num2str(Smape)])
disp(['iwoa-R2 = ' num2str(R2)])