%% LSTMʱ������Ԥ��
clc;clear;close all
%%
load fuhe 
n=10;m=2;
[x,y]=data_process(unnamed2,n,m);%ǰn��ʱ�� Ԥ���m��ʱ��

method=@mapminmax;%��һ��
%method=@mapstd;%��׼��
[xs,mappingx]=method(x');x=xs';
[ys,mappingy]=method(y');y=ys';

%��������
n=size(x,1);
m=round(n*0.7);%ǰ70%ѵ�� ��30%����
XTrain=x(1:m,:)';
XTest=x(m+1:end,:)';
YTrain=y(1:m,:)';
YTest=y(m+1:end,:)';
%% ��������
train=1;%Ϊ1������ѵ�����������ѵ���õ�ģ�ͽ���Ԥ��
if train==1
    rng(0)
    numFeatures = size(XTrain,1);%����ڵ���
    numResponses = size(YTrain,1);%����ڵ���
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
% ����һ��
predict_value=method('reverse',YPred,mappingy);
true_value=method('reverse',YTest,mappingy);
save result/lstm predict_value true_value
%%
load result/lstm
disp('�������')
result(true_value,predict_value,'LSTM')

%
figure
plot(true_value(end,:))
hold on
plot(predict_value(end,:))
grid on
title('LSTM')
legend('��ʵֵ','Ԥ��ֵ')
xlabel('Ԥ������������')
ylabel('ֵ')

ae= abs(predict_value - true_value);
rmse = (mean(ae.^2)).^0.5;
mse = mean(ae.^2);
mae = mean(ae);
mape = mean(ae./true_value);
R2 = 1 - (sum((predict_value- true_value).^2) / sum((true_value - mean(true_value)).^2))
Smape = mean(abs(predict_value - true_value)./((true_value+predict_value)./2));
disp('����Ԥ��������ָ�꣺')
disp(['RMSE = ', num2str(rmse)])
disp(['MSE  = ', num2str(mse)])
disp(['MAE  = ', num2str(mae)])
disp(['MAPE = ', num2str(mape)])
disp(['SMAPE = ', num2str(Smape)])
disp(['iwoa-R2 = ' num2str(R2)])