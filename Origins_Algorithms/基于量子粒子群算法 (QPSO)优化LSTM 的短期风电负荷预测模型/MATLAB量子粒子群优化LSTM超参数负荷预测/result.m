function result(true_value,predict_value,type)
[m,n]=size(true_value);
true_value=reshape(true_value,[1,m*n]);
predict_value=reshape(predict_value,[1,m*n]);
disp(type)
rmse=sqrt(mean((true_value-predict_value).^2));
disp(['��������(RMSE)��',num2str(rmse)])
mae=mean(abs(true_value-predict_value));
disp(['ƽ��������MAE����',num2str(mae)])
mape=mean(abs((true_value-predict_value)./true_value));
disp(['ƽ����԰ٷ���MAPE����',num2str(mape*100),'%'])
R2 = 1 - norm(true_value-predict_value)^2/norm(true_value - mean(true_value))^2;
disp(['����ϵ����R2����',num2str(R2)])


fprintf('\n')