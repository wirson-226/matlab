function [in,out]=data_process(data,num,mem)
% ����1��num��Ϊ���� ��num��mem��Ϊ���
n=length(data)-num-mem+1;
for i=1:n
    in(i,:)=data(i:i+num-1);
    out(i,:)=data(i+num:i+num+mem-1);
end