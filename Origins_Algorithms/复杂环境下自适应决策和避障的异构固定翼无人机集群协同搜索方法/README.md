# ��������
�˳���ʹ���ض������䣬�����ڽϵͰ汾�� MATLAB ��ʹ�á�����ʹ�ò������£�
1. �� MATLAB �д� UAV_Cooperative_Search �ļ��С�
2. ���ļ���run_demo.m��������demo_index=1�����нű�������ÿʮ���������߸���һ��ͼ�е���������������������4��ͬ�����˻���
   ��ֹ�ϰ�������������ÿ�����г���ʱ������������ܶ���ͬ��
   [![img](./README.assets/UAV_Trajectory.png)]
3. ���ļ���run_demo.m��������demo_index=2�����нű������Կ���5���������˻��뾲ֹ�ϰ������������������˻��켣ͼ�У�
   ����ʹ��Rotate 3D���ߵ����ӽǡ�
   [![img](./README.assets/UAV_Trajectory2.png)]
4. ���ļ���run_demo.m��������demo_index=3�����нű������Կ���5���칹���˻��ڶ�̬�ϰ����µ���Ѱ�����������ɺ�
   �������³��򣬿����������˻�Эͬ��Ѱ���̵���Ƶ��
   ```
   F=Plot_UAV_Trajectory_Dynamic(map,GS,UAV_Coordinate,TAR,OBS);
   h=figure;
   set(gcf,'unit','inches','position',[0,0,14,6]);
   movie(h,F,1,10);
   v=VideoWriter('Trajectory.mp4','MPEG-4');
   v.FrameRate=10;
   v.Quality=100;
   open(v);
   writeVideo(v,F);
   close(v);
   ```
����������δ�����ܻ�������±������ǰѶ��������������Ƶ�Ĵ�����Բ�����ᡣ
����ʹ�� VideoWriter/writeVideo (line 410)
д���ļ� ��Ҫ�ȳ�ʼ���˶���Ȼ�����ִ������Ĳ����� ʱ�����������