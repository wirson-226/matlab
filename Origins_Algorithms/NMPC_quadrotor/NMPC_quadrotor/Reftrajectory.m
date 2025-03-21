yref = QuadrotorReferenceTrajectory(0:0.1:20);
subplot(1,2,1)
plot3(yref(1,:),yref(2,:),yref(3,:),'g.');
xlabel('x')
ylabel('y')
zlabel('z')
subplot(1,2,2)
plot3(yref(1,:),yref(2,:),yref(3,:),'g.');
xlabel('x')
ylabel('y')
zlabel('z')
