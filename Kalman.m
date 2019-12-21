Z1=importdata('data.txt');       % sensor data
z=Z1(1,1);
x=0;
xd=0;
I=[1 0;0 1];
Xp=[x;xd];
Xm=[x;xd];
Phi=[1 1;0 1];
Sm=[0.1 0;0 0.01];
M=[1 0];
Q=[0 0;0 0.00001];
R=1;
A(1:639)=0
A(1)=8

for (i=2:639)
  %Predict Equations
   Xp=Phi*Xm;
   Sp=Phi*Sm*Phi' + Q;
  % Correction Equations
   KG=Sp*M'*inv(M*Sp*M'+R);
   Xm=Xp+KG*(Z1(i,1)-M*Xp);
   Sm=(I-KG*M)*Sp;
   A(i)=Xm(1)
   xdot=Z1(i,1)-Z1(i-1,1)
   x=Xm(1,1)
end
hold on
plot(Z1)
plot(A')
xlabel('Sampling Time')
ylabel('Position')
%ylim([-3 2.5])
%xlim([0 70])
legend( "Measured Position","Estimated Position for Q/R=0.00001")
saveas(gcf,'Fig5','epsc')
hold off

