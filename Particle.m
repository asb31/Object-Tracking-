Z=importdata('particle.txt');
Truepos=Z(:,1)
Mupdate(3,100)=0;                               %particle's with respective state and weights.
Mupdate(3,:)=0.01;                               % initializing values
Mupdate(2,:)=rand(1,100);
Mupdate(1,:)=rand(1,100);
pos(100,1109)=0;
pos(:,:)=0;                               % Store previous state weight and velocity
Moldpos(1:100)=rand(1,100);
Moldvelo(1:100)=rand(1,100);
Moldweight(1:100)=0.01;
Wnorm(100,1109)=0;
Mot=Z(:,3); 
Sout(1109)=0;                                    %Store state / o/p at each iteration
T=1;

for s=1:1109
%Step1: Particle propogation through state transition%
  for i=1:100
     Mupdate(1,i)= Moldpos(1,i)+T*Moldvelo(1,i);    % state position update 
  %state velocity update begin
     if Moldpos(1,i) < -20                      
        Mupdate(2,i)=2;
     end   
     if  Moldpos(1,i)>= -20 && Moldpos(1,i) < 0
         Mupdate(2,i)=Moldvelo(1,i) + abs(normrnd(-0.003906,0.003906));
     end
     if  Moldpos(1,i)>= 0 && Moldpos(1,i) <= 20
         Mupdate(2,i)= Moldvelo(1,i) - abs(normrnd(-0.003906,0.003906));
     end
     if  Moldpos(1,i) > 20 
         Mupdate(2,i)= -2;
     end
  % state velocity update end  
     Moldvelo(1,i)=Mupdate(2,i);
     Moldpos(1,i)=Mupdate(1,i);
     pos(i,s)=Mupdate(1,i);

%Step 2:  Weight update for each particle%
     Zideal=0.0998*exp((-(Mupdate(1,i)+10)^2)/32)+ 0.0998*exp((-(Mupdate(1,i)-10)^2)/32);
     Pyxt=102.1617*exp(-((Zideal-Z(s,3))^2)/(2*1.5257e-05) );
     Mupdate(3,i)=Moldweight(1,i)*Pyxt;
     Moldweight(1,i)=Mupdate(3,i);
    
%Weight update ends%
  end
  
  
   
 %Step 3  Weight Normalization for a instance
  norm=sum(Mupdate(3,:));
  for j=1:100                                                     % j represents number of particle
     
    Mupdate(3,j)=Mupdate(3,j)/norm;      
    %Prd= Mupdate(3,j)*Mupdate(1,j) + Prd;
    Moldweight(1,j)=Mupdate(3,j);
    Wnorm(j,s)=Mupdate(3,j);
  end
 %  Step 3 Weight normalization end 
 
 % Step 4 Desired Output Computation at a instance
  Prd=0;  
  for k=1:100
      Prd= Mupdate(3,k)*Mupdate(1,k) + Prd;
      Sout(s)= Prd;                                                  %  s is measurement
  end
    
  % Step 4 Computation End
  %Step 5: Resampling criteria begin%
  CV=0;  
  for v=1:100
     CV=CV+(500*Mupdate(3,v)-1)^2;
  end
  CV=CV*0.002;
  ESS=100/(1+CV);
  if ESS<50
      
       Q=cumsum(Mupdate(3,:));
       t=rand(1,101);
       T1=sort(t);
       T1(101)=1;
       i=1;
       j=1;
       while (i<=100)
           if T1(i) <Q(j)
               Index(i)=j;
               i=i+1;
           else
               j=j+1;
           end
       end
       s
       for i=1:100
           Mupdate(1,i)=Moldpos(Index(i));
           pos(i,s)=Mupdate(1,i);
           Mupdate(3,i)=0.01;
           Wnorm(i,s)=Mupdate(3,i);
           Moldpos(i)= Mupdate(1,i);
           Moldweight(i)= Mupdate(3,i);
       end
      
  end    
        
%Resampling step ends
end
% Plot Part 1
plot(Mot(1:1109),'color','Black')
hold on
plot(Truepos,'color','Red')
plot(Sout,'color','Blue')
legend( "Sensor Measurement","True Position","Estimated Position" )
xlabel("Iterations(Measurement)")
ylabel("Position")
saveas(gcf,'particle5','epsc')
%part 1 end


%part  2
%B(1:500)=-0.3937
%A=500*(Wnorm(:,70))
%plot(pos(:,70))
%xlim([0 500])
%hold on
%xlabel("Particles at iteration '70'")
%ylabel("Position & Weight")
%plot(A)
%plot(B)
%legend("State Position","Weight","True Position" )
%saveas(gcf,'particle2A','epsc')
% Part 2 end


%part  3
%B(1:100)=-0.0231;
%A=100*(Wnorm(:,679));

%C=200*(Wnorm(:,681));

%xlim([0 100]);
%hold on
%scatter(1:100,pos(:,679));
%xlabel("Particles" );
%ylabel("Weights");
%plot(B);
%plot(C);

%legend("Weights before resampling(itertion 679)","weights after resampling(iteration 681)" );
%saveas(gcf,'particle3A','epsc');
% Part 3 end;









