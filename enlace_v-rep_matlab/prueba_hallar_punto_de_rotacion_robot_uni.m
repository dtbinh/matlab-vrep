vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
            Kp =1.3;
            Ki = 0.01;
            Kd = 0.001;
            L=0.47  
            R=0.15;
            b1=0;
            b=0;
            % errors
            E_k = 0;
            e_k_1 = 0;
            v=2;
            dt=0.1;

 if (clientID>-1)
      disp('Conected');

      [returnCode,Motor_izquierdo]=vrep.simxGetObjectHandle(clientID,'motor_1',vrep.simx_opmode_blocking);
      [returnCode,Motor_izquierdo_2]=vrep.simxGetObjectHandle(clientID,'motor_11',vrep.simx_opmode_blocking);
      
      
      %pause(5);
      %[returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo,0,vrep.simx_opmode_blocking)
       
      [returnCode,Motor_derecho]=vrep.simxGetObjectHandle(clientID,'Motor_3',vrep.simx_opmode_blocking);
      [returnCode,Motor_derecho_2]=vrep.simxGetObjectHandle(clientID,'motor_22',vrep.simx_opmode_blocking);


      %pause(5);
      %[returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho,0,vrep.simx_opmode_blocking)
      %[returnCode,position1]=vrep.simxGetObjectPosition(clientID,Cuerpo,-1,vrep.simx_opmode_oneshot)
      %Code here
      [returnCode,cuerpo]=vrep.simxGetObjectHandle(clientID,'Cuerpo',vrep.simx_opmode_blocking);
      x=-1;
      y=1;
      x0=-1.922;
      y0=-0.5446;
      x1=x0-x;
      y1=y0+y;
      theta=atan2(x,y);
      a=1;
      i=0;
      w=theta;
      while (a==1)
      [returnCode,position1]=vrep.simxGetObjectPosition(clientID,cuerpo,-1,vrep.simx_opmode_blocking);
       x_g=position1(:,1);
       i=i+1;
       x_p(i)=x_g;
       y_g=position1(:,2);
       y_p(i)=y_g;
%        plot(x,y,'b--o');
%        hold on
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo,0,vrep.simx_opmode_blocking);
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo_2,0,vrep.simx_opmode_blocking);
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho,-0.8,vrep.simx_opmode_blocking);
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho_2,-0.8,vrep.simx_opmode_blocking);
     if(x_g>x1*0.8 && x_g<x1*1.2 )
         b=1;
     end
     if(y_g<y1*0.8 && y_g>y1*1.2)
         b1=2;
     end
     
     if(b==1 && b1==2)
         a=2;
     end
      pause(0.01);
      end
      E_k = 0;
      e_k_1 = 0;
      v=0
while(a==2)
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo,0,vrep.simx_opmode_oneshot);
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho,0,vrep.simx_opmode_oneshot); 
end
    vrep.simxFinish(-1);
  end
  
    vrep.delete();