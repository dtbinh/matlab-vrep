vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
            Kp = 1.1;
            Ki = 0.01;
            Kd = 0.01;
            L=0.47  
            R=0.15;
            b1=0;
            b=0;
            % errors
            E_k = 0;
            e_k_1 = 0;
            x_g=0;
            y_g=0;
            v=0.01;
            dt=0.5
            theta=0;
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
      x=-2;
      y=-2;
      theta=atan2(x,y);
      w=theta;
      a=1;
      while (a==1)
      [returnCode,position1]=vrep.simxGetObjectPosition(clientID,cuerpo,-1,vrep.simx_opmode_blocking);
      x_g=position1(:,1);
%     i=i+1;
%     x_gg(i)=x_g;
      y_g=position1(:,2);
%       u_x = (x_g-x);   
%       u_y = (y_g-y);
%       theta_g = atan2(u_y,u_x);  
      e_k = w;
      %e1=e_k;
      e_k = atan2(sin(e_k),cos(e_k));
      e_P = e_k;    
      e_I = E_k + e_k*dt;
      e_D = (e_k-e_k_1)/dt;    
      w = Kp*e_P+Ki*e_I+Kd*e_D;
      E_k = e_I;
      e_k_1 = e_k; 
      vi=(2*v+w*L)/(R*2);
      vi=-vi;
      vd=(2*v-w*L)/(R*2);
      vd=-vd;
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo,vi,vrep.simx_opmode_blocking);
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo_2,vi,vrep.simx_opmode_blocking);
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho,vd,vrep.simx_opmode_blocking);
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho_2,vd,vrep.simx_opmode_blocking);
     if(x_g>1.4 && x_g<1.2)
         b=1;
     end
     if(y_g>-0.5 && y_g<-0.4)
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
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo_2,0,vrep.simx_opmode_oneshot);
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho,0,vrep.simx_opmode_oneshot);
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho_2,0,vrep.simx_opmode_oneshot);
end
    vrep.simxFinish(-1);
  end
  
    vrep.delete();