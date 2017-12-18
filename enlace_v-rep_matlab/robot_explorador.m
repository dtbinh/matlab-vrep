vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
            Kp =1.02;
            Ki = 0.01;
            Kd = 0.03;
            L=0.6;  
            R=0.12;
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
      x0=-0.9362;
      y0=-1.8215;
      x1=x0-x;
      y1=y0+y;
      theta=atan2(x,y);
      a=1;
      i=0;
      w=theta;
      w1=w;
      while (a==1)
      [returnCode,position1]=vrep.simxGetObjectPosition(clientID,cuerpo,-1,vrep.simx_opmode_blocking);
       x_g=position1(:,1);
         i=i+1;
         x_p(i)=x_g;
% %     x_gg(i)=x_g;
       y_g=position1(:,2);
       y_p(i)=y_g;
%       u_x = (x_g-x);   
%       u_y = (y_g-y);
%     theta_g = atan2(u_y,u_x);  
      e_k = w;
      e_k_aa(i)=e_k;
      eeee(i)=e_k;
      e_k = atan2(sin(e_k),cos(e_k));
      e_P = e_k;    
      e_I = E_k + e_k*dt;
      E_k=e_k;
      e_D = (e_k-e_k_1)/dt;    
      w = Kp*e_P+Ki*e_I+Kd*e_D;
      ww(i)=w;
     e_k_1 = e_k; 
     vi=(2*v+w*L)/(R*2);
     vii=-vi/10;
     vd=(2*v-w*L)/(R*2);
     vdd=-vd/10;
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo,vii,vrep.simx_opmode_blocking);
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo_2,vii,vrep.simx_opmode_blocking);
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho,vdd,vrep.simx_opmode_blocking);
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho_2,vdd,vrep.simx_opmode_blocking);
     if(x_g>x1*0.8 && x_g<x1*1.2 )
         b=1;
     end
     if(y_g<y1*0.8 && y_g>y1*1.2)
         b1=2;
     end
     
     if(b==1 && b1==2)
         a=2;
     end
      pause(0.1);
      end
      E_k = 0;
      e_k_1 = 0;
      v=0
while(a==2)
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo,0,vrep.simx_opmode_oneshot);
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho_2,0,vrep.simx_opmode_oneshot); 
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo_2,0,vrep.simx_opmode_oneshot);
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho,0,vrep.simx_opmode_oneshot); 
end
    vrep.simxFinish(-1);
  end
  
    vrep.delete();