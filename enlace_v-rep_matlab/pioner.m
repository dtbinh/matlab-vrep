vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
            % initialize memory banks
            Kp = 4;
            Ki = 0.01;
            Kd = 0.01;
                        
            % errors
            E_k = 0;
            e_k_1 = 0;
            x_g=0;
            y_g=0;
            v=0.1;
            w=0;
  
  if (clientID>-1)
      disp('Conected');

      [returnCode,Motor_izquierdo]=vrep.simxGetObjectHandle(clientID,'Motor_1',vrep.simx_opmode_blocking)
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo,0,vrep.simx_opmode_blocking)
      %pause(5);
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_izquierdo,0,vrep.simx_opmode_blocking)
       
       
      [returnCode,Motor_derecho]=vrep.simxGetObjectHandle(clientID,'Motor_3',vrep.simx_opmode_blocking)
      [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho,v,vrep.simx_opmode_blocking)
      %pause(5);
       [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor_derecho,v,vrep.simx_opmode_blocking)
      %Code here
           
            

      
        

        %% EXECUTE Computes the left and right wheel speeds for go-to-goal.
        %   [v, w] = execute(obj, robot, x_g, y_g, v) will compute the
        %   necessary linear and angular speeds that will steer the robot
        %   to the goal location (x_g, y_g) with a constant linear velocity
        %   of v.
        %
        %   See also controller/execute
        
            % Retrieve the (relative) goal location
            x_g = x_g; 
            y_g = y_g;
            
            % Get estimate of current pose
            x=1;
            y=1;
            theta=atan2(y,x);
            
            % Compute the v,w that will get you to the goal
            v = v;
            
            % 1. Calculate the heading (angle) to the goal.
            
            % distance between goal and robot in x-direction
            u_x = x_g-x;     
                
            % distance between goal and robot in y-direction
            u_y = y_g-y;
                
            % angle from robot to goal. Hint: use ATAN2, u_x, u_y here.
            theta_g = atan2(u_y,u_x);
            
            % 2. Calculate the heading error.
            
            % error between the goal angle and robot's angle
            % Hint: Use ATAN2 to make sure this stays in [-pi,pi].
            e_k = theta_g-theta;
            e_k = atan2(sin(e_k),cos(e_k));
            
                
            % 3. Calculate PID for the steering angle 
            
            % error for the proportional term
            e_P = e_k;
            
            % error for the integral term. Hint: Approximate the integral using
            % the accumulated error, obj.E_k, and the error for
            % this time step, e_k.
            e_I = obj.E_k + e_k*dt;
                     
            % error for the derivative term. Hint: Approximate the derivative
            % using the previous error, obj.e_k_1, and the
            % error for this time step, e_k.
            e_D = (e_k-obj.e_k_1)/dt;    
                  
            w = obj.Kp*e_P + obj.Ki*e_I + obj.Kd*e_D;
            
            % 4. Save errors for next time step
            obj.E_k = e_I;
            obj.e_k_1 = e_k; 
            vout= v;
            wout= w;
            % Reset accumulated and previous error
            obj.E_k = 0;
            obj.e_k_1 = 0;
       

       vrep.simxFinish(-1);
       
  end
  
    vrep.delete();