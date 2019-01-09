classdef controller < handle
    %  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
    %  Created on: 2018-10-05
    %  Copyright (c)  2018, The Chinese University of Hong Kong
    %  This software is provided "as is" under BSD License, with
    %  no warranty. The complete license can be found in LICENSE

    properties(Access = public)
        pub_tor
        sub_pos
        dynamic_param_pos
        dynamic_param_neg
        safe_upper_torque_limit
        safe_lower_torque_limit
        beta_vel_amplitude
        g
        Zero_Output_Joint_No
        mtm_arm
        msg_counter_buffer = 0
        is_disp_init_info = false
        drift_test_safe_vel_limit
        is_drift_vel_exceed_limit
        ARM_NAME
    end

    methods(Access = public)
        % Class constructor
        function obj = controller(mtm_arm,...
                dynamic_params_pos,...
                dynamic_params_neg,...
                safe_upper_torque_limit,...
                safe_lower_torque_limit,...
                beta_vel_amplitude,...
                g,...
                ARM_NAME)
            obj.safe_upper_torque_limit = safe_upper_torque_limit;
            obj.safe_lower_torque_limit = safe_lower_torque_limit;
            obj.beta_vel_amplitude = beta_vel_amplitude;
            obj.g = g;
            obj.mtm_arm = mtm_arm;
            obj.ARM_NAME = ARM_NAME;
            obj.dynamic_param_pos = dynamic_params_pos;
            obj.dynamic_param_neg = dynamic_params_neg;
            disp('Controller dynamic parameters of MTM: [param]');
            for i=1:size(obj.dynamic_param_pos,1)
                fprintf('Param_%d: [%0.5f], [%0.5f]\n', i, obj.dynamic_param_pos(i), obj.dynamic_param_neg(i));
            end
            obj.pub_tor = rospublisher(['/dvrk/',ARM_NAME,'/set_effort_joint']);
            obj.sub_pos = rossubscriber(['/dvrk/',ARM_NAME,'/state_joint_current']);
        end

        % Callback function of pose subscriber when start gc controller
        function callback_gc_publisher(obj, q, q_dot)
            if(~obj.is_disp_init_info)
                fprintf('GC of %s starts, you can move %s now. If you need to stop gc controller, call "mtm_gc_controller.stop_gc()".\n',obj.ARM_NAME,obj.ARM_NAME);
                obj.is_disp_init_info = true;
            end

            if(obj.msg_counter_buffer==0)
                fprintf('.');
            end

            if(obj.msg_counter_buffer == 100)
                obj.msg_counter_buffer = 0;
            else
                obj.msg_counter_buffer = obj.msg_counter_buffer+1;
            end
            % Calculate predict torques
            Torques = obj.base_controller(q, q_dot);

            % Publish predict torques
            msg = rosmessage(obj.pub_tor);
            for i =1:7
                msg.Effort(i) = Torques(i);
            end
            send(obj.pub_tor, msg);
        end
        
                % Callback function of pose subscriber when start gc controller
        function callback_gc_pub_with_vel_safestop(obj, q, q_dot)
            for i=1:7
                if(abs(q_dot(i))>obj.drift_test_safe_vel_limit(i))
                    obj.is_drift_vel_exceed_limit = true;
                end
            end
            
            if obj.is_drift_vel_exceed_limit 
                msg = rosmessage(obj.pub_tor);
                for i =1:7
                    msg.Effort(i) = 0.0;
                end
            else
                % Calculate predict torques
                Torques = obj.base_controller(q, q_dot);
                % Publish predict torques
                msg = rosmessage(obj.pub_tor);
                for i =1:7
                    msg.Effort(i) = Torques(i);
                end
                % for testing
%                 msg.Effort(5) = 0.05;
            end
            send(obj.pub_tor, msg);
        end

        % Base controller to calculate the predict torque
        function Torques = base_controller(obj, q, q_dot)
            vel = q_dot;
            Regressor_Matrix =analytical_regressor_mat(obj.g,q);
            Torques_pos = Regressor_Matrix*obj.dynamic_param_pos;
            Torques_neg = Regressor_Matrix*obj.dynamic_param_neg;
            Torques = zeros(7,1);
            for i =1:6
                % Fusing predicted torques with positive and negative direction
                alpha = obj.sin_vel(vel(i),obj.beta_vel_amplitude(i));
                Torques(i) = Torques_pos(i)*alpha+Torques_neg(i)*(1-alpha);

                % Set upper and lower torque limit, if output value exceed limits, just keep the limit value for output
                if Torques(i)>=obj.safe_upper_torque_limit(i)
                    Torques(i)=obj.safe_upper_torque_limit(i);
                elseif Torques(i)<=obj.safe_lower_torque_limit(i)
                    Torques(i)=obj.safe_lower_torque_limit(i);
                end
            end
            Torques(obj.Zero_Output_Joint_No) = 0;
        end

        % Beta function for fusing torques
        function sign_vel = sin_vel(obj, joint_vel, amplitude)
            if joint_vel >= abs(amplitude)
                sign_vel = 1;
            elseif joint_vel <= -abs(amplitude)
                sign_vel = 0;
            else
                sign_vel = 0.5+sin(pi*(joint_vel/amplitude)/2)/2;
            end
        end


        % call this function to start the gc controller
        function start_gc(obj)
            % Apply GC controllers
            callback_MTM = @(src,msg)(obj.callback_gc_publisher(msg.Position,...
                msg.Velocity));
            obj.sub_pos = rossubscriber(['/dvrk/',obj.ARM_NAME,'/state_joint_current'],callback_MTM,'BufferSize',10);
        end

        % call this function to stop the gc controller and move to origin pose
        function stop_gc(obj)
            obj.sub_pos = rossubscriber(['/dvrk/',obj.ARM_NAME,'/state_joint_current']);
            obj.mtm_arm.move_joint([0,0,0,0,0,0,0]);
            disp('gc_controller stopped');
        end
        
        % call this function to start the gc controller
        function start_gc_with_vel_safestop(obj, safe_vel_limit)
            % Apply GC controllers
            obj.drift_test_safe_vel_limit = safe_vel_limit;
            obj.is_drift_vel_exceed_limit = false;
            callback_MTM = @(src,msg)(obj.callback_gc_pub_with_vel_safestop(msg.Position,...
                                                                            msg.Velocity));
            obj.sub_pos = rossubscriber(['/dvrk/',obj.ARM_NAME,'/state_joint_current'],callback_MTM,'BufferSize',10);
        end
        
        function set_zero_tor_output_joint(obj, Zero_Output_Joint_No)
            obj.Zero_Output_Joint_No = Zero_Output_Joint_No
        end
    end
end

