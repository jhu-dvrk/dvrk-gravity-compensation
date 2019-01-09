function is_test_success =  gravity_compensation_test(config, test_mode)

    mtm_gc_controller = online_test_init(config);
    if strcmp(test_mode, 'ONLINE_GC_PRT_ERR')
        is_test_success = online_gc_predict_error_test(mtm_gc_controller, config);
    elseif strcmp(test_mode, 'ONLINE_GC_DRT')
        is_test_success = online_gc_drift_test(mtm_gc_controller, config);
    end
end

function mtm_gc_controller = online_test_init(config)
    % % Spawn GC Controllers and test
    param_num = 40; % hardcode
    gc_dynamic_params_pos = param_vec_checking(config.GC_controller.gc_dynamic_params_pos, param_num, 1);
    gc_dynamic_params_neg = param_vec_checking(config.GC_controller.gc_dynamic_params_neg, param_num, 1);
    mtm_arm = mtm(config.ARM_NAME);
    mtm_gc_controller= controller(mtm_arm,...
        gc_dynamic_params_pos,...
        gc_dynamic_params_neg,...
        config.GC_controller.safe_upper_torque_limit,...
        config.GC_controller.safe_lower_torque_limit,...
        config.GC_controller.beta_vel_amplitude,...
        config.lse.g_constant,...
        config.ARM_NAME);
end

function is_test_success = online_gc_predict_error_test(mtm_gc_controller, config)
    % Test Controller
    is_test_success =  true;
    fprintf('Testing GC controller for %s\n', config.ARM_NAME);
    fprintf('Calculating "error rate" between predicted and measured torques\n');
    pos_name_cell = fieldnames(config.GC_Test.ONLINE_GC_PRT_ERR.testing_sets);
    test_pos_mat = [];
    abs_err_mat_MTM = [];
    rel_err_mat_MTM = [];
    for k=1:size(pos_name_cell)
        test_pos_mat = [test_pos_mat,getfield(config.GC_Test.ONLINE_GC_PRT_ERR.testing_sets,pos_name_cell{k})];
        fprintf('Test joint position #%d, joint_pos=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n', k, test_pos_mat(:,end)');
        [abs_err, rel_err] = online_gc_predict_error_controller(mtm_gc_controller,...
                                                                deg2rad(test_pos_mat(:,end)),...
                                                                config.GC_Test.ONLINE_GC_PRT_ERR.divider_const_vec);
        abs_err_mat_MTM= [abs_err_mat_MTM,abs_err];
        rel_err_mat_MTM = [rel_err_mat_MTM,rel_err];
    end
    fprintf('====================\n');
    fprintf('Test Result for %s\n', config.ARM_NAME);
    fprintf('Test result: "absolute error"\n');
    for i=1:7
        fprintf('Joint#%d\t', i);
    end
    fprintf('\n');
    for j=1:size(pos_name_cell)
        for k = 1:7
            fprintf('%.4f   \t', abs_err_mat_MTM(k,j));
        end
        fprintf('\n');
    end
    
    fprintf('Test result: "relative error rate"\n');
    for i=1:7
        fprintf('Joint#%d\t', i);
    end
    fprintf('\n');
    for j=1:size(pos_name_cell)
 
        for k = 1:7
            fprintf('%d%%\t', int32(rel_err_mat_MTM(k,j)*100));
        end
        fprintf('\n');
    end
    
    for j=1:size(pos_name_cell)
        for k = 1:7
            if(int32(rel_err_mat_MTM(k,j)*100)>=config.GC_Test.ONLINE_GC_PRT_ERR.rel_err_threshold(k))
                warning('[Test Pos %d]: --%s Joint%d-- absolute torque error:[%.4f], error rate:[%d%%], has exceed the error rate threshold %d%%',...
                    j,config.ARM_NAME,k,...
                    abs_err_mat_MTM(k,j),...
                    int32(rel_err_mat_MTM(k,j)*100),...
                    config.GC_Test.ONLINE_GC_PRT_ERR.rel_err_threshold(k));
                is_test_success =  false;
            end
        end
    end
    % Move MTM to origin position
    mtm_gc_controller.mtm_arm.move_joint(deg2rad(zeros(1,7)));
end

% call this function to test the controller and calculate the absolute and relate error between predicted and measuring torques
function [abs_err, rel_err] = online_gc_predict_error_controller(mtm_gc_controller, q, divider_const_vec)
    % Moving to test pose and waiting for 3 seconds until MTM remains static
    mtm_gc_controller.mtm_arm.move_joint(q);
    pause(3);
    % Get joint position and velocity signal by ROS communication
    msg = receive(mtm_gc_controller.sub_pos);
    q = msg.Position;
    q_dot = msg.Velocity;
    tau_measured = msg.Effort;
    tau_computed = mtm_gc_controller.base_controller(q,q_dot);
    abs_err = abs(tau_measured-tau_computed);
    div_vec = abs(tau_measured)+divider_const_vec;
    rel_err = abs(abs_err./div_vec);
end

function is_test_success = online_gc_drift_test(mtm_gc_controller, config)
    disp('Started test estimating drift at a fixed position');
    mtm_gc_controller.mtm_arm.move_joint(deg2rad(zeros(1,7)));
    % pause(5.0); % to make sure PID is stable
    disp('Measuring drift...');
    mtm_gc_controller.start_gc_with_vel_safestop(config.GC_Test.ONLINE_GC_DRT.safe_vel_limit);
    rate = config.GC_Test.ONLINE_GC_DRT.rate;
    deta_time = 1/rate;
    duration = config.GC_Test.ONLINE_GC_DRT.duration;
    pos_mat = [];
    vel_mat = [];
    count = 0;
    while(~mtm_gc_controller.is_drift_vel_exceed_limit &&  count<=duration*rate)
        msg = receive(mtm_gc_controller.sub_pos);
        pos_mat = cat(2, pos_mat, msg.Position);
        vel_mat = cat(2, vel_mat, msg.Velocity);
        pause(deta_time);
        count = count +1;
    end
    if(~mtm_gc_controller.is_drift_vel_exceed_limit)
        mtm_gc_controller.stop_gc();
        is_test_success = true;
    else
        for i=1:20
            msg = receive(mtm_gc_controller.sub_pos);
            pos_mat = cat(2, pos_mat, msg.Position);
            vel_mat = cat(2, vel_mat, msg.Velocity);
            pause(deta_time);
        end
        mtm_gc_controller.stop_gc();
        is_test_success = false;
    end
    online_gc_drift_vel_plot(vel_mat, duration, deta_time, config.GC_Test.ONLINE_GC_DRT.safe_vel_limit);
    
end

function online_gc_drift_vel_plot(vel_mat, duration, deta_time, safe_vel_limit)
    figure
    for i = 1:7
        subplot(7,1,i);
        x = deta_time: deta_time:size(vel_mat, 2)*deta_time;
        x_limit = deta_time: deta_time: duration; 
        plot(x, vel_mat(i,:), x_limit, safe_vel_limit(i)*ones(1,size(x_limit,2)), 'r--', x_limit, -safe_vel_limit(i)*ones(1,size(x_limit,2)), 'r--');
        title(sprintf('Velocity of joint #%d for drift test within %.1f seconds', i, duration));
        if max(vel_mat(i,:),[],2) <=safe_vel_limit(i)
            axis([0 duration -1.5*safe_vel_limit(i) 1.5*safe_vel_limit(i)])
        else
            axis tight
        end
        if i==3
            ylabel('Joint Velocity(rad/s)') 
        elseif i==7
            xlabel('Time(s)') 
        end
    end
end

function gc_dynamic_params = param_vec_checking(input_vec, rows, columns)
    [rows_t, columns_t] = size(input_vec);
    if rows==rows_t && columns == columns_t
        gc_dynamic_params = input_vec;
    elseif rows==columns_t && rows == columns_t
        gc_dynamic_params = input_vec';
    else
        error("size of dynamic vector is not correct. Current size is (%d, %d). Vector size for gc controller should be (%d, %d)",...
                rows_t, columns_t, rows, columns);
    end
end