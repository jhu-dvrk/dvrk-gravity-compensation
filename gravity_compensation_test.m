function is_test_success =  gravity_compensation_test(config, test_mode)

    mtm_gc_controller = online_test_init(config);
    online_gc_predict_error_test(mtm_gc_controller, config);
    if strcmp(test_mode, 'ONLINE_GC_PRT_ERR')
        is_test_success = online_gc_predict_error_test(mtm_gc_controller, config);
    elseif strcmp(test_mode, 'ONLINE_GC_DRT')
        is_test_success = online_gc_drift_test(mtm_gc_controller)
    end
end

function mtm_gc_controller = online_test_init(config)
    % % Spawn GC Controllers and test
    mtm_arm = mtm(ARM_NAME);
    mtm_gc_controller= controller(mtm_arm,...
        config.GC_controller.gc_dynamic_params_pos',...
        config.GC_controller.gc_dynamic_params_neg',...
        config.GC_controller.safe_upper_torque_limit,...
        config.GC_controller.safe_lower_torque_limit,...
        config.GC_controller.beta_vel_amplitude,...
        config.lse.g_constant,...
        [],...
        config.GC_controller.GC_test_constant_vec,...
        config.ARM_NAME);
end

function is_test_success = online_gc_predict_error_test(mtm_gc_controller, config)
      % Test Controller
    fprintf('Testing GC controller for %s\n', ARM_NAME);
    fprintf('Calculating "error rate" between predicted and measured torques\n');
    pos_name_cell = fieldnames(config.GC_controller.GC_test);
    test_pos_mat = [];
    abs_err_mat_MTM = [];
    rel_err_mat_MTM = [];
    for k=1:size(pos_name_cell)
        test_pos_mat = [test_pos_mat,getfield(config.GC_controller.GC_test,pos_name_cell{k})];
        [abs_err, rel_err] = mtm_gc_controller.controller_test(deg2rad(test_pos_mat(:,end)));
        abs_err_mat_MTM= [abs_err_mat_MTM,abs_err];
        rel_err_mat_MTM = [rel_err_mat_MTM,rel_err];
    end
    fprintf('====================\n');
    fprintf('Test Result for %s\n', ARM_NAME);
    for j=1:size(pos_name_cell)
        fprintf('For Pose_%d Joint_No: [''absolute error''], [''error rate%%'']\n', j);
        for k = 1:7
            fprintf('Joint%d:[%.4f], [%d%%]\n', k , abs_err_mat_MTM(k,j), int32(rel_err_mat_MTM(k,j)*100));
        end
    end
    for j=1:size(pos_name_cell)
        for k = 1:7
            if(int32(rel_err_mat_MTM(k,j)*100)>=config.GC_controller.GC_test_error_rate_threshold)
                warning('[Test Pos %d]: --%s Joint%d-- absolute torque error:[%.4f], error rate:[%d%%], has exceed the error rate threshold %d%%',...
                    j,ARM_NAME,k,...
                    abs_err_mat_MTM(k,j), int32(rel_err_mat_MTM(k,j)*100),...
                    config.GC_controller.GC_test_error_rate_threshold);
            end
        end
    end
    % Move MTM to origin position
    mtm_gc_controller.mtm_arm.move_joint(deg2rad(zeros(1,7)));
   
end

function is_test_success = online_gc_drift_test(mtm_gc_controller)
    mtm_gc_controller.start_gc();
    pause(3);
    rate = 100;
    deta_time = 1/rate;
    pos_mat = [];
    vel_mat = [];
    for i=1:count
        msg = receive(mtm_gc_controller.sub_pos);
        pos_mat = cat(2, pos_mat, msg.Position);
        vel_mat = cat(2, vel_mat, msg.Velocity);
        pause(deta_time)
    end
    is_test_success = true;
end