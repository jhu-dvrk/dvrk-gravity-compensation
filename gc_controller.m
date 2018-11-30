function mtm_gc_controller = gc_controller(gc_controller_cofig_json)
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Copyright (c)  2018, The Chinese University of Hong Kong
%  This software is provided "as is" under BSD License, with
%  no warranty. The complete license can be found in LICENSE

%     % check if input arguements correct
%     arguement_checking(gc_controller_cofig_json,...
%                        ARM_data_path_with_date);
    
    % Checking arguments
    argument_checking(gc_controller_cofig_json);
    
    % Reading config file "gc_controller_cofig_json"
    fid = fopen(gc_controller_cofig_json);
    if fid<3
        error(sprintf('Cannot find file %s',gc_controller_cofig_json))
    end
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);

    % Version Checking
    if config.version ~= '1.0';
        error('The version of dvrk gravity compensation should be 1.0')
    end
    
    % General Setting
    safe_upper_torque_limit = config.GC_controller.safe_upper_torque_limit;
    safe_lower_torque_limit = config.GC_controller.safe_lower_torque_limit;
    amplitude_vec = config.GC_controller.amplitude_vec;
    Zero_Output_Joint_No = [];
    beta_vel_amplitude = config.GC_controller.beta_vel_amplitude; 
    GC_controllers = [];
    ARM_NAME = config.ARM_NAME;
    g = config.lse.g_constant; 
   
    % Load MTM GC Param
    dynamic_params_pos = config.GC_controller.gc_dynamic_params_pos;
    dynamic_params_neg = config.GC_controller.gc_dynamic_params_neg;
  
    % % Spawn GC Controllers and test
    mtm_arm = mtm(ARM_NAME);
    mtm_gc_controller= controller(mtm_arm,...
                                  dynamic_params_pos,...
                                  dynamic_params_neg,...
                                  safe_upper_torque_limit,...
                                  safe_lower_torque_limit,...
                                  beta_vel_amplitude,...
                                  g,...
                                  Zero_Output_Joint_No,...                              
                                  amplitude_vec,...
                                  ARM_NAME);
                              
    % Test Controller
    disp(sprintf('Testing GC controller of %s performance',ARM_NAME));
    disp(sprintf('Calculating "error rate" between predicted and measuring torques',ARM_NAME));
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
    GC_controllers.abs_err = abs_err;
    GC_controllers.rel_err = rel_err;
    disp(sprintf('===================='));
    disp(sprintf('Test Result for %s', ARM_NAME));
    for j=1:size(pos_name_cell)
    disp(sprintf('For Pose_%d Joint_No: [''absolute error''], [''error rate%%'']',j));
        for k = 1:7
            disp(sprintf('Joint%d:[%.4f], [%d%%]',k, abs_err_mat_MTM(k,j), int32(rel_err_mat_MTM(k,j)*100)));
        end
    end
    for j=1:size(pos_name_cell)
        for k = 1:7
            if(int32(rel_err_mat_MTM(k,j)*100)>=config.GC_controller.GC_test_error_rate_threshold)
                warning(sprintf('[Test Pos %d]: --%s Joint%d-- absolute torque error:[%.4f], error rate:[%d%%], has exceed the error rate threshold %d%%',...
                                                j,ARM_NAME,k,...
                                                abs_err_mat_MTM(k,j), int32(rel_err_mat_MTM(k,j)*100),...
                                                config.GC_controller.GC_test_error_rate_threshold));
                                            
                % Return a empty struct if the gravity compensation test fails
                mtm_gc_controller = [];
                return 
            end
        end
    end
    
    % Move to gc controller start joint position and wait until MTM remains static
    mtm_gc_controller.mtm_arm.move_joint(deg2rad(config.GC_controller.GC_init_pos));
    pause(2.5); 
    
    % Start gc controller
    mtm_gc_controller.start_gc();


    % Assign output struct
    GC_controllers.controller = mtm_gc_controller;

end

function argument_checking(gc_controller_cofig_json)
    if  ~ischar(gc_controller_cofig_json)
        error('Input argument "gc_controller_cofig_json" should be a char arrary')
    end
    if ~(strcmp(gc_controller_cofig_json(end-4:end),'.json'))
        error(sprintf(['Input of argument ''gc_controller_cofig_json''= %s',...
                      ' is error, you should input file with .json extend format '],gc_controller_cofig_json));
    end
end


