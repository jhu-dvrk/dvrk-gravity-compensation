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
    GC_test_constant_vec = config.GC_controller.GC_test_constant_vec;
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
        GC_test_constant_vec,...
        ARM_NAME);

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


