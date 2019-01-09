function mtm_gc_controller = gc_controller(gc_controller_config_json)
    %  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
    %  Created on: 2018-10-05
    %  Copyright (c)  2018, The Chinese University of Hong Kong
    %  This software is provided "as is" under BSD License, with
    %  no warranty. The complete license can be found in LICENSE

    %     % check if input arguements correct
    %     arguement_checking(gc_controller_cofig_json,...
    %                        ARM_data_path_with_date);

    % Checking arguments
    argument_checking(gc_controller_config_json);

    % Reading config file "gc_controller_config_json"
    fid = fopen(gc_controller_config_json);
    if fid<3
        error('Cannot find file %s', gc_controller_config_json)
    end
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);

    % Version Checking
    if config.version ~= '1.0'
        error('The version of dVRK gravity compensation should be 1.0')
    end

    % General Setting
    GC_controllers = [];
    param_num = 40; %hard code
    
    gc_dynamic_params_pos = param_vec_checking(config.GC_controller.gc_dynamic_params_pos, param_num, 1);
    gc_dynamic_params_neg = param_vec_checking(config.GC_controller.gc_dynamic_params_neg, param_num, 1);
    
    
    % % Spawn GC Controllers and test
    mtm_arm = mtm(config.ARM_NAME);
    mtm_gc_controller= controller(mtm_arm,...
        gc_dynamic_params_pos,...
        gc_dynamic_params_neg,...
        config.GC_controller.safe_upper_torque_limit,...
        config.GC_controller.safe_lower_torque_limit,...
        config.GC_controller.beta_vel_amplitude,...
        config.lse.g_constant,...
        config.ARM_NAME);

    % Move to gc controller start joint position and wait until MTM remains static
    mtm_gc_controller.mtm_arm.move_joint(deg2rad(config.GC_controller.GC_init_pos));
    pause(2.5);

    % Start gc controller
    mtm_gc_controller.start_gc();


    % Assign output struct
    GC_controllers.controller = mtm_gc_controller;

end

function argument_checking(gc_controller_config_json)
    if  ~ischar(gc_controller_config_json)
        error('Input argument "gc_controller_config_json" should be a char array')
    end
    if ~(strcmp(gc_controller_config_json(end-4:end),'.json'))
        sprintf(['Input of argument ''gc_controller_config_json''= %s',...
            ' is error, you should input file with .json extension '], gc_controller_config_json);
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
