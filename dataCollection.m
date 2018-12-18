function output_file_str = dataCollection(dataCollection_config_str)
    %  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
    %  Created on: 2018-10-05
    %  Copyright (c)  2018, The Chinese University of Hong Kong
    %  This software is provided "as is" under BSD License, with
    %  no warranty. The complete license can be found in LICENSE

    % checking if arguments correct
    argument_checking(dataCollection_config_str);

    % Read JSON Config File
    disp(sprintf('Loading JSON File %s', dataCollection_config_str));
    fid = fopen(dataCollection_config_str);
    if fid<3
        error('Cannot read file %s',dataCollection_config_str)
    end
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);
    ARM_NAME = config.ARM_NAME;

    % Create unique date folder to store the collecting data
    date_time = datestr(datetime('now'),'mmmm-dd-yyyy-HH:MM:SS');
    [MTM_data_path,name,ext] = fileparts(dataCollection_config_str);
    input_data_path_with_date = [MTM_data_path,'/',date_time];
    mkdir(input_data_path_with_date);

    % Add addtional info in config file and will output to file
    config.date_time = date_time;

    % Save wizard JSON file
    output_file_str = [input_data_path_with_date,'/', 'dataCollection_info.json'];
    fid = fopen(output_file_str,'w');
    jsonStr = jsonencode(config);
    fwrite(fid, jsonStr);
    fclose(fid);
    disp(sprintf('Save config file to %s', output_file_str));



    % Create mtm_arm obj and move every arm in home position for safety reason
    mtm_arm = mtm(ARM_NAME);
    mtm_arm.move_joint([0,0,0,0,0,0,0]);

    [config_joint1, config_joint2, config_joint3,config_joint4,config_joint5,config_joint6]= setting_dataCollection(config,...
        input_data_path_with_date);

    % dataCollection
    is_collision_checking = false;
    is_collecting_data = true
    output_data_struct.joint6 = collect_mtm_one_joint(config_joint6, mtm_arm, is_collision_checking, is_collecting_data);
    output_data_struct.joint5 = collect_mtm_one_joint(config_joint5, mtm_arm, is_collision_checking, is_collecting_data);
    output_data_struct.joint4 = collect_mtm_one_joint(config_joint4, mtm_arm, is_collision_checking, is_collecting_data);
    output_data_struct.joint3 = collect_mtm_one_joint(config_joint3, mtm_arm, is_collision_checking, is_collecting_data);
    output_data_struct.joint2 = collect_mtm_one_joint(config_joint2, mtm_arm, is_collision_checking, is_collecting_data);
    output_data_struct.joint1 = collect_mtm_one_joint(config_joint1, mtm_arm, is_collision_checking, is_collecting_data);
    mtm_arm.move_joint([0,0,0,0,0,0,0]);
end

function argument_checking(dataCollection_config_str)
    if ~ischar(dataCollection_config_str)
        error('Argument dataCollection_config_str should be char array instead of %s', dataCollection_config_str)
    end
end
