function output_file_str = dataCollection(dataCollection_config_str)
    %  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
    %  Created on: 2018-10-05
    %  Copyright (c)  2018, The Chinese University of Hong Kong
    %  This software is provided "as is" under BSD License, with
    %  no warranty. The complete license can be found in LICENSE

    % checking if arguments correct
    argument_checking(dataCollection_config_str);

    % Read JSON Config File
    fprintf('Loading JSON File %s\n', dataCollection_config_str);
    fid = fopen(dataCollection_config_str);
    if fid<3
        error('Cannot read file %s', dataCollection_config_str)
    end
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);
    ARM_NAME = config.ARM_NAME;

    % Create unique date folder to store the collecting data
    date_time = datestr(datetime('now'),'mmmm-dd-yyyy-HH:MM:SS');
    [MTM_data_path,~,~] = fileparts(dataCollection_config_str);
    input_data_path_with_date = [MTM_data_path,'/',date_time];
    mkdir(input_data_path_with_date);

    % Add addtional info in config file and will output to file
    config.date_time = date_time;

    % Save wizard JSON file
    output_file_str = [input_data_path_with_date, '/', 'dataCollection_info.json'];
    fid = fopen(output_file_str,'w');
    jsonStr = jsonencode(config);
    fwrite(fid, jsonStr);
    fclose(fid);
    fprintf('Save config file to %s\n', output_file_str);

    % Create mtm_arm obj and move every arm in home position for safety reason
    mtm_arm = mtm(ARM_NAME);
    mtm_arm.move_joint([0,0,0,0,0,0,0]);

    config_joint_list = setting_dataCollection(config,...
                                               input_data_path_with_date);

    % dataCollection
    is_collision_checking = false;
    is_collecting_data = true; 
    current_progress = 0.0;
    total_data_sets = 0;
    for j=1:size(config_joint_list,2)
        total_data_sets = total_data_sets + config_joint_list{j}.data_size; 
    end
    one_data_progress_increment = 100 / total_data_sets;
    for i=1:size(config_joint_list,2)
            current_progress = collect_mtm_one_joint(config_joint_list{i},...
                                  mtm_arm,...
                                  is_collision_checking,...
                                  is_collecting_data,...
                                  current_progress,...
                                  one_data_progress_increment);
    end
    mtm_arm.move_joint([0,0,0,0,0,0,0]);
end

function argument_checking(dataCollection_config_str)
    if ~ischar(dataCollection_config_str)
        error('Argument dataCollection_config_str should be char array instead of %s', dataCollection_config_str)
    end
end
