function output_file_str = mlse(dataCollection_info_str)
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Copyright (c)  2018, The Chinese University of Hong Kong
%  This software is provided "as is" under BSD License, with
%  no warranty. The complete license can be found in LICENSE

    argument_checking(dataCollection_info_str)
    
    % General Setting
    output_file_str = '';
   
    % Read JSON config input file dataCollection_info_str
    fid = fopen(dataCollection_info_str);
    if fid<3
        error('Cannot read file %s',dataCollection_info_str)
    end
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);
    
    input_data_path_with_date = config.input_data_path_with_date;
    
    % display data input root path
    disp(' ');
    disp(sprintf('data path for MLSE : ''%s'' ',input_data_path_with_date));
    disp(' ');
    
    % read JSON config file "mlse_config.json"
    fid = fopen('mlse_config.json');
    if fid<3
        error('Cannot read file "mlse_config.json"')
    end
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config_temp = jsondecode(str);
    config.lse = config_temp.lse;
    
    % read JSON config file "gc_controller_config.json"
    fid = fopen('gc_controller_config.json');
    if fid<3
        error('Cannot read file "gc_controller_config.json"')
    end
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config_temp = jsondecode(str);
    config.GC_controller = config_temp.GC_controller;
   

    % create config_LSE objects
    [config_lse_joint1,config_lse_joint2, config_lse_joint3,config_lse_joint4,config_lse_joint5,config_lse_joint6]=...
                        setting_lse(config,input_data_path_with_date);

   % Multi-steps MLSE from Joint#6 to Joint#1.
    lse_mtm_one_joint(config_lse_joint6);
    lse_mtm_one_joint(config_lse_joint5, config_lse_joint6);
    lse_mtm_one_joint(config_lse_joint4, config_lse_joint5);
    lse_mtm_one_joint(config_lse_joint3 ,config_lse_joint4);
    lse_mtm_one_joint(config_lse_joint2 ,config_lse_joint3);
    output_dynamic_matrix = lse_mtm_one_joint(config_lse_joint1 ,config_lse_joint2);

    % Save the output parameters for gravity compensation controller
    config = rmfield(config,'data_collection');
    config.GC_controller.gc_dynamic_params_pos = output_dynamic_matrix(1:40)';
    config.GC_controller.gc_dynamic_params_neg = [output_dynamic_matrix(1:10);output_dynamic_matrix(41:70)]';
    config.version = '1.0';
    output_file_str = [input_data_path_with_date,'/gc-',config.ARM_NAME,'-',config.SN,'.json'];
    fid = fopen(output_file_str,'w');
    jsonStr = jsonencode(config);
    fwrite(fid, jsonStr);
    fclose(fid);
    disp(sprintf('Save config file to %s', output_file_str));
   
end

function argument_checking(input_data_path_with_date)
    if ischar(input_data_path_with_date) ==0
        error(sprintf('%s is not a char object',input_data_path_with_date))
    end    
end

function dynamic_param = lse_mtm_one_joint(config_lse_joint, previous_config)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    if ~exist('previous_config')
        disp(sprintf('LSE of Joint%d  has start..',config_lse_joint.Joint_No));
        
        dynamic_param = lse_model(config_lse_joint);

     
    else
        disp(sprintf('LSE of Joint%d has start..',config_lse_joint.Joint_No));
        
        % if there is 'previous_config', we pass the path to the result of previous step of LSE to lse_model
        dynamic_param = lse_model(config_lse_joint,...
                                  previous_config.output_param_path); 
    end  
end  

function dynamic_parameters_vec = lse_model(config_lse_joint1,...
                                        old_param_path)
                                                       
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    
    lse_obj = lse_preparation(config_lse_joint1);

    R2_augmented = [lse_obj.R2_augmented_pos; lse_obj.R2_augmented_neg];
    T2_augmented = [lse_obj.T2_augmented_pos; lse_obj.T2_augmented_neg];
    
    % Generate the input argument,input_param_map and input_param_rel_std_map, for LSE
    if exist('old_param_path')
        % If there is old parameters which is trained by the previous step, we need to simplify the regressor and torque matrix using the old paramters
        load(strcat(old_param_path,'/param.mat'));
        old_param_map = output_param_map;
        old_param_rel_std_map = output_param_rel_std_map;

        % If prior overlaps with old param values, we choose use old param values instead of piror values.
        prior_param_map = containers.Map(lse_obj.prior_param_index,lse_obj.prior_param_values);
        prior_param_map.remove(0);
        prior_k = prior_param_map.keys;
        old_index_array = cell2mat(output_param_map.keys);
        if size(prior_k,2)~=0
            for j=1:size(prior_k,2)
                if ismember(prior_k{j}, old_index_array)
                    prior_param_map.remove(prior_k{j});
                end
            end
        end
        prior_param_index = prior_param_map.keys;
        prior_param_values = prior_param_map.values;

        % input_param_map =[old_param_map,prior_param_map]
        input_param_index = horzcat(old_param_map.keys, prior_param_index);
        input_param_value = horzcat(old_param_map.values, prior_param_values);
        input_param_map = containers.Map(input_param_index, input_param_value);
        
        %input_param_rel_std_map = old_param_rel_std_map
        input_param_rel_std_map = containers.Map(old_param_rel_std_map.keys,old_param_rel_std_map.values);
    else
        % input_param_map = empty
        input_param_map = containers.Map(config_lse_joint1.prior_param_index, config_lse_joint1.prior_param_values);
        input_param_map.remove(0);
        
        % input_param_rel_std_map = empty
        input_param_rel_std_map = containers.Map({0},{0});
        input_param_rel_std_map.remove(0);
    end
    
    % general config
    Output_Param_Joint_No = lse_obj.Output_Param_Joint_No;
    Train_Joint_No = lse_obj.Joint_No;

    [output_param_map,output_param_full_map, output_param_rel_std_map] = lse(input_param_map,...
                                                                            input_param_rel_std_map,...
                                                                            R2_augmented,... 
                                                                            T2_augmented,...
                                                                            Train_Joint_No,...
                                                                            Output_Param_Joint_No);

    output_keys = output_param_map.keys;
    output_values = output_param_map.values;
    dynamic_parameters_vec = zeros(size(output_param_full_map.values,2),1);
    dynamic_parameters_vec(:) = cell2mat(output_param_full_map.values.');
    output_param_rel_std_values = output_param_rel_std_map.values;
    
    disp('Output Parameters: [Value], [Relative_std(%)]');
    for i=1:size(output_param_map.keys,2)
        disp(sprintf('Param_%d: [%0.4f], [%0.1f%%]',output_keys{i}, output_values{i}, output_param_rel_std_values{i}));
    end
    disp(' ');
    
    % Plot the fitting figures using trained parameters
    if lse_obj.Is_Plot~=0 | lse_obj.issave_figure~=0
       plot_fitting_curves(lse_obj,'pos',dynamic_parameters_vec);
       plot_fitting_curves(lse_obj,'neg',dynamic_parameters_vec);
    end

    % Save param to file
    if exist(lse_obj.new_param_save_path)~=7
        mkdir(lse_obj.new_param_save_path)
    end
    fit_method = lse_obj.fit_method;
    g_constant = lse_obj.g_constant;
    save(strcat(lse_obj.new_param_save_path,'/param.mat'),'output_param_map','output_param_full_map','output_param_rel_std_map','fit_method','g_constant');
   
end


function lse_obj = lse_preparation(config_lse_joint)

    % create lse_obj inheriting config_lse_joint
    lse_obj.Joint_No = config_lse_joint.Joint_No;
    lse_obj.std_filter = config_lse_joint.std_filter;
    lse_obj.g_constant = config_lse_joint.g_constant;
    lse_obj.Is_Plot = config_lse_joint.Is_Plot;
    lse_obj.issave_figure = config_lse_joint.issave_figure;
    lse_obj.Input_Pos_Data_Path = config_lse_joint.input_pos_data_path; 
    lse_obj.Input_Neg_Data_Path = config_lse_joint.input_neg_data_path; 
    lse_obj.input_pos_data_files = config_lse_joint.input_pos_data_files;
    lse_obj.input_neg_data_files = config_lse_joint.input_neg_data_files;
    lse_obj.new_param_save_path = config_lse_joint.output_param_path;
    lse_obj.new_fig_pos_save_path = config_lse_joint.output_pos_fig_path;
    lse_obj.new_fig_neg_save_path = config_lse_joint.output_neg_fig_path;
    lse_obj.prior_param_index = config_lse_joint.prior_param_index;
    lse_obj.prior_param_values = config_lse_joint.prior_param_values;
    lse_obj.Output_Param_Joint_No = config_lse_joint.Output_Param_Joint_No;
    lse_obj.std_filter = config_lse_joint.std_filter;
    lse_obj.fit_method = config_lse_joint.fit_method;

    % check the given joint path exist
    if exist(lse_obj.Input_Pos_Data_Path)==0
       error(sprintf('Cannot find input data folder: %s, Please check if input data folder exist. ',lse_obj.Input_Pos_Data_Path));
    end
    if exist(lse_obj.Input_Neg_Data_Path)==0
       error(sprintf('Cannot find input data folder: %s, Please check if input data folder exist. ',lse_obj.Input_Neg_Data_Path));
    end
    
    data_path_struct_list = dir(lse_obj.input_pos_data_files);
    lse_obj.Torques_pos_data_list = {};
    lse_obj.theta_pos_list = {};
    for i=1:size(data_path_struct_list,1)
        load(strcat(data_path_struct_list(i).folder,'/',data_path_struct_list(i).name));
        lse_obj.Torques_pos_data_list{end+1} = torques_data_process(current_position,...
                                                                    desired_effort,...
                                                                    'mean',...
                                                                    lse_obj.std_filter);
        lse_obj.theta_pos_list{end+1} = int32(Theta);
    end
    
    data_path_struct_list = dir(lse_obj.input_neg_data_files);
    lse_obj.Torques_neg_data_list = {};
    lse_obj.theta_neg_list = {};
    for i=1:size(data_path_struct_list,1)
        load(strcat(data_path_struct_list(i).folder,'/',data_path_struct_list(i).name));
         lse_obj.Torques_neg_data_list{end+1} = torques_data_process(current_position,...
                                                                    desired_effort,...
                                                                    'mean',...
                                                                    lse_obj.std_filter);
        lse_obj.theta_neg_list{end+1} = int32(Theta);
    end
    
     % Append List Torques Data
    lse_obj.Torques_pos_data = [];
    for j = 1:size(lse_obj.Torques_pos_data_list,2)
        lse_obj.Torques_pos_data = cat(3,lse_obj.Torques_pos_data,lse_obj.Torques_pos_data_list{j});
    end
    lse_obj.Torques_neg_data = [];
    for j = 1:size(lse_obj.Torques_neg_data_list,2)
        lse_obj.Torques_neg_data = cat(3,lse_obj.Torques_neg_data,lse_obj.Torques_neg_data_list{j});
    end
    
    [lse_obj.R2_augmented_pos, lse_obj.T2_augmented_pos] = data2augmat(lse_obj.Torques_pos_data,...
                                                       lse_obj.Joint_No,...
                                                       'pos',...
                                                       lse_obj.g_constant);
    
    [lse_obj.R2_augmented_neg, lse_obj.T2_augmented_neg] = data2augmat(lse_obj.Torques_neg_data,...
                                                       lse_obj.Joint_No,...
                                                       'neg',...
                                                       lse_obj.g_constant);
    
end

function Torques_data = torques_data_process(current_position, desired_effort, method, std_filter)
    %current_position = current_position(:,:,1:10);
    %desired_effort = desired_effort(:,:,1:10);
    d_size = size(desired_effort);
    Torques_data = zeros(7,2,d_size(2));
    %First Filter out Point out of 1 std, then save the date with its index whose value is close to mean
    for i=1:d_size(2)
        for j=1:d_size(1)
            for k=1:d_size(3)
                effort_data_array(k)=desired_effort(j,i,k);
                position_data_array(k)=current_position(j,i,k);
            end
            effort_data_std = std(effort_data_array);
            effort_data_mean = mean(effort_data_array);
            if effort_data_std<0.0001
                effort_data_std = 0.0001;
            end
            %filter out anomalous data out of 1 standard deviation
            select_index = (effort_data_array <= effort_data_mean+effort_data_std*std_filter)...
                &(effort_data_array >= effort_data_mean-effort_data_std*std_filter);
            
            effort_data_filtered = effort_data_array(select_index);
            position_data_filtered = position_data_array(select_index);
            if size(effort_data_filtered,2) == 0
                effort_data_filtered =effort_data_array;
                position_data_filtered = position_data_array;
            end
            effort_data_filtered_mean = mean(effort_data_filtered);
            position_data_filtered_mean = mean(position_data_filtered);
            for e = 1:size(effort_data_filtered,2)
                if e==1
                    final_index = 1;
                    min_val =abs(effort_data_filtered(e)-effort_data_filtered_mean);
                else
                   abs_result =abs(effort_data_filtered(e)-effort_data_filtered_mean);
                   if(min_val>abs_result)
                       min_val = abs_result;
                       final_index = e;
                   end
                end
            end
            if(strcmp(lower(method),'mean'))
                Torques_data(j,1,i) = position_data_filtered_mean;
                Torques_data(j,2,i) = effort_data_filtered_mean;              
            elseif(strcmp(lower(method),'min_abs_error'))
                Torques_data(j,1,i) = current_position(j,i,final_index);
                Torques_data(j,2,i) = desired_effort(j,i,final_index);
            else
                Error('Method argument is wrong, please pass: mean or min_abs_error.')
            end
        end
    end

    % Tick out the data collecting from some joint configuration which reaches limits and have cable force effect. 
    Torques_data = Torques_data(:,:,3:end-1);
end

function  [R2_augmented, T2_augmented] = data2augmat(Torques_data,...
                                                     Joint_No,...
                                                     direction,...
                                                     g) 
    R2_augmented = [];
    T2_augmented = [];

    for i=1:size(Torques_data,3)
        R = analytical_regressor_mat_dual_dir(direction,g,Torques_data(:,1,i)');
        R2_augmented = [R2_augmented;R(Joint_No,:)];
        T2_augmented = [T2_augmented;Torques_data(Joint_No,2,i)];
    end 

end

function plot_fitting_curves(lse_obj,direction,dynamic_parameters_vec)
    if(strcmp(direction,'pos'))
        Torques_data_list = lse_obj.Torques_pos_data_list;
        theta_list = lse_obj.theta_pos_list;
        fig_save_path = lse_obj.new_fig_pos_save_path;
    elseif(strcmp(direction,'neg'))
        Torques_data_list = lse_obj.Torques_neg_data_list;
        theta_list = lse_obj.theta_neg_list;
        fig_save_path = lse_obj.new_fig_neg_save_path;
    end
    for j = 1:size(Torques_data_list,2)
        if(size(Torques_data_list{j},3)~=0  )
        plot_fit_joint(Torques_data_list{j},...
                       dynamic_parameters_vec,...
                       theta_list{j},...
                       direction,...
                       lse_obj.Joint_No, ...
                       lse_obj.g_constant,...
                       lse_obj.Is_Plot, ...
                       lse_obj.issave_figure,...
                       fig_save_path, ...
                       j);
        end
    end
end

function plot_fit_joint(Torques_data,...
                        dynamic_parameters,...
                        theta,...
                        dir,...
                        Joint_No,... 
                        g,...
                        isplot,...
                        issave,...
                        save_path,...
                        save_file_index)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05


    for i=1:size(Torques_data,3)
        Regressor_Matrix = analytical_regressor_mat_dual_dir(dir,g,Torques_data(:,1,i));
        F = Regressor_Matrix(Joint_No,:)*dynamic_parameters;

        x(i) = Torques_data(Joint_No,1,i);
        y1(i) = Torques_data(Joint_No,2,i);
        y2(i) = F;
    end
    x= x.';
    x=x*180/pi;
    y1= y1.';  
    y2= y2.';

    if isplot
        figure;
    else
        figure('visible', 'off')
    end
    title_string = sprintf('Actual and Predicted Torque of Joint%d at theta=%d', Joint_No, theta);
    xlabel_string = sprintf('Joint %d Angle',Joint_No);
    ylabel_string = sprintf('Joint %d Torque',Joint_No);
    scatter(x,y1,100);
    hold on
    plot(x,y2)
    title(title_string);
    xlabel(xlabel_string);
    ylabel(ylabel_string);

    if issave == 1
        if exist(save_path)~=7
            mkdir(save_path)
        end
        saveas(gcf, strcat(save_path,'/Figure_',int2str(save_file_index),'_',title_string,'.png'));
        disp(sprintf(strcat('Figure, [',title_string,'.png], has saved.')));
    end
end



