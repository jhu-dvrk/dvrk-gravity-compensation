function output_file_str = wizard_config_dataCollection(ARM_NAME,...
                                                        SN)
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Copyright (c)  2018, The Chinese University of Hong Kong
%  This software is provided "as is" under BSD License, with
%  no warranty. The complete license can be found in LICENSE
    
    % checking if arguments correct
    argument_checking(ARM_NAME,...
                      SN);

    % Read 'dataCollection_config.json' file
    fid = fopen('dataCollection_config.json');
    if fid<3
        error('cannot open file dataCollection_config.json, please check the path')
    end
    raw = fread(fid, inf);
    str = char(raw');
    config = jsondecode(str);
    fclose(fid);
      
    % Customized joint pose 
    joint_origin_pose = [0,0,0,0,0,0,0];
    joint_tele_pose = [0,0,0,0,90,0,0];
    % Pose of Joint5 that most likely to hit upper plane
    joint_pose_1 = [0,0,0,0,167,0,0];
    
    % Create a parent data folder for each MTM to store data
    if strcmp(ARM_NAME,'MTML') 
        output_data_path_mtm = [config.data_collection.output_data_root_path,'/MTML_',SN];
    else 
        output_data_path_mtm = [config.data_collection.output_data_root_path,'/MTMR_',SN];
    end
    mkdir(output_data_path_mtm);
    
    % Add additional info in config struct and will save to output file
    config.ARM_NAME = ARM_NAME;
    config.SN = SN;
    
    % Initiate the arm object and move to origin pose
    mtm_arm = mtm(ARM_NAME);
    mtm_arm.move_joint(deg2rad(joint_origin_pose));
    
    
    % Identify the hard limit, [config.data_collection.joint5.theta_angle_max], 
    % to prevent hitting upper panel of cartesian space  
    Joint_No = 3;
    joint_init_pos = joint_pose_1;
    param_name = 'theta_angle_max';
    default_value = config.data_collection.joint5.theta_angle_max;
    joint_init_pos(3) = default_value-7; %10 degree smaller for saftey reason
    goal_msg = 'Moving MTM upward by increasing Joint#3, finish when distal links of MTM 10cm away from top panel of environment';   
    config.data_collection.joint5.theta_angle_max = wizard_move_one_joint(mtm_arm,...
                                                                       joint_init_pos,...
                                                                       Joint_No,...
                                                                       param_name,...
                                                                       ARM_NAME,...
                                                                       default_value,...
                                                                       goal_msg);
   
    % Identify the hard limit, [config.data_collection.joint3.theta_angle_max], 
    % to prevent hitting front panel of cartesian space
    if strcmp(ARM_NAME,'MTML') 
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTML;
    else
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTMR;
    end    
    Joint_No = 2;
    param_name = 'theta_angle_max';
    default_value = config.data_collection.joint3.theta_angle_max;
    joint_init_pos(2) = default_value-5; %10 degree smaller for saftey reason
    joint_init_pos(3) = config.data_collection.joint3.couple_lower_limit-joint_init_pos(2)
    couple_contraint =  config.data_collection.joint3.couple_lower_limit;
    goal_msg = 'Moving MTM forward, finish when distal links of MTM 10cm away from front panel of environment'; 
    config.data_collection.joint3.theta_angle_max = wizard_move_two_joint(mtm_arm,...
                                                                          joint_init_pos,...
                                                                          param_name,...
                                                                          ARM_NAME,...
                                                                          default_value,...
                                                                          goal_msg,...
                                                                          couple_contraint);

    
    % Identify the hard limit, [config.data_collection.joint3.couple_upper_limit], 
    % to prevent hitting upper panel of cartesian space
    if strcmp(ARM_NAME,'MTML') 
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTML;
    else
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTMR;
    end    
    Joint_No = 3;
    joint_init_pos(2) = config.data_collection.joint3.theta_angle_max;
    joint_init_pos(3) = config.data_collection.joint3.couple_upper_limit - 10 - joint_init_pos(2);
    param_name = 'couple_upper_limit';
    default_value = config.data_collection.joint3.couple_upper_limit;
    goal_msg = 'Moving MTM upward by increasing Joint#3, finish when distal links of MTM 10cm away from top panel of environment';;
    config.data_collection.joint3.couple_upper_limit = wizard_move_one_joint(mtm_arm,...
                                                                               joint_init_pos,...
                                                                               Joint_No,...
                                                                               param_name,...
                                                                               ARM_NAME,...
                                                                               default_value,...
                                                                               goal_msg,...
                                                                               true);
                                                                           
                                                                           
    % Identify the hard limit, [config.data_collection.joint2.train_angle_max], 
    % to prevent hitting upper panel of cartesian space  
    Joint_No = 2;
    joint_init_pos = config.data_collection.joint2.init_joint_range;
    param_name = 'train_angle_max';
    default_value = config.data_collection.joint2.train_angle_max;
    joint_init_pos(Joint_No) = default_value-7; 
    goal_msg = 'Moving MTM upward by increasing Joint#2, finish when distal links of MTM 10cm away from top panel of environment';   
    config.data_collection.joint2.train_angle_max= wizard_move_one_joint(mtm_arm,...
                                                                       joint_init_pos,...
                                                                       Joint_No,...
                                                                       param_name,...
                                                                       ARM_NAME,...
                                                                       default_value,...
                                                                       goal_msg);
                                                                           
    

    % Identify the hard limit, [config.data_collection.joint1.train_angle_min.MTML], 
    % to prevent MTML hitting left panel of cartesian space
     if strcmp(ARM_NAME,'MTML') 
        joint_init_pos = config.data_collection.joint1.init_joint_range;
        Joint_No = 1;
        param_name = 'train_angle_min';
        default_value = config.data_collection.joint1.train_angle_min.MTML;
        goal_msg = 'Moving MTM left by decreasing Joint#1, finish when distal links of MTM 10cm away from left panel of environment';;
        config.data_collection.joint1.train_angle_min.MTML = wizard_move_one_joint(mtm_arm,...
                                                                                   joint_init_pos,...
                                                                                   Joint_No,...
                                                                                   param_name,...
                                                                                   ARM_NAME,...
                                                                                   default_value,...
                                                                                   goal_msg); 
        mtm_arm.move_joint(deg2rad(joint_tele_pose));
     end

    % Identify the hard limit, [config.data_collection.joint1.train_angle_max.MTMR], 
    % to prevent MTMR hitting right panel of cartesian space
     if strcmp(ARM_NAME,'MTMR') 
        joint_init_pos = config.data_collection.joint1.init_joint_range;

        Joint_No = 1;
        param_name = 'train_angle_max';
        default_value = config.data_collection.joint1.train_angle_max.MTMR;
        goal_msg = 'Moving MTM left by increasing Joint#1, finish when distal links of MTM 10cm away from right panel of environment';
        config.data_collection.joint1.train_angle_max.MTMR = wizard_move_one_joint(mtm_arm,...
                                                                                   joint_init_pos,...
                                                                                   Joint_No,...
                                                                                   param_name,...
                                                                                   ARM_NAME,...
                                                                                   default_value,...
                                                                                   goal_msg); 
        mtm_arm.move_joint(deg2rad(joint_tele_pose));
     end
     
    % Identify the hard limit, [config.data_collection.joint1.train_angle_max.MTML], 
    % to prevent MTML hitting right panel of cartesian space
     if strcmp(ARM_NAME,'MTML') 
        joint_init_pos = config.data_collection.joint1.init_joint_range;
        joint_init_pos(1) = 30;
        Joint_No = 1;
        param_name = 'train_angle_max';
        default_value = config.data_collection.joint1.train_angle_max.MTML;
        goal_msg = 'Moving MTM right by increasing Joint#1, finish when distal links of MTM 10cm away from right panel of environment/other MTM';
        config.data_collection.joint1.train_angle_max.MTML = wizard_move_one_joint(mtm_arm,...
                                                                                   joint_init_pos,...
                                                                                   Joint_No,...
                                                                                   param_name,...
                                                                                   ARM_NAME,...
                                                                                   default_value,...
                                                                                   goal_msg);  
        mtm_arm.move_joint(deg2rad(joint_tele_pose));
     end
     
    % Identify the hard limit, [config.data_collection.joint1.train_angle_min.MTMR], 
    % to prevent MTMR hitting right panel of cartesian space
     if strcmp(ARM_NAME,'MTMR') 
        joint_init_pos = config.data_collection.joint1.init_joint_range;
        joint_init_pos(1) = -30;
        Joint_No = 1;
        param_name = 'train_angle_min';
        default_value = config.data_collection.joint1.train_angle_min.MTMR;
        goal_msg = 'Moving MTM left by decreasing Joint#1, finish when distal links of MTM 10cm away from left panel of environment/other MTM';
        config.data_collection.joint1.train_angle_min.MTMR = wizard_move_one_joint(mtm_arm,...
                                                                                   joint_init_pos,...
                                                                                   Joint_No,...
                                                                                   param_name,...
                                                                                   ARM_NAME,...
                                                                                   default_value,...
                                                                                   goal_msg);  
        mtm_arm.move_joint(deg2rad(joint_tele_pose));
     end    
     

     
    % Final result output display
    clc
    disp(sprintf('Your customized parameters is:'));
    disp(sprintf('joint3.theta_angle_max: %d', config.data_collection.joint3.theta_angle_max));
    disp(sprintf('joint3.couple_upper_limit: %d', config.data_collection.joint3.couple_upper_limit));
    if strcmp(ARM_NAME,'MTML') 
        disp(sprintf('Joint1.train_angle_min.MTML: %d', config.data_collection.joint1.train_angle_min.MTML));
        disp(sprintf('Joint1.train_angle_max.MTML: %d', config.data_collection.joint1.train_angle_max.MTML));
    end
    if strcmp(ARM_NAME,'MTMR') 
        disp(sprintf('Joint1.train_angle_min.MTMR: %d', config.data_collection.joint1.train_angle_min.MTMR));
        disp(sprintf('Joint1.train_angle_max.MTMR: %d', config.data_collection.joint1.train_angle_max.MTMR));
    end


    % Execute collision checking process
    collision_checking(config);
   
    % Save customized json output file for further data_collection process
    output_file_str = [output_data_path_mtm,'/dataCollection_config_customized.json'];
    fid = fopen(output_file_str,'w');
    jsonStr = jsonencode(config);
    fwrite(fid, jsonStr);
    fclose(fid);
    disp(sprintf('Save config file to %s', output_file_str));
end

function customized_value = wizard_move_one_joint(mtm_arm,...
                                                  joint_init_pos,...
                                                  Joint_No,...
                                                  param_name,...
                                                  ARM_NAME,...
                                                  default_value,...
                                                  goal_msg,...
                                                  is_couple_limit)
    input_str = '';
    disp(sprintf('Setting Hard limit for when collecting data of Joint#%d ',Joint_No))
    disp(sprintf('Moving to init pose'));
    mtm_arm.move_joint(deg2rad(joint_init_pos));
    if(~exist('is_couple_limit'))
        customized_value = joint_init_pos(Joint_No);
    else
        customized_value = joint_init_pos(Joint_No) + joint_init_pos(Joint_No-1);
    end
    joint_pos = joint_init_pos;
    while(true)
         while(~strcmp(input_str,'i') & ~strcmp(input_str,'d') & ~strcmp(input_str,'f'))
            clc
            disp(sprintf('Instruction: %s', goal_msg));
            disp(sprintf('Arm: %s',ARM_NAME));
            disp(sprintf('Joint_No: %d',Joint_No));
            disp(sprintf('Customized Param Name: %s',param_name));           
            disp(sprintf('Recommended Value: [%s] = %d degree ',param_name,default_value));
            disp(sprintf('Current Value: [%s] = %d degree', param_name,customized_value));
            disp('Increase the value by 1 degree: [i]');
            disp('Decrease the value by 1 degree: [d]');
            input_str = input(sprintf('Finish the process: [f]:'),'s');
         end
         if(input_str == 'i')
             customized_value = customized_value+1;      
         elseif(input_str == 'd')
             customized_value = customized_value-1;
         else
             break
         end
         if(~exist('is_couple_limit'))
            joint_pos(Joint_No) = customized_value;
         else
             joint_pos(Joint_No) = customized_value - joint_pos(Joint_No-1);
         end
         mtm_arm.move_joint(deg2rad(joint_pos));
         input_str = '';
    end
end

function customized_value = wizard_move_two_joint(mtm_arm,...
                                                  joint_init_pos,...
                                                  param_name,...
                                                  ARM_NAME,...
                                                  default_value,...
                                                  goal_msg,...
                                                  couple_contraint)
    % Hard Code
    Joint3_pos_min_limit = -35;
    input_str = '';
    disp(sprintf('Setting Hard limit for when collecting data of Joint#%d ',3))
    disp(sprintf('Moving to init pose'));
    mtm_arm.move_joint(deg2rad(joint_init_pos));
    customized_value = joint_init_pos(2);
    joint_pos = joint_init_pos;
    while(true)
         while(~strcmp(input_str,'i') & ~strcmp(input_str,'d') & ~strcmp(input_str,'f'))
            clc
            disp(sprintf('Instruction: %s', goal_msg));
            disp(sprintf('Arm: %s',ARM_NAME));
            disp(sprintf('Joint_No: %d',2));
            disp(sprintf('Customized Param Name: %s',param_name));           
            disp(sprintf('Recommended Value: [%s] = %d degree ',param_name,default_value));
            disp(sprintf('Current Value: [%s] = %d degree', param_name,customized_value));
            disp('Increase the value by 1 degree: [i]');
            disp('Decrease the value by 1 degree: [d]');
            input_str = input(sprintf('Finish the process: [f]:'),'s');
         end
         if(input_str == 'i')
             customized_value = customized_value+1;      
         elseif(input_str == 'd')
             customized_value = customized_value-1;
         else
             break
         end
         joint_pos(2) = customized_value;
         joint_pos(3) = couple_contraint - joint_pos(2);
         if joint_pos(3)<= Joint3_pos_min_limit
             customized_value = customized_value-1;
             joint_pos(2) = customized_value;
             joint_pos(3) = couple_contraint - joint_pos(2);
         end
         mtm_arm.move_joint(deg2rad(joint_pos));
         input_str = '';
    end
end


function argument_checking(ARM_NAME,...
                           SN)
    if~strcmp(ARM_NAME,'MTML') & ~strcmp(ARM_NAME,'MTMR') 
        error(sprintf(['Input of argument ''ARM_NAME''= %s is error, you should input one of the string',...
                       '[''MTML'',''MTMR'']'],ARM_NAME));
    end
    if ~ischar(SN)
        error(sprintf('SN input %s is not char arrary', SN))
    end
end


function collision_checking(config)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
    
    % General variables
    ARM_NAME = config.ARM_NAME;  
    is_collision_checking = true;
    is_collecting_data = false;
    
    % Vitual path for collision checking
    input_data_path = 'test';
    
    % create mtm_arm obj and move every arm in home joint position
    mtm_arm = mtm(ARM_NAME);
    mtm_arm.move_joint([0,0,0,0,0,0,0]);   
    
    [config_joint1, config_joint2, config_joint3,config_joint4,config_joint5,config_joint6]= setting_dataCollection(config,...
                                                                                                                    input_data_path);
    collect_mtm_one_joint(config_joint6, mtm_arm, is_collision_checking, is_collecting_data);
    collect_mtm_one_joint(config_joint5, mtm_arm, is_collision_checking, is_collecting_data);
    collect_mtm_one_joint(config_joint4, mtm_arm, is_collision_checking, is_collecting_data);
    collect_mtm_one_joint(config_joint3, mtm_arm, is_collision_checking, is_collecting_data);
    collect_mtm_one_joint(config_joint2, mtm_arm, is_collision_checking, is_collecting_data);
    collect_mtm_one_joint(config_joint1, mtm_arm, is_collision_checking, is_collecting_data);
    mtm_arm.move_joint([0,0,0,0,0,0,0]);   

end

