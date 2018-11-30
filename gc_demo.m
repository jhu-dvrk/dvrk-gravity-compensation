function [GC_controllers,output_data_struct] = gc_demo(ARM_NAME,SN)
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Copyright (c)  2018, The Chinese University of Hong Kong
%  This software is provided "as is" under BSD License, with
%  no warranty. The complete license can be found in LICENSE
    
    % argument input type checking
    arguement_checking(ARM_NAME)

    % string of config json files
    dataCollection_config_json_str = 'dataCollection_config.json';
    LSE_config_json_str =  'MLSE_config.json';
    GC_controller_config_json_str = 'GC_controller_config.json';

    output_file_str = wizard_dataCollection_config(dataCollection_config_json_str)

    
end

function arguement_checking(ARM_NAME,SN)
    % Argument Checking
    if~ischar(ARM_NAME) 
        error('Both Argument ''ARM_NAME'' should be string object');
    end
    if~ischar(SN)
        error(z)
    end
    if~(strcmp(ARM_NAME,'MTML') | strcmp(ARM_NAME,'MTMR') | strcmp(ARM_NAME,'MTML&MTMR'))
        error(sprintf(['Input of argument ''ARM_NAME''= %s is error, you should input one of the string',...
                       '[''MTML'',''MTMR'',''MTML&MTMR'']'],ARM_NAME));
    end
end
