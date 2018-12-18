function mtm_gc_controller = estimate_gc_params(ARM_NAME,SN)
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Copyright (c)  2018, The Chinese University of Hong Kong
%  This software is provided "as is" under BSD License, with
%  no warranty. The complete license can be found in LICENSE

    % argument input type checking
    arguement_checking(ARM_NAME,SN)

    disp(sprintf('This software is being developed by Biorobotics and Medical Technology Group of The Chinese University of Hong Kong (CUHK).'));
    disp(sprintf('Author(s):  Hongbin LIN, Vincent Hui, Samuel Au'));

    output_file_str = wizard_config_dataCollection(ARM_NAME, SN);
    output_file_str = dataCollection(output_file_str);
    output_file_str = mlse(output_file_str);
    mtm_gc_controller = gc_controller(output_file_str);

end

function arguement_checking(ARM_NAME,SN)
    % Argument Checking
    if~ischar(ARM_NAME)| ~ischar(SN)
        error('Both Argument ''ARM_NAME'' and ''SN'' should be string object');
    end
    if~(strcmp(ARM_NAME,'MTML') | strcmp(ARM_NAME,'MTMR') )
        error(sprintf(['Input of argument ''ARM_NAME''= %s is error, you should input one of the string',...
                       '[''MTML'',''MTMR'']'],ARM_NAME));
    end
end
