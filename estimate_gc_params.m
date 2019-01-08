function mtm_gc_controller = estimate_gc_params(ARM_NAME,SN)
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Copyright (c)  2018, The Chinese University of Hong Kong
%  This software is provided "as is" under BSD License, with
%  no warranty. The complete license can be found in LICENSE

    % argument input type checking
    arguement_checking(ARM_NAME,SN)

    disp('This software is being developed by Biorobotics and Medical Technology Group of The Chinese University of Hong Kong (CUHK).');
    disp('Author(s):  Hongbin LIN, Vincent Hui, Samuel Au');

    output_file_str = wizard_config_dataCollection(ARM_NAME, SN);
    output_file_str = dataCollection(output_file_str);
    [output_file_str, is_gc_test_pass] = mlse(output_file_str);
    if console_inquire(is_gc_test_pass)
        mtm_gc_controller = [];
        return
    end
    mtm_gc_controller = gc_controller(output_file_str);

end

function arguement_checking(ARM_NAME,SN)
    % Argument Checking
    if~ischar(ARM_NAME)| ~ischar(SN)
        error('Both arguments ''ARM_NAME'' and ''SN'' should be string objects');
    end
    if~(strcmp(ARM_NAME,'MTML') | strcmp(ARM_NAME,'MTMR') )
        error(['Input of argument ''ARM_NAME''= %s is incorrect, you should input one of the strings ',...
               '[''MTML'',''MTMR'']'], ARM_NAME);
    end
end

function is_exist_program = console_inquire(is_gc_test_pass)
    disp(sprintf('Estimation step has finished, do you want to continue?'))
    if is_gc_test_pass
        disp(sprintf('GC test status: passed!'))
    else
        disp(sprintf('GC test status: not passed! It might damage your device'))
    end
    disp(sprintf('yes and start gc controller [y]'))
    disp(sprintf('no and exist the program [n]:'))
    input_str = '';
    while (~strcmp(input_str,'y') && ~strcmp(input_str,'n'))
        w = waitforbuttonpress;
        if w
            input_str = get(gcf, 'CurrentCharacter');
        end
    end
    if strcmp(input_str,'y')
        is_exist_program = false;
    else
        is_exist_program = true;
    end
end
