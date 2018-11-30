function bool_Regressor_Mat = analytical_bool_regressor_mat()
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Copyright (c)  2018, The Chinese University of Hong Kong
%  This software is provided "as is" under BSD License, with
%  no warranty. The complete license can be found in LICENSE

    % bool_Regressor_Mat, if Regressor(i,j)~=Null, bool_Regressor_Mat=1; else bool_Regressor_Mat=0;
    % Hard code, computed by simbolic derivation
    bool_Regressor_Mat = zeros(7,70);
    bool_Regressor_Mat(2,1:2) = 1;
    bool_Regressor_Mat(2:3,3:4) = 1;
    bool_Regressor_Mat(2:4,5:6) = 1;
    bool_Regressor_Mat(2:5,7:8) = 1;
    bool_Regressor_Mat(2:6,9:10) = 1;
    bool_Regressor_Mat(1,11:15) = 1;
    bool_Regressor_Mat(2,16:20) = 1;
    bool_Regressor_Mat(3,21:25) = 1;
    bool_Regressor_Mat(4,26:30) = 1;
    bool_Regressor_Mat(5,31:35) = 1;
    bool_Regressor_Mat(6,36:40) = 1;

    bool_Regressor_Mat(1,41:45) = 1;
    bool_Regressor_Mat(2,46:50) = 1;
    bool_Regressor_Mat(3,51:55) = 1;
    bool_Regressor_Mat(4,56:60) = 1;
    bool_Regressor_Mat(5,61:65) = 1;
    bool_Regressor_Mat(6,66:70) = 1;
end
