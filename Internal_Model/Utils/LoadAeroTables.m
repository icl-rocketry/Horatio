function [aerosplinefits] = LoadAeroTables()
% Creates spline fits through data tables to make them C1 continuous
    % ROCKET
    static_table_raw = readtable("B0000.xlsx");
    dynamic_table_raw = readtable("xpitch_B0000.xlsx");
    static_table_raw = static_table_raw{:, 2:19};
    dynamic_table_raw = dynamic_table_raw{:, 2:19};

    % array definitions:
    % Dynamic derivatives 
    M_dynamic = dynamic_table_raw(:, 1);
    alpha_dynamic = dynamic_table_raw(:, 2);
    CA_dynamic = dynamic_table_raw(:, 3);
    CN_dynamic = dynamic_table_raw(:, 7);
    CMy_dynamic = dynamic_table_raw(:, 11);

    % static derivatives
    M_static = static_table_raw(:, 1);
    alpha_static = static_table_raw(:, 2);
    CA_static = static_table_raw(:, 3);
    CN_static = static_table_raw(:, 7);
    CMy_static = static_table_raw(:, 11);

    % dynamics splines
    x = [M_dynamic, alpha_dynamic];
    CA_xpitch_spline = tpaps(x', CA_dynamic', 0.8);
    CN_xpitch_spline = tpaps(x', CN_dynamic', 0.8);
    CMy_xpitch_spline = tpaps(x', CMy_dynamic', 0.8);

    % static splines
    x = [M_static, alpha_static];
    CA_static_spline = tpaps(x', CA_static', 0.8);
    CN_static_spline = tpaps(x', CN_static', 0.8);
    CMy_static_spline = tpaps(x', CMy_static', 0.8);

    % gridfin tables
    CL_alpha_table = readmatrix("honeycomb-clalpha.xlsx");
    CD_alpha_table = readmatrix("honeycomb-cdalpha.xlsx");
    
    % isolate alpha > 0 to allow mirroring
    CL_positive = CL_alpha_table(CL_alpha_table(:,1) >= 0, :);
    CD_positive = CD_alpha_table(CD_alpha_table(:,1) >= 0, :);

    % create mirror
    Clalpha_flip = -flip(CL_positive(2:end, 1));
    Cdalpha_flip = -flip(CD_positive(2:end, 1));
    
    % add mirror onto original array
    Clalphaextended = [Clalpha_flip; CL_positive(:,1)];
    Cdalphaextended = [Cdalpha_flip; CD_positive(:,1)];
    
    % lift is an odd function so negate 
    Clextended = [-flip(CL_positive(2:end, 2)); CL_positive(:,2)];
    
    % drag is an even function so leave as is
    Cdextended = [flip(CD_positive(2:end, 2)); CD_positive(:,2)];
    
    % create spline fits through data
    CLfit_spline = spline(Clalphaextended, Clextended);
    CDfit_spline = spline(Cdalphaextended, Cdextended);

    % create struct containing spline fits
    % [CA_xpitch_spline, CN_xpitch_spline, CMy_xpitch_spline, CA_static_spline, CN_static_spline, CMy_static_spline, CLfit, CDfit]
    aerosplinefits = struct("CA_dynamic", CA_xpitch_spline, ...
        "CN_dynamic", CN_xpitch_spline, ...
        "CM_pitch_dynamic", CMy_xpitch_spline, ...
        "CA_static", CA_static_spline, ...
        "CN_static", CN_static_spline, ...
        "CM_pitch_static", CMy_static_spline, ...
        "CLfit", CLfit_spline, ...
        "CDfit", CDfit_spline); 
end