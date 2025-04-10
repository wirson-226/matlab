% TTR-VTOL Multi-Agent GUI Launcher
function ttr_gui_launcher()
    % 创建主界面窗口
    fig = figure('Name','TTR-VTOL Multi-Agent Sim Launcher','NumberTitle','off',...
        'Position',[500 300 400 350],'Resize','off');

    uicontrol('Style','text','Position',[30 300 140 20],'String','Number of Agents:');
    numAgentBox = uicontrol('Style','edit','Position',[180 300 60 25],'String','6');

    uicontrol('Style','text','Position',[30 260 140 20],'String','Trajectory Type:');
    trajPopup = uicontrol('Style','popupmenu','Position',[180 260 160 25],...
        'String',{'Circle Lift','Line','Helix','4-Point Step'});

    uicontrol('Style','text','Position',[30 220 140 20],'String','Controller:');
    ctrlPopup = uicontrol('Style','popupmenu','Position',[180 220 160 25],...
        'String',{'Global Controller','Copter Done','Cruise'});

    % 是否实时显示
    realtimeCheck = uicontrol('Style','checkbox','Position',[30 180 200 20],...
        'String','Real-time Display','Value',1);

    % 启动按钮
    uicontrol('Style','pushbutton','Position',[150 100 100 40],'String','Start Sim',...
        'Callback',@(src,event) startSimulation());

    function startSimulation()
        % 获取参数
        num_agents = str2double(get(numAgentBox, 'String'));
        traj_choice = get(trajPopup, 'Value');
        ctrl_choice = get(ctrlPopup, 'Value');
        real_time = get(realtimeCheck, 'Value');

        % 映射轨迹函数
        traj_map = {@traj_formation_circle_lift, @traj_line, @traj_helix, @traj_4point_step};
        trajhandle = traj_map{traj_choice};

        % 映射控制器函数
        ctrl_map = {@global_controller_main_testing, @copter_controller_Done, @cruise_controller_needtodo};
        controlhandle = ctrl_map{ctrl_choice};

        assignin('base','real_time',real_time);  % 将参数送入 base workspace

        % 运行仿真
        try
            [~, ~] = simulation_formation_ttr(trajhandle, controlhandle, num_agents);
        catch ME
            errordlg(['Simulation failed: ' ME.message],'Simulation Error');
        end
    end
end
