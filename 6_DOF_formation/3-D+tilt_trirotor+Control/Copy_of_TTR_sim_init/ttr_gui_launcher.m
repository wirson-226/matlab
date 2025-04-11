% TTR-VTOL Multi-Agent GUI Launcher with dynamic layout
function ttr_gui_launcher()
    % 创建主界面窗口和布局容器
    fig = uifigure('Name','TTR-VTOL Sim Launcher','Position',[1000 300 420 450]);
    gl = uigridlayout(fig, [8, 2]);
    gl.RowHeight = {40, 40, 40, 40, 40, 60, 60, '1x'};
    gl.ColumnWidth = {'1x','2x'};

    % 组件控件
    % uilabel(gl, 'Text','Number of Agents:');
    % numAgentBox = uieditfield(gl,'numeric','Value',6);

    uilabel(gl, 'Text','Trajectory Type:');
    trajPopup = uidropdown(gl,'Items',{'VTOL cruise','VTOL global','Helix','4-Point Step'});

    uilabel(gl, 'Text','Controller:');
    ctrlPopup = uidropdown(gl,'Items',{'Global Controller','Copter Done','Cruise'});

    realtimeCheck = uicheckbox(gl,'Text','Real-time Display','Value',true);
    dummy = uilabel(gl,'Text',''); dummy.Layout.Row = 4; dummy.Layout.Column = 1;

    uilabel(gl, 'Text','Simulation Time [s]:');
    timeBox = uieditfield(gl,'numeric','Value',5);

    startButton = uibutton(gl,'Text','Start Sim','ButtonPushedFcn',@(btn,event) startSimulation());
    startButton.Layout.Row = 6;
    startButton.Layout.Column = [1 2];
    
    stopButton = uibutton(gl,'Text','Stop Sim','ButtonPushedFcn',@(btn,event) stopSimulation());
    stopButton.Layout.Row = 7;
    stopButton.Layout.Column = [1 2];
    stopButton.BackgroundColor = [0.8 0.2 0.2];  % Red color for stop button

    % 启动仿真函数
    function startSimulation()
        % num_agents = numAgentBox.Value;
        traj_choice = trajPopup.Value;
        ctrl_choice = ctrlPopup.Value;
        real_time = realtimeCheck.Value;
        sim_time = timeBox.Value;

        traj_map = {@traj_vtol_cruise, @traj_vtol_global, @traj_helix, @traj_4point_step};
        trajhandle = traj_map{find(strcmp(trajPopup.Items, traj_choice))};

        ctrl_map = {@global_controller_main_testing, @copter_controller_Done, @cruise_controller_A};
        controlhandle = ctrl_map{find(strcmp(ctrlPopup.Items, ctrl_choice))};

        assignin('base','real_time',real_time);
        assignin('base','max_time',sim_time);

        try
            [~, ~] = simulation_3d_ttr(trajhandle, controlhandle);
        catch ME
            uialert(fig,['Simulation failed: ' ME.message],'Simulation Error');
        end
    end

    % 停止仿真函数
    function stopSimulation()
        % 设置停止标志到基础工作区
        assignin('base','stop_simulation',true);
        disp('Simulation stop requested...');
        uialert(fig,'Simulation stop requested. The simulation will terminate after completing current step.','Stop Command');
    end
end