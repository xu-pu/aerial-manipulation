classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2020 The MathWorks, Inc.
    
    properties (Constant)
        djiros_Activation = 'djiros/Activation'
        djiros_ActivationRequest = 'djiros/ActivationRequest'
        djiros_ActivationResponse = 'djiros/ActivationResponse'
        djiros_CameraAction = 'djiros/CameraAction'
        djiros_CameraActionRequest = 'djiros/CameraActionRequest'
        djiros_CameraActionResponse = 'djiros/CameraActionResponse'
        djiros_DroneArmControl = 'djiros/DroneArmControl'
        djiros_DroneArmControlRequest = 'djiros/DroneArmControlRequest'
        djiros_DroneArmControlResponse = 'djiros/DroneArmControlResponse'
        djiros_DroneTaskControl = 'djiros/DroneTaskControl'
        djiros_DroneTaskControlRequest = 'djiros/DroneTaskControlRequest'
        djiros_DroneTaskControlResponse = 'djiros/DroneTaskControlResponse'
        djiros_Gimbal = 'djiros/Gimbal'
        djiros_MFIOConfig = 'djiros/MFIOConfig'
        djiros_MFIOConfigRequest = 'djiros/MFIOConfigRequest'
        djiros_MFIOConfigResponse = 'djiros/MFIOConfigResponse'
        djiros_MFIOSetValue = 'djiros/MFIOSetValue'
        djiros_MFIOSetValueRequest = 'djiros/MFIOSetValueRequest'
        djiros_MFIOSetValueResponse = 'djiros/MFIOSetValueResponse'
        djiros_MissionHotpointTask = 'djiros/MissionHotpointTask'
        djiros_MissionHpAction = 'djiros/MissionHpAction'
        djiros_MissionHpActionRequest = 'djiros/MissionHpActionRequest'
        djiros_MissionHpActionResponse = 'djiros/MissionHpActionResponse'
        djiros_MissionHpGetInfo = 'djiros/MissionHpGetInfo'
        djiros_MissionHpGetInfoRequest = 'djiros/MissionHpGetInfoRequest'
        djiros_MissionHpGetInfoResponse = 'djiros/MissionHpGetInfoResponse'
        djiros_MissionHpResetYaw = 'djiros/MissionHpResetYaw'
        djiros_MissionHpResetYawRequest = 'djiros/MissionHpResetYawRequest'
        djiros_MissionHpResetYawResponse = 'djiros/MissionHpResetYawResponse'
        djiros_MissionHpUpdateRadius = 'djiros/MissionHpUpdateRadius'
        djiros_MissionHpUpdateRadiusRequest = 'djiros/MissionHpUpdateRadiusRequest'
        djiros_MissionHpUpdateRadiusResponse = 'djiros/MissionHpUpdateRadiusResponse'
        djiros_MissionHpUpdateYawRate = 'djiros/MissionHpUpdateYawRate'
        djiros_MissionHpUpdateYawRateRequest = 'djiros/MissionHpUpdateYawRateRequest'
        djiros_MissionHpUpdateYawRateResponse = 'djiros/MissionHpUpdateYawRateResponse'
        djiros_MissionHpUpload = 'djiros/MissionHpUpload'
        djiros_MissionHpUploadRequest = 'djiros/MissionHpUploadRequest'
        djiros_MissionHpUploadResponse = 'djiros/MissionHpUploadResponse'
        djiros_MissionStatus = 'djiros/MissionStatus'
        djiros_MissionStatusRequest = 'djiros/MissionStatusRequest'
        djiros_MissionStatusResponse = 'djiros/MissionStatusResponse'
        djiros_MissionWaypoint = 'djiros/MissionWaypoint'
        djiros_MissionWaypointAction = 'djiros/MissionWaypointAction'
        djiros_MissionWaypointTask = 'djiros/MissionWaypointTask'
        djiros_MissionWpAction = 'djiros/MissionWpAction'
        djiros_MissionWpActionRequest = 'djiros/MissionWpActionRequest'
        djiros_MissionWpActionResponse = 'djiros/MissionWpActionResponse'
        djiros_MissionWpGetInfo = 'djiros/MissionWpGetInfo'
        djiros_MissionWpGetInfoRequest = 'djiros/MissionWpGetInfoRequest'
        djiros_MissionWpGetInfoResponse = 'djiros/MissionWpGetInfoResponse'
        djiros_MissionWpGetSpeed = 'djiros/MissionWpGetSpeed'
        djiros_MissionWpGetSpeedRequest = 'djiros/MissionWpGetSpeedRequest'
        djiros_MissionWpGetSpeedResponse = 'djiros/MissionWpGetSpeedResponse'
        djiros_MissionWpSetSpeed = 'djiros/MissionWpSetSpeed'
        djiros_MissionWpSetSpeedRequest = 'djiros/MissionWpSetSpeedRequest'
        djiros_MissionWpSetSpeedResponse = 'djiros/MissionWpSetSpeedResponse'
        djiros_MissionWpUpload = 'djiros/MissionWpUpload'
        djiros_MissionWpUploadRequest = 'djiros/MissionWpUploadRequest'
        djiros_MissionWpUploadResponse = 'djiros/MissionWpUploadResponse'
        djiros_MobileData = 'djiros/MobileData'
        djiros_SDKControlAuthority = 'djiros/SDKControlAuthority'
        djiros_SDKControlAuthorityRequest = 'djiros/SDKControlAuthorityRequest'
        djiros_SDKControlAuthorityResponse = 'djiros/SDKControlAuthorityResponse'
        djiros_SendMobileData = 'djiros/SendMobileData'
        djiros_SendMobileDataRequest = 'djiros/SendMobileDataRequest'
        djiros_SendMobileDataResponse = 'djiros/SendMobileDataResponse'
        djiros_SetHardSync = 'djiros/SetHardSync'
        djiros_SetHardSyncRequest = 'djiros/SetHardSyncRequest'
        djiros_SetHardSyncResponse = 'djiros/SetHardSyncResponse'
        djiros_Waypoint = 'djiros/Waypoint'
        n3ctrl_ControllerDebug = 'n3ctrl/ControllerDebug'
        n3ctrl_N3CtrlState = 'n3ctrl/N3CtrlState'
        quadrotor_msgs_AuxCommand = 'quadrotor_msgs/AuxCommand'
        quadrotor_msgs_Bspline = 'quadrotor_msgs/Bspline'
        quadrotor_msgs_Corrections = 'quadrotor_msgs/Corrections'
        quadrotor_msgs_Float64Stamped = 'quadrotor_msgs/Float64Stamped'
        quadrotor_msgs_Gains = 'quadrotor_msgs/Gains'
        quadrotor_msgs_Odometry = 'quadrotor_msgs/Odometry'
        quadrotor_msgs_OptimalTimeAllocator = 'quadrotor_msgs/OptimalTimeAllocator'
        quadrotor_msgs_OutputData = 'quadrotor_msgs/OutputData'
        quadrotor_msgs_PPROutputData = 'quadrotor_msgs/PPROutputData'
        quadrotor_msgs_PolynomialTrajectory = 'quadrotor_msgs/PolynomialTrajectory'
        quadrotor_msgs_PositionCommand = 'quadrotor_msgs/PositionCommand'
        quadrotor_msgs_PositionCommand_back = 'quadrotor_msgs/PositionCommand_back'
        quadrotor_msgs_Replan = 'quadrotor_msgs/Replan'
        quadrotor_msgs_ReplanCheck = 'quadrotor_msgs/ReplanCheck'
        quadrotor_msgs_SO3Command = 'quadrotor_msgs/SO3Command'
        quadrotor_msgs_Serial = 'quadrotor_msgs/Serial'
        quadrotor_msgs_SpatialTemporalTrajectory = 'quadrotor_msgs/SpatialTemporalTrajectory'
        quadrotor_msgs_StatusData = 'quadrotor_msgs/StatusData'
        quadrotor_msgs_SwarmCommand = 'quadrotor_msgs/SwarmCommand'
        quadrotor_msgs_SwarmInfo = 'quadrotor_msgs/SwarmInfo'
        quadrotor_msgs_SwarmOdometry = 'quadrotor_msgs/SwarmOdometry'
        quadrotor_msgs_TRPYCommand = 'quadrotor_msgs/TRPYCommand'
        quadrotor_msgs_TrajectoryMatrix = 'quadrotor_msgs/TrajectoryMatrix'
        rnw_msgs_ConeState = 'rnw_msgs/ConeState'
        rnw_msgs_GripState = 'rnw_msgs/GripState'
        rnw_msgs_RockingCmd = 'rnw_msgs/RockingCmd'
        rnw_msgs_WalkingState = 'rnw_msgs/WalkingState'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(78, 1);
                msgList{1} = 'djiros/ActivationRequest';
                msgList{2} = 'djiros/ActivationResponse';
                msgList{3} = 'djiros/CameraActionRequest';
                msgList{4} = 'djiros/CameraActionResponse';
                msgList{5} = 'djiros/DroneArmControlRequest';
                msgList{6} = 'djiros/DroneArmControlResponse';
                msgList{7} = 'djiros/DroneTaskControlRequest';
                msgList{8} = 'djiros/DroneTaskControlResponse';
                msgList{9} = 'djiros/Gimbal';
                msgList{10} = 'djiros/MFIOConfigRequest';
                msgList{11} = 'djiros/MFIOConfigResponse';
                msgList{12} = 'djiros/MFIOSetValueRequest';
                msgList{13} = 'djiros/MFIOSetValueResponse';
                msgList{14} = 'djiros/MissionHotpointTask';
                msgList{15} = 'djiros/MissionHpActionRequest';
                msgList{16} = 'djiros/MissionHpActionResponse';
                msgList{17} = 'djiros/MissionHpGetInfoRequest';
                msgList{18} = 'djiros/MissionHpGetInfoResponse';
                msgList{19} = 'djiros/MissionHpResetYawRequest';
                msgList{20} = 'djiros/MissionHpResetYawResponse';
                msgList{21} = 'djiros/MissionHpUpdateRadiusRequest';
                msgList{22} = 'djiros/MissionHpUpdateRadiusResponse';
                msgList{23} = 'djiros/MissionHpUpdateYawRateRequest';
                msgList{24} = 'djiros/MissionHpUpdateYawRateResponse';
                msgList{25} = 'djiros/MissionHpUploadRequest';
                msgList{26} = 'djiros/MissionHpUploadResponse';
                msgList{27} = 'djiros/MissionStatusRequest';
                msgList{28} = 'djiros/MissionStatusResponse';
                msgList{29} = 'djiros/MissionWaypoint';
                msgList{30} = 'djiros/MissionWaypointAction';
                msgList{31} = 'djiros/MissionWaypointTask';
                msgList{32} = 'djiros/MissionWpActionRequest';
                msgList{33} = 'djiros/MissionWpActionResponse';
                msgList{34} = 'djiros/MissionWpGetInfoRequest';
                msgList{35} = 'djiros/MissionWpGetInfoResponse';
                msgList{36} = 'djiros/MissionWpGetSpeedRequest';
                msgList{37} = 'djiros/MissionWpGetSpeedResponse';
                msgList{38} = 'djiros/MissionWpSetSpeedRequest';
                msgList{39} = 'djiros/MissionWpSetSpeedResponse';
                msgList{40} = 'djiros/MissionWpUploadRequest';
                msgList{41} = 'djiros/MissionWpUploadResponse';
                msgList{42} = 'djiros/MobileData';
                msgList{43} = 'djiros/SDKControlAuthorityRequest';
                msgList{44} = 'djiros/SDKControlAuthorityResponse';
                msgList{45} = 'djiros/SendMobileDataRequest';
                msgList{46} = 'djiros/SendMobileDataResponse';
                msgList{47} = 'djiros/SetHardSyncRequest';
                msgList{48} = 'djiros/SetHardSyncResponse';
                msgList{49} = 'djiros/Waypoint';
                msgList{50} = 'n3ctrl/ControllerDebug';
                msgList{51} = 'n3ctrl/N3CtrlState';
                msgList{52} = 'quadrotor_msgs/AuxCommand';
                msgList{53} = 'quadrotor_msgs/Bspline';
                msgList{54} = 'quadrotor_msgs/Corrections';
                msgList{55} = 'quadrotor_msgs/Float64Stamped';
                msgList{56} = 'quadrotor_msgs/Gains';
                msgList{57} = 'quadrotor_msgs/Odometry';
                msgList{58} = 'quadrotor_msgs/OptimalTimeAllocator';
                msgList{59} = 'quadrotor_msgs/OutputData';
                msgList{60} = 'quadrotor_msgs/PPROutputData';
                msgList{61} = 'quadrotor_msgs/PolynomialTrajectory';
                msgList{62} = 'quadrotor_msgs/PositionCommand';
                msgList{63} = 'quadrotor_msgs/PositionCommand_back';
                msgList{64} = 'quadrotor_msgs/Replan';
                msgList{65} = 'quadrotor_msgs/ReplanCheck';
                msgList{66} = 'quadrotor_msgs/SO3Command';
                msgList{67} = 'quadrotor_msgs/Serial';
                msgList{68} = 'quadrotor_msgs/SpatialTemporalTrajectory';
                msgList{69} = 'quadrotor_msgs/StatusData';
                msgList{70} = 'quadrotor_msgs/SwarmCommand';
                msgList{71} = 'quadrotor_msgs/SwarmInfo';
                msgList{72} = 'quadrotor_msgs/SwarmOdometry';
                msgList{73} = 'quadrotor_msgs/TRPYCommand';
                msgList{74} = 'quadrotor_msgs/TrajectoryMatrix';
                msgList{75} = 'rnw_msgs/ConeState';
                msgList{76} = 'rnw_msgs/GripState';
                msgList{77} = 'rnw_msgs/RockingCmd';
                msgList{78} = 'rnw_msgs/WalkingState';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(21, 1);
                svcList{1} = 'djiros/Activation';
                svcList{2} = 'djiros/CameraAction';
                svcList{3} = 'djiros/DroneArmControl';
                svcList{4} = 'djiros/DroneTaskControl';
                svcList{5} = 'djiros/MFIOConfig';
                svcList{6} = 'djiros/MFIOSetValue';
                svcList{7} = 'djiros/MissionHpAction';
                svcList{8} = 'djiros/MissionHpGetInfo';
                svcList{9} = 'djiros/MissionHpResetYaw';
                svcList{10} = 'djiros/MissionHpUpdateRadius';
                svcList{11} = 'djiros/MissionHpUpdateYawRate';
                svcList{12} = 'djiros/MissionHpUpload';
                svcList{13} = 'djiros/MissionStatus';
                svcList{14} = 'djiros/MissionWpAction';
                svcList{15} = 'djiros/MissionWpGetInfo';
                svcList{16} = 'djiros/MissionWpGetSpeed';
                svcList{17} = 'djiros/MissionWpSetSpeed';
                svcList{18} = 'djiros/MissionWpUpload';
                svcList{19} = 'djiros/SDKControlAuthority';
                svcList{20} = 'djiros/SendMobileData';
                svcList{21} = 'djiros/SetHardSync';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
