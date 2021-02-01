-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Definition of the state machine
-- KU Leuven 2020
-- ==============================================================================

require("rtt")
require("rttlib")
require("rfsm_timeevent")
require("kdlutils")
require "rttros" 
require "deployer_utils"
gettime = rtt.getTime
rfsm_timeevent.set_gettime_hook(gettime)

tc          = rtt.getTC()
depl        = tc:getPeer("Deployer")
etaslcore   = depl:getPeer("etaslcore")
reporter    = depl:getPeer("Reporter")
solver      = depl:getPeer("solver")

simulation = tc:getProperty("simulation"):get()
robot_etasl_dir = tc:getProperty("robot_etasl_dir"):get()
depl_robot_file = tc:getProperty("depl_robot_file"):get()
robot = require(depl_robot_file)
joint_pos = robot.home_joint_positions()

etasl_application_dir = rtt.provides("ros"):find("probe_a_scan")
json_pose_prop=rtt.Property("string","pose_file","file with probing motion json file")
tc:addProperty(json_pose_prop)

JSON = require("JSON")

local function read_to_string(file)
    local f = io.open(file, "rb")
    local content = f:read("*all")
    f:close()
    return content
end


-- =====================seom master + force sensor + components =========================================================
timefreq_sensor = 1/1000
dofile( etasl_application_dir .. "/configuration/SoemMasterComponent_properties.lua" )
dofile( etasl_application_dir .. "/configuration/AtiIface_properties.lua" )
dofile( etasl_application_dir .. "/configuration/Application_properties.lua" )
depl:import( "ati_iface" )
depl:loadComponent( "AtiIface", "ati::AtiIface" )
ati_iface = depl:getPeer( "AtiIface" ) 
depl:import( "force_trigger" )
depl:loadComponent( "ForceTrigger", "forcetrigger::ForceTrigger" ) 
F_trigger = depl:getPeer( "ForceTrigger" )
depl:import( "gui_connector" )
depl:loadComponent( "GuiConnector", "guiconnector::GuiConnector" ) 
gui_connect= depl:getPeer( "GuiConnector" )

depl:setActivity( ati_iface:getName( ), timefreq_sensor, 50, rtt.globals.ORO_SCHED_RT)
ati_iface:getProperty( "scale_matrix" ):set( scale_matrix )
ati_iface:getProperty( "calibration_matrix" ):set( calibration_matrix )
ati_iface:getProperty( "lowpass_filter_on" ):set( lowpass_filter_on )
ati_iface:getProperty( "filter_bandwidth" ):set( filter_bandwidth )
ati_iface:getProperty( "filter_order" ):set( filter_order )
ati_iface:getProperty( "compensate_deadband" ):set( compensate_deadband )
ati_iface:getProperty( "f_deadband" ):set( f_deadband )
ati_iface:getProperty( "m_deadband" ):set( m_deadband )
ati_iface:configure()
depl:setActivity( F_trigger:getName( ), timefreq_sensor, 50, rtt.globals.ORO_SCHED_RT)
F_trigger:getProperty( "force_constraint" ):set( force_constraint )
F_trigger:getProperty( "probe_weight" ):set( probe_weight )
F_trigger:configure( )
depl:setActivity( gui_connect:getName( ), timefreq_sensor, 50, rtt.globals.ORO_SCHED_RT)
gui_connect:configure( )

depl:import( "soem_master" )
depl:import( "soem_beckhoff_drivers" )
depl:loadComponent( "soem_master", "soem_master::SoemMasterComponent" )
soem_master = depl:getPeer( "soem_master" )

depl:setActivity( soem_master:getName( ), timefreq_sensor, 50, rtt.globals.ORO_SCHED_RT) 
soem_master:getProperty( "ifname" ):set( ifname )
soem_master:configure( )
if not soem_master:isConfigured( ) then -- Switch the interface names in case soem_master doesn't configure.
    soem_master:getProperty( "ifname" ):set( ifname2 )
    soem_master:configure( )
end

cp = rtt.Variable( "ConnPolicy" )
depl:connect( soem_master:getName( )..".Slave_1002.values", ati_iface:getName( )..".in_ai1_port", cp )
depl:connect( soem_master:getName( )..".Slave_1003.values", ati_iface:getName( )..".in_ai2_port", cp )
soem_master:start( )

depl:connect(ati_iface:getName( )..".out_W_ati","etaslcore.force_sensor",cp )
depl:stream( ati_iface:getName( )..".out_W_ati" , rtt.provides( "ros" ):topic("/wrench_raw") )
ati_iface:start( ) 

depl:connect( ati_iface:getName( )..".out_W_ati", F_trigger:getName( )..".in_W_ati", cp )
-- depl:stream( F_trigger:getName( )..".in_W_ati" , rtt.provides( "ros" ):topic("/wrench_raw") )
depl:stream( F_trigger:getName( )..".in_fusion_", rtt.provides( "ros" ):topic("/Fusion_track_rpy") )
depl:stream( F_trigger:getName( )..".out_contact", rtt.provides( "ros" ):topic("/contact_message") )
depl:stream( F_trigger:getName( )..".out_net_W", rtt.provides( "ros" ):topic("/wrench_net") )
F_trigger:start( )

depl:stream( gui_connect:getName( )..".in_ifnewpose_message", rtt.provides( "ros" ):topic("/if_new_pose") )
depl:stream( gui_connect:getName( )..".in_pose_message", rtt.provides( "ros" ):topic("/new_pose") )
depl:stream( gui_connect:getName( )..".out_write_finish", rtt.provides( "ros" ):topic("/write_finish") )
depl:stream( gui_connect:getName( )..".out_pose_message", rtt.provides( "ros" ):topic("/out_pose_message") )
gui_connect:start()

-- ==============================================================================  --


function load_probe_poses()
    local js=read_to_string (tc:getProperty("pose_file"):get())
    local t=JSON:decode(js)
    local n_elements=#t.probe_frames
    local endframes={}
    for i=1,n_elements do
        local f=t.probe_frames[i]
        endframes[i]={f.x,f.y,f.z,f.R,f.P,f.Y}
    end
    local f=t.center_frame
    local centerframe={f.x,f.y,f.z,f.R,f.P,f.Y}
    return n_elements, centerframe, endframes
end


function check_new_poses(n_elements) 
	myport = rttlib.port_clone_conn(gui_connect:getPort("out_write_finish")) 
	local fs, data=myport:read() 
    local new_pose_flag = false 
    local new_pose={} 
    if  n_elements <= 6 then
        new_pose_flag =true 
        new_pose[1] = -0.55
    	new_pose[2] = 0.15 + n_elements*0.05
		new_pose[3] = 0.3
		new_pose[4] = 3.44
		new_pose[5] = 0.0  + n_elements*0.05
		new_pose[6] = 3.24
	else 
		new_pose_flag = false
    end 
    return new_pose_flag, new_pose, fs ,data
end


function driver_particularities()
  if robot.robot_name == "franka_panda" and not simulation then
    local panda = depl:getPeer("panda")
    panda:low_level_velocity()
  end

end

function configure_reporter()
    reporter:setPeriod(0.05)
    reporter:configure()
    reporter:start()
end

function idle_config()
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    etaslcore:configure()
    etaslcore:initialize()
    etaslcore:start()
end

function cleanup_controller()
    etaslcore:stop()
    etaslcore:cleanup()
end

function cleanup_controller_emergency()
    if etaslcore:isRunning() then
        cleanup_controller()
    end
end

function moving_joint_space_config()
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_jointspace_trap.lua")
    etaslcore:set_etaslvar("global.maxvel",0.5)
    etaslcore:set_etaslvar("global.maxacc",0.1)
    if robot.is_continuous_joints then --If the function exists it modifies it. Otherwise the defaults are used (non continuous)
            for i=1,#joint_pos do
                etaslcore:set_etaslvar("global.continuous_j"..i,robot.is_continuous_joints()[i])
            end
    end
    for i=1,#joint_pos do
            etaslcore:set_etaslvar("global.end_j"..i,joint_pos[i])
    end
    etaslcore:configure()
    etaslcore:initialize()
    etaslcore:start()
    driver_particularities()
end
function moving_approach_config(middlepose,endpose)
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    print("HERE0")
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_approach.lua")
    print("HERE1")
    etaslcore:set_etaslvar("global.maxvel",0.5)
    etaslcore:set_etaslvar("global.maxacc",0.1)
    print("HERE2")
    endpose = rtt.Variable("KDL.Frame")
    endpose.p:fromtab{X =-0.4 , Y = 0.0, Z = 0.7 }
    endpose.M = rtt.provides("KDL"):provides("Rotation"):RPY(3.14, 0.0 , 3.14)
    
    middlepose = rtt.Variable("KDL.Frame")
    middlepose.p:fromtab{X =-0.2 , Y = 0.0, Z = 0.3 }
    middlepose.M = rtt.provides("KDL"):provides("Rotation"):RPY(3.14, 0.0 , 3.14)
    etaslcore:set_etaslvar_frame("endpose",endpose)
    print("HERE2a")
    etaslcore:set_etaslvar_frame("middlepose",middlepose)
     print("HERE3")
    etaslcore:configure()
    print("HERE1")
    etaslcore:initialize()
     print("HERE2")
    etaslcore:start()
     print("HERE3")
    driver_particularities()
     print("HERE4")
end
function moving_cartesian_frame_absolute_config(endframe)
    local endpose = rtt.Variable("KDL.Frame")
    endpose.p:fromtab{X =endframe[1] ,Y= endframe[2],Z= endframe[3] }
    endpose.M = rtt.provides("KDL"):provides("Rotation"):RPY(endframe[4] , endframe[5], endframe[6] )
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_cartesian_frame_absolute.lua")
    etaslcore:set_etaslvar("global.maxvel",0.5)
    etaslcore:set_etaslvar("global.maxacc",0.1)
    etaslcore:set_etaslvar("global.eq_r",0.08)
    etaslcore:set_etaslvar_frame("global.endpose",endpose)
    etaslcore:configure()
    etaslcore:initialize()
    etaslcore:start()
    driver_particularities()
end

function moving_cartesian_frame_config(delta)
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_cartesian_frame.lua")
    etaslcore:set_etaslvar("global.maxvel",0.5)
    etaslcore:set_etaslvar("global.maxacc",0.1)
    etaslcore:set_etaslvar("global.eq_r",0.08)
    etaslcore:set_etaslvar("global.delta_x",delta[1])
    etaslcore:set_etaslvar("global.delta_y",delta[2])
    etaslcore:set_etaslvar("global.delta_z",delta[3])
    etaslcore:configure()
    etaslcore:initialize()
    etaslcore:start()
    driver_particularities()
end

function moving_cartesian_frame_force_guard_config(delta,max_force)
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_cartesian_frame_force_guarded.lua")
    etaslcore:set_etaslvar("global.maxvel",0.5)
    etaslcore:set_etaslvar("global.maxacc",0.1)
    etaslcore:set_etaslvar("global.eq_r",0.08)
    etaslcore:set_etaslvar("global.delta_x",delta[1])
    etaslcore:set_etaslvar("global.delta_y",delta[2])
    etaslcore:set_etaslvar("global.delta_z",delta[3])
    etaslcore:set_etaslvar("global.max_force",max_force)
    print(max_force)
    etaslcore:configure()
    etaslcore:initialize()
    etaslcore:start()
    driver_particularities()
end

function moving_spline_config()
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_spline.lua")
    etaslcore:set_etaslvar("global.maxvel",0.5)
    etaslcore:set_etaslvar("global.maxacc",0.1)
    etaslcore:configure()
    etaslcore:initialize()
    etaslcore:start()
    driver_particularities()
end

function admittance_config()
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_admittance.lua")
    etaslcore:configure()
    etaslcore:initialize()
    etaslcore:start()
    driver_particularities()
end

function moving_learned_model_config()
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_learned_model.lua")
    etaslcore:set_etaslvar("global.maxvel",0.5)
    etaslcore:set_etaslvar("global.maxacc",0.1)
    etaslcore:set_etaslvar("global.c_tgt_x",0.8)
    etaslcore:set_etaslvar("global.c_tgt_y",0.15)
    etaslcore:set_etaslvar("global.c_tgt_z",0.1)
    etaslcore:configure()
    etaslcore:initialize()
    etaslcore:start()
    driver_particularities()
end

