-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- KU Leuven 2020
-- ==============================================================================

require "rttlib"
require "rttros"
require "deployer_utils"
 
-- ====================================== User Parameters =========================================
robot_name = "kuka_lwr"
freq = 1000

-- ====================================== Standard deployment stuff =========================================
rtt.setLogLevel("Warning")

gs=rtt.provides()
tc=rtt.getTC()
if tc:getName() == "lua" then
  depl=tc:getPeer("Deployer")
elseif tc:getName() == "Deployer" then
  depl=tc
end
depl:import("rtt_ros")
ros = gs:provides("ros")
ros:import("etasl_rtt")
ros:import("rtt_rospack")
rttlib.color = true

etasl_application_dir = rtt.provides("ros"):find("probe_a_scan")
robot_def_dir = etasl_application_dir .. "/scripts/etasl/robot_def"
json_file_poses=etasl_application_dir.. "/poses/probe_poses.json"

-- The following will make run always in simulation, unless you provide "deploy_robot as the first argument"
-- Example to run in real robot: rttlua -i deploy_general.lua "real_robot"
-- TO RUN THE SOEM MASTER rttlua -i deploy_general.lua "real_robot" "sensor_attached"


if arg[1]== "real_robot" or arg[2] == "real_robot" then
    print("real robot option")
  simulation = false
else
  simulation = true
end
 
cp=rtt.Variable("ConnPolicy")

-- ====================================== Robot Hardware definition =========================================
depl_robot_file,robot_etasl_dir = determine_robot(robot_name)
robot = require(depl_robot_file)

-- ====================================== eTaSL components ===================================
-- ====================================== Solver
ros:import("etasl_solver_qpoases")
depl:loadComponent("solver","etasl_solver_qpoases")
solver = depl:getPeer("solver")

-- ====================================== jointstate I/O factories
ros:import("etasl_iohandler_jointstate")
depl:loadComponent("jointstate","Etasl_IOHandler_Jointstate")
jointstate = depl:getPeer("jointstate")

-- ====================================== eTaSL core
ros:import("etasl_rtt")
depl:loadComponent("etaslcore", "etasl_rtt")
-- create LuaComponents
etaslcore = depl:getPeer("etaslcore")
depl:connectPeers("etaslcore","solver")
depl:connectPeers("etaslcore","jointstate")

-- ====================================== Output ports task
etaslcore:add_etaslvar_outputport("tf_pose","Executed pose of the task frame",s{"x_tf","y_tf","z_tf","roll_tf","pitch_tf","yaw_tf"})
etaslcore:add_etaslvar_inputport("force_sensor","Force read from force sensor",s{"Fx","Fy","Fz","Tx","Ty","Tz"}, d{0,0,0,0,0,0})
etaslcore:add_etaslvar_outputport("path_coordinate","Information of the path coordinate",s{"s"}) 
etaslcore:add_etaslvar_inputport("tf_pose_new","Executed pose of the task frame",s{"f.x","f.y","f.z","f.r","f.p","f.yaw"},d{0,0,0,0,0,0})

-- ====================================== Configure eTaSL ports for the robot
robot.create_etasl_ports(etaslcore,jointstate)

depl:setActivity("etaslcore", 1/freq, 50, rtt.globals.ORO_SCHED_RT)

-- ====================================== simulation
if simulation then
-- deploy simulated robot:
    depl:loadComponent("simrobot", "OCL::LuaComponent")
    simrobot = depl:getPeer("simrobot")
    simrobot:exec_file(etasl_application_dir.."/scripts/components/simple_robot_sim.lua")
    init_jnts = robot.initial_joints_states()
    simrobot:getProperty("initial_position"):set( init_jnts )
    depl:setActivity("simrobot", 1/freq, 50, rtt.globals.ORO_SCHED_RT)
    depl:connect("etaslcore.jointvel","simrobot.jointvel",cp )
    depl:connect("simrobot.jointpos","etaslcore.jointpos",cp )
    simrobot:configure()
    simrobot:start()

else
-- ====================================== Real robot
    robot.deploy_driver(1/freq)
end

-- ====================================== Reporter =========================================
function exists(file)
  local ok, err, code = os.rename(file, file)
  if not ok then
if code == 13 then
  -- Permission denied, but it exists
  return true
end
  end
  return ok
end

function isdir(path)
  -- "/" works on both Unix and Windows
  return exists(path.."/")
end

date = os.date("%Y_%m_%d")
tmstamp = os.date("%Y_%m_%d_%H_%M_%S")
--dir_name = "/reports_from_" .. date
--dir_path = etasl_application_dir .. "/reports/all_data" .. dir_name

--file_name = "/report_of_" .. tmstamp ..'.dat'
dir_path = etasl_application_dir .. "/reports"
file_name='/report.dat'


if not isdir(dir_path) then
  os.execute("mkdir -p " .. dir_path )
  print('Directory ' .. dir_name .. ' created')
end

depl:loadComponent("Reporter","OCL::FileReporting")
reporter=depl:getPeer("Reporter")
depl:connectPeers("etaslcore","Reporter")
reporter:reportPort("etaslcore","jointvel")
reporter:reportPort("etaslcore","tf_pose")
reporter:reportPort("etaslcore","path_coordinate")
reporter:getProperty("ReportFile"):set(dir_path .. file_name)

-- ====================================== Supervisor =========================================
depl:loadComponent("Supervisor", "OCL::LuaComponent")
sup = depl:getPeer("Supervisor")

define_property( sup, "simulation", "bool", simulation, "Boolean value to set simulation mode" )
define_property( sup, "robot_etasl_dir", "string", robot_etasl_dir, "Directory of the etasl robot definition" )
define_property( sup, "depl_robot_file", "string", depl_robot_file, "Directory of the file containing deployment of the robot" )

sup:exec_file(etasl_application_dir.."/scripts/components/fsm_component.lua")


sup:getProperty("state_machine"):set(etasl_application_dir.."/scripts/rfsm/a_mode_fsm.lua")
sup:getProperty("additional_code"):set(etasl_application_dir.."/scripts/rfsm/a_mode_fsm_extra.lua")
sup:getProperty("viz_on"):set(true)
sup:addPeer(depl)
depl:setActivity("Supervisor", 1/freq, 50, rtt.globals.ORO_SCHED_RT) --needed to use the time events of rfsm
sup:configure()
sup:getProperty("pose_file"):set(json_file_poses)
sup:start()
ev = rttlib.port_clone_conn(sup:getPort("events"))
---- EVENT ECHO ----

depl:loadComponent("eventEcho", "OCL::LuaComponent")
eventEcho = depl:getPeer("eventEcho")
eventEcho:exec_file( rtt.provides("ros"):find("python_gui").."/lua_components/signal_echo.lua")
eventEcho:configure()
--stream data to component
cp_ros=rtt.Variable("ConnPolicy")
cp_ros.transport=3
cp_ros.name_id="/events"
depl:stream("eventEcho.event_in",cp_ros)
--here, connect "eventEcho.event_out" to some other component
depl:connect("Supervisor.events","eventEcho.event_out",cp)
eventEcho:start()

------ POSE SAVING COMPONENT -----------------
depl:loadComponent("pose_manager", "OCL::LuaComponent")
pose_manager = depl:getPeer("pose_manager")
pose_manager:exec_file(etasl_application_dir.."/scripts/components/probe_pos_manager.lua")

pose_manager:getProperty("data_file"):set( json_file_poses)
depl:setActivity("pose_manager", 0, 50, rtt.globals.ORO_SCHED_OTHER)
depl:connect("etaslcore.tf_pose","pose_manager.tf_pose",cp )
pose_manager:configure()
depl:connect("pose_manager.commands","eventEcho.event_out",cp)
pose_manager:start()
cmd = rttlib.port_clone_conn(pose_manager:getPort("commands"))

-- connect ports:
if not simulation then
  robot.connect_ports_driver(etaslcore,1/freq)
end
depl:connect("etaslcore.eventPort","Supervisor.events",cp)
depl:stream("etaslcore.joint_state", ros:topic("/joint_states"))
