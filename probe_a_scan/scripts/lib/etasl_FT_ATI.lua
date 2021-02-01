-- ==============================================================================
-- Main deploy the driver of the force sensor
-- KU Leuven 2020
-- =============================================================================
local M = {}

function FTsensor_deployer(comp,timefreq)
    etasl_application_dir = rtt.provides("ros"):find("probe_a_scan")
    dofile( etasl_application_dir .. "/configuration/SoemMasterComponent_properties.lua" )
    dofile( etasl_application_dir .. "/configuration/AtiIface_properties.lua" )

    compname = comp:getName()
    
    depl:import("rtt_ros")
    ros:import("rtt_tf")
    depl:import( "rtt_std_msgs" )
    depl:import( "rtt_rosnode" )
    depl:import( "rtt_rospack" )
    depl:import( "rtt_roscomm" )
    depl:import( "rtt_rosclock" )
    depl:import( "soem_master" )
    depl:import( "soem_beckhoff_drivers" )
    depl:import( "ati_iface" ) 


    depl:loadComponent( "soem_master", "soem_master::SoemMasterComponent" )
    depl:addPeer( "soem_master","Deployer" )
    soem_master = depl:getPeer( "soem_master" )

    depl:loadComponent( "AtiIface", "ati::AtiIface" )
    depl:addPeer( "AtiIface", "Deployer" )
    ati_iface = depl:getPeer( "AtiIface" ) 

    depl:loadComponent("tf","rtt_tf::RTT_TF")
    depl:connectServices("AtiIface","tf")
    tf = depl:getPeer("tf")


    depl:setActivity( soem_master:getName( ), timefreq, 50, rtt.globals.ORO_SCHED_RT)
    soem_master:getProperty( "ifname" ):set( ifname )

    depl:setActivity( ati_iface:getName( ), timefreq, 50, rtt.globals.ORO_SCHED_RT)
    ati_iface:getProperty( "scale_matrix" ):set( scale_matrix )
    ati_iface:getProperty( "calibration_matrix" ):set( calibration_matrix )
    ati_iface:getProperty( "lowpass_filter_on" ):set( lowpass_filter_on )
    ati_iface:getProperty( "filter_bandwidth" ):set( filter_bandwidth )
    ati_iface:getProperty( "filter_order" ):set( filter_order )
    ati_iface:getProperty( "compensate_deadband" ):set( compensate_deadband )
    ati_iface:getProperty( "f_deadband" ):set( f_deadband )
    ati_iface:getProperty( "m_deadband" ):set( m_deadband )


    soem_master:configure( )
    if not soem_master:isConfigured( ) then -- Switch the interface names in case soem_master doesn't configure.
        soem_master:getProperty( "ifname" ):set( ifname2 )
        soem_master:configure( )
    end
    ati_iface:configure()
    tf:configure()
 
    soem_master:start( )
    ati_iface:start( )
    tf:start()



    -- ====================================== Configure eTaSL ports of F/T sensor
    wr=s{"global.Fx","global.Fy","global.Fz","global.Tx","global.Ty","global.Tz"}
    -- F_w=s{"global.W_Fx","global.W_Fy","global.W_Fz","global.W_Tx","global.W_Ty","global.W_Tz"}
    comp:add_etaslvar_inputport("wrench_array","Vector with wrench values",wr,d{})
    -- comp:add_etaslvar_inputport("force_weights","Vector with constraint weights",F_w,d{})
    -- comp:add_etaslvar_deriv_inputport("force_weights_derivatives", "Vector with constraint weights derivatives", F_w)
    -- comp:add_etaslvar_frame_outputport("FT_frame","Frame of the load cell for the force component","FT_frame")


    cp = rtt.Variable( "ConnPolicy" )
    depl:connect( soem_master:getName( )..".Slave_1002.values", ati_iface:getName( )..".in_ai1_port", cp )
    depl:connect( soem_master:getName( )..".Slave_1003.values", ati_iface:getName( )..".in_ai2_port", cp )
    depl:connect("AtiIface.out_W_ati",compname..".wrench_array",cp )
    depl:stream("AtiIface.out_W_ati", rtt.provides( "ros" ):topic("/wrench_raw") )
     


end


-- export functions
M.FTsensor_deployer = FTsensor_deployer

return M
