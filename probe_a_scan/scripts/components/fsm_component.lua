require "rttlib"
require "rfsm"
require "rfsm_rtt"
require "rfsmpp"
require "rfsm_timeevent"
msgs=require("messages")
errmsg ,infomsg =msgs.err ,msgs.info

gs=rtt.provides()
ros = gs:provides("ros")

if rtt then
   require "rttlib"
   gettime = rtt.getTime
   print("using RTT timing services")
else
   if pcall(require, "rtp") then
      function gettime() return rtp.clock.gettime("CLOCK_MONOTONIC") end
      print("using rtp timing services")
   else
      print("falling back on low resolution Lua time")
      function gettime() return os.time(), 0 end
   end
end

rfsm_timeevent.set_gettime_hook(gettime)


function loadrequire(module)
    local function requiref(module)
        require(module)
    end
    res = pcall(requiref,module)
    if not(res) then
        print('cannot find  ' .. module)
        return false
        else return true
    end
end



local tc=rtt.getTC();
local fsm
local fqn_out, events

state_port = rtt.OutputPort("std_msgs.String", "state_port")
tc:addPort(state_port)
events = rtt.InputPort("string")
tc:addEventPort(events, "events", "rFSM event input port")

state_machine_prop=rtt.Property("string","state_machine","file with state machine to execute")
tc:addProperty(state_machine_prop)
add_code_prop=rtt.Property("string","additional_code","file with addittional code to execute")
tc:addProperty(add_code_prop)

viz_on_prop=rtt.Property("bool","viz_on","save to file statemachine image")
tc:addProperty(viz_on_prop)
print_transition_prop=rtt.Property("bool","print_transition","print transitions")
tc:addProperty(print_transition_prop)

-- defalut values
viz_on_prop:set(false) 
print_transition_prop:set(true)


function errmsg(...)   msgs.err("fsmMsg:\t",...)  end
function infomsg(...)  msgs.info("fsmMsg:\t",...) end
function warnmsg(...)  msgs.warn("fsmMsg:\t",...) end



function file_exists(name)
   local f=io.open(name,"r")
   if f~=nil then io.close(f) return true else return false end
end

function my_gen_write_fqn(port)
   assert(port:info().type==rtt.Variable('std_msgs.String'):getType(), "gen_write_fqn: port must be of type std_msgs.String")

   local act_fqn = ""
   local out_dsb = rtt.Variable.new('std_msgs.String')
   port:write(out_dsb,"<none>") -- initial val

   return function (fsm)
             local actl = fsm._act_leaf
             if not actl or act_fqn == actl._fqn then return end
             act_fqn = actl._fqn
             out_dsb.data=act_fqn
             port:write(out_dsb,act_fqt)
          end
end


function configureHook()

   -- load state machine

   -- load extra definition

   if  #add_code_prop:get()==0 then
        infomsg('add_code_prop not set')
   else
        if file_exists(add_code_prop:get()) then
            extra=assert(loadfile(add_code_prop:get()))
            extra()
        else
            errmsg("file " .. add_code_prop:get() .. " do not exists" )
            return false
        end
   end
   fsm = rfsm.init(rfsm.load(state_machine_prop:get()) )

   -- enable state entry and exit dbg output
   if print_transition_prop:get() then
   		fsm.dbg=rfsmpp.gen_dbgcolor("fsm", 
                   { STATE_ENTER=true, STATE_EXIT=true}, 
                   false)
 	 end
 	 ok=true
 	 if viz_on_prop:get() then
   		ok=ok and loadrequire("rfsm2uml")
			ok=ok and loadrequire("rfsm2tree")
			if not ok then
				rtt.logl('Warning', "cannot find modules 'rfsm2uml' or 'rfsm2tree'. State machine will not be printed")
				viz_on_prop:set(false)
			end
 	 end
 	 
 	 
   -- redirect rFSM output to rtt log
   fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
   fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
   fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end

   fsm.getevents = rfsm_rtt.gen_read_str_events(events)


   -- optional: create a string port to which the currently active
   -- state of the FSM will be written. gen_write_fqn generates a
   -- function suitable to be added to the rFSM step hook to do this.
--   fqn_out = rtt.OutputPort("string")
--   tc:addPort(fqn_out, "rFSM_cur_fqn", "current active rFSM state")
   rfsm.post_step_hook_add(fsm, my_gen_write_fqn(state_port))
   return true
end
 
function updateHook()
 rfsm.run(fsm) 
 if viz_on_prop:get() then
  rfsm2uml.rfsm2dot(fsm, "fsm-uml.dot")
 --	print("generating umldot... ", rfsm2uml.rfsm2dot(fsm, "fsm-uml.dot"))
 	os.execute("dot fsm-uml.dot -Tpdf -O")
 end
end
 
function cleanupHook()
   -- cleanup the created ports.
   rttlib.tc_cleanup()
end


