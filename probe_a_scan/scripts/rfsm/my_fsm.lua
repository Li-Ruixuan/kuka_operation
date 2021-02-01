-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Definition of the state machine
-- KU Leuven 2020
-- ==============================================================================
key = ""
function PrintTable(table , level)
  level = level or 1
  local indent = ""
  for i = 1, level do
    indent = indent.."  "
  end

  if key ~= "" then
    print(indent..key.." ".."=".." ".."{")
  else
    print(indent .. "{")
  end

  key = ""
  for k,v in pairs(table) do
     if type(v) == "table" then
        key = k
        PrintTable(v, level + 1)
     else
        local content = string.format("%s%s = %s", indent .. "  ",tostring(k), tostring(v))
      print(content)  
      end
  end
  print(indent .. "}")

end

return rfsm.state {
   configured = rfsm.state {
      entry=function()
        idle_config()                   
      end,
      exit=function()
            cleanup_controller()
      end,                      
   },

   idle = rfsm.state {
      entry=function()
            idle_config()

      end,
      exit=function()
            cleanup_controller()
      end,
   },

-- velocity profile is created giving as arguments the joint initial postions, the joint desired positions, the maximum velocity 
-- and the maximum acceleration. The total duration is automatically calculated and stored in an expression
   moving_joint_space= rfsm.state {
      entry=function()
            moving_joint_space_config()
      end,
      exit=function()
            cleanup_controller()
      end,
   },
   wait = rfsm.state {
      entry=function()
            idle_config()

      end,
      exit=function()
            cleanup_controller()
      end,
   },
  
-- The motion is defined with respect to the initial pose of the task frame.
-- The position and orientation trajectories are defined to follow a trapezoidal velocity profile parameterized with time. 
-- In case of the orientation, its values corresponds to a linear interpolation in the angle-axis representation.
    move_to_probe=rfsm.state{
        entry=function()
            iteration=1  
            n_elements, centerframe, endframes=load_probe_poses ()
        
        end,
        exit=function()
            cleanup_controller_emergency()
        end,
        moving_cartesian_middle_init = rfsm.state {
            entry=function()
                moving_cartesian_frame_absolute_config(centerframe)
            end,
            exit=function()
                cleanup_controller()
            end,
        },

        moving_cartesian_approach = rfsm.state {
            entry=function() 
                moving_cartesian_frame_absolute_config(endframes[iteration])
            end,
            exit=function()
                cleanup_controller()
            end,
        },
        moving_cartesian_force_in = rfsm.state {
          --here need to put a force control /impedance                                      
            entry=function()
                moving_cartesian_frame_force_guard_config({0,0,0.1},1)
                --configure_reporter()
            end,
            exit=function()
                --reporter:stop()                            
                cleanup_controller()
                
            end,
        },
        measure_data = rfsm.state {
          --here need to put a force control /impedance                                      
            entry=function()
                infomsg("start measure data  for point " .. tostring(iteration))
            end,
            exit=function()
                infomsg("stop measure data  for point " .. tostring(iteration))
            end,
        },
        moving_cartesian_out_contact = rfsm.state {
            entry=function()
                moving_cartesian_frame_config({0,0,-0.15},5)
                --configure_reporter()
            end,
            exit=function()
                --reporter:stop()                            
                cleanup_controller()
                
            end,
        },
                             
        moving_cartesian_middle = rfsm.state {
            entry=function()
                moving_cartesian_frame_absolute_config(centerframe)
            end,
            exit=function()
                cleanup_controller()
            end,
        },
    
        check_if_done = rfsm.state {
            entry =	function(fsm) 
                -- check end condition
                if iteration+1>n_elements then
                    rfsm.send_events(fsm, "e_probe_done")
                    --infomsg("done " ..n_elements .. " measurements" )
                else
                    iteration=iteration+1
                end
            end
            },

            check_if_new_pose = rfsm.state {
            	entry = function() 
            	-----====== comment this part ===============
            	if iteration >= n_elements then
            		new_number = 1+ n_elements
            		new_pose_flag, new_pose, data = check_new_poses (n_elements )
            		if new_pose_flag then
            			n_elements = n_elements + 1
            			endframes[new_number] = new_pose 
            			infomsg("  adding new pose point " .. tostring(endframes[new_number]) ) 
            			infomsg("  adding a new pose point " .. tostring(data)  )
                  PrintTable(endframes[new_number]) 
            		else
            			infomsg("  no new pose after point " .. tostring(iteration)  )
            		end
            	end
            	-----====== comment this part ===============
            	end,
            	exit=function()
             		cleanup_controller()
        		end,

            },

        rfsm.trans {src="initial",  tgt="moving_cartesian_middle_init" },
        rfsm.trans {src="moving_cartesian_middle_init", tgt="moving_cartesian_approach",    events={"e_finished@etaslcore"}},
        rfsm.trans {src="moving_cartesian_approach",    tgt="moving_cartesian_force_in",    events={"e_finished@etaslcore"}},
        rfsm.trans {src="moving_cartesian_force_in",    tgt="measure_data",                 events={"e_finished@etaslcore"}},
        rfsm.trans {src="measure_data",                 tgt="moving_cartesian_out_contact", events={"e_after(2)"}},
        rfsm.trans {src="moving_cartesian_out_contact", tgt="moving_cartesian_middle",      events={"e_finished@etaslcore"}},
        rfsm.trans {src="moving_cartesian_middle",      tgt="check_if_new_pose",            events={"e_finished@etaslcore"}},
        rfsm.trans {src="check_if_new_pose",            tgt="check_if_done",                events={"e_after(2)"}},
        rfsm.trans {src="check_if_done",                tgt="moving_cartesian_approach"}, 
    },
 
    admittance = rfsm.state {
        entry=function()                   
        admittance_config()
        end,
                        
        exit=function()
            cleanup_controller()
        end,
    },


    
--============================== RUN A_mode_scanning  ===================================  
rfsm.trans {src="initial",              tgt="configured" },
rfsm.trans {src="configured",           tgt="idle",                 events={}},
rfsm.trans {src="idle",                 tgt="moving_joint_space",   events={"e_after(2)"}},
rfsm.trans {src="moving_joint_space",   tgt="wait",                 events={"e_finished@etaslcore"}},
rfsm.trans {src="wait",                 tgt="move_to_probe",        events={"e_start_probe"}},
rfsm.trans {src="move_to_probe",        tgt="wait",                 events={"e_probe_done"}},
rfsm.trans {src="wait",                 tgt="admittance",           events={"start_admittance"}},
rfsm.trans {src="admittance",           tgt="wait",                 events={"stop_admittance"}},


}
