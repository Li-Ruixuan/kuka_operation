require("rttlib")
require("math")

JSON = require("JSON")

local function read_to_string(file)
    local f = io.open(file, "rb")
    local content = f:read("*all")
    f:close()
    return content
end





tc=rtt.getTC()

iface_spec = {
   ports={
      { name='tf_pose', datatype='array', type='in', desc="x,y,z,R,P,Y from robot" },
      { name='commands', datatype='string', type='in+event', desc="commands" },
   },
 
   properties={
      { name='data_file', datatype='string', desc="name of file to save/load poses" }
   }
}


iface=rttlib.create_if(iface_spec)

jvals=rtt.Variable("array")

-- The Lua component starts its life in PreOperational, so
-- configureHook can be used to set stuff up.
function configureHook()
    iface=rttlib.create_if(iface_spec)
    return true
end

function startHook()
    local filename=iface.props.data_file:get()
    local js=read_to_string(filename)
    if (js~=nil) then
        return true
    end
    rtt.logl('Error', tc:getName() .. ": File " .. filename  .. "  does not exists")
    return false
end
--[[     return (is_zero(t.x) and is_zero(t.y) and is_zero(t.z) and 
             is_zero(t.R) and is_zero(t.P) and is_zero(t.Y))
]]
function check_if_not_zero(t)
    all_zero=true
    for key,val in pairs(t) do
        if val==0 then 
            all_zero=false
        end
    end
    return not all_zero
end

function update_table(pose,index,table)
    local function update_pose (pose, tab_entry)
        tab_entry.x=pose[1]
        tab_entry.y=pose[2]
        tab_entry.z=pose[3]
        tab_entry.R=pose[4]
        tab_entry.P=pose[5]
        tab_entry.Y=pose[6]
    end
    
    if type(index) == "number" then
        update_pose(pose,table.probe_frames[index])
    else
        update_pose(pose,table.center_frame)
    end
end

function updateHook()
    local template_end_pose="c_update_end_probe_pose_%d+"
    local template_middle_pose="c_update_middle_probe_pose"
    --Read the and interpret the command
    local fs,cmd=iface.ports.commands:read()
    if fs=='NewData' then 
 
        local to_update=nil
        if  nil ~= string.match(cmd,template_end_pose) then
            to_update=tonumber(string.match(cmd,'%d+'))
        end
        if cmd==template_middle_pose then
            to_update='middle'
        end
        if to_update then
    -- if something is to be updated, read the current robot position.
            fs,new_pose=iface.ports.tf_pose:read()
            if fs==NoData then
                rtt.logl('Error', tc:getName() .. ": No frame  available to update frame" .. tostring(to_update))
                return
            end
            if check_if_not_zero(new_pose:totab()) then
                rtt.logl('Error', tc:getName() .. ": The frame is read is all null, etaslcore is probably not running - frame not updated!")
                return
            end
           
            -- read the current pose list
            local filename=iface.props.data_file:get()
            rtt.logl('Info', tc:getName() .. "Update:" .. to_update .. " file: ".. filename)
            local js=read_to_string (filename)
            local t=JSON:decode(js)
            update_table(new_pose:totab(),to_update,t)
            js=JSON:encode_pretty(t)
            f=io.open(filename,"w")
            f:write(js)
            f:close()
            
        end
    end
end


function cleanupHook()
    rttlib.tc_cleanup()
end


