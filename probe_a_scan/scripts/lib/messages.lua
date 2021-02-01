local messages ={}

function messages.err(...)
   print(ansicolors.bright(ansicolors.red(table.concat({...}, ' '))))
end


function messages.info(...)
    print(ansicolors.bright(ansicolors.green(table.concat({...}, ' '))))
end

function messages.warn(...)
    print(ansicolors.bright(ansicolors.yellow(table.concat({...}, ' '))))
end


messages.check_com=function(comm)
     f=loadstring("return " .. comm)
     res=f()
     if res==nil then messages.err("function ill-formed: " .. comm)
        return false
     end
    if not res then
        messages.err("failed execute: " .. comm)
        return false;
    end
    return true;
end

return messages

