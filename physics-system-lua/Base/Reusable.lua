local Reusable = CreateClass("Reusable")

local _poolList = {}

function Reusable:__delete()
    if self.__class.__cname ~= 'Reusable' then
        _poolList[self.__class.__cname] = _poolList[self.__class.__cname] or {}
        table.insert(_poolList[self.__class.__cname], self)
    end
end

local function Get(cname)
    local list = _poolList[cname]
    if list and #list > 0 then
        return table.remove(list)
    end
end

local function ClearPool(cname)
    _poolList[cname] = nil
end

local function ClearAllPools()
    _poolList = {}
end

Reusable.Get = Get
Reusable.ClearPool = ClearPool
Reusable.ClearAllPools = ClearAllPools

return Reusable