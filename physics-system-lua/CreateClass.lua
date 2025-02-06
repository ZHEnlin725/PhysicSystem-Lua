local _Class = {}

local ClassType = {
    Class = 1,
    Instance = 2
}

function CreateClass(className, base)
    assert(type(className) == "string" and #className > 0)
    local class = {}

    --构造函数
    class.__ctor = false
    --析构函数
    class.__delete = false
    --类名
    class.__cname = className
    --类类型
    class.__ctype = ClassType.Class

    class.base = base
    class.New = function(...)
        local inst = {}
        inst.__class = class
        inst.__ctype = ClassType.Instance
        setmetatable(inst, { __index = _Class[class] })
        do
            local create
            create = function(c, ...)
                if c.base then
                    create(c.base, ...)
                end
                if c.__ctor then
                    c.__ctor(inst, ...)
                end
            end
            create(class, ...)
        end

        inst.Delete = function(self)
            local _base = self.__class
            while _base do
                if _base.__delete then
                    _base.__delete(self)
                end
                _base = _base.base
            end
        end
        return inst
    end

    --修改虚表 访问安全
    local vtbl = {}
    _Class[class] = vtbl

    setmetatable(class, { __newindex = function(t, k, v)
        vtbl[k] = v
    end, __index = vtbl })
    if base then
        setmetatable(vtbl, { __index = function(t, k)
            return _Class[base][k]
        end })
    end
    return class
end