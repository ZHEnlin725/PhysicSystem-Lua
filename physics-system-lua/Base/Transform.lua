---@class Transform
local Transform = CreateClass("Transform")

---@param pos Vec2
function Transform:__ctor(pos, rad)
    ---@type Vec2
    self.t = pos:duplicate()
    self.c = math.cos(rad)
    self.s = math.sin(rad)
end

function Transform:set(pos, rad)
    self:setPosition(pos)
    self:setRotation(rad)
end

function Transform:setRotation(rad)
    self.c = math.cos(rad)
    self.s = math.sin(rad)
end

function Transform:setPosition(pos)
    self.t:copy(pos)
end

function Transform:identity()
    self:set(0, 0)
    self.c = 1
    self.s = 0
    return self
end

function Transform:rotate(v)
    return Vec2.New(v.x * self.c - v.y * self.s, v.x * self.s + v.y * self.c)
end

function Transform:invRotate(v)
    return Vec2.New(v.x * self.c + v.y * self.s, -v.x * self.s + v.y * self.c)
end

function Transform:transform(v)
    return Vec2.New(v.x * self.c - v.y * self.s + self.t.x, v.x * self.s + v.y * self.c + self.t.y)
end

function Transform:invTransform(v)
    local px = v.x - self.t.x
    local py = v.y - self.t.y
    return Vec2.New(px * self.c + py * self.s, -px * self.s + py * self.c)
end

return Transform