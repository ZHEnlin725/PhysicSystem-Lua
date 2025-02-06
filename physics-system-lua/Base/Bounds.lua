---@class Bounds_Joint
local Bounds = CreateClass("Bounds")

---@param min Vec2
---@param max Vec2
function Bounds:__ctor(min, max)
    ---@type Vec2
    self.min = min and Vec2.New(min.x, min.y) or Vec2.New(math.huge, math.huge)
    ---@type Vec2
    self.max = max and Vec2.New(max.x, max.y) or Vec2.New(-math.huge, -math.huge)
end

function Bounds:reset()
    self.min:set(math.huge, math.huge)
    self.max:set(-math.huge, -math.huge)
end

function Bounds:isEmpty()
    if self.min.x > self.max.x or self.min.y > self.max.y then
        return true
    end
end

function Bounds:setMinMax(min, max)
    self.min:set(math.min(self.min.x, min.x), math.min(self.min.y, min.y))
    self.min:set(math.min(self.min.x, max.x), math.min(self.min.y, max.y))

    self.max:set(math.max(self.max.x, max.x), math.max(self.max.y, max.y))
    self.max:set(math.max(self.max.x, min.x), math.max(self.max.y, min.y))
end

function Bounds:getCenter()
    return Vec2.scale(Vec2.add(self.min, self.max), 0.5)
end

function Bounds:getExtent()
    return Vec2.scale(Vec2.sub(self.max, self.min), 0.5)
end

function Bounds:getPerimeter()
    return (self.max.x - self.min.x + self.max.y - self.min.y) * 2
end

function Bounds:encapsulate(point)
    self:setMinMax(point, point)
end

function Bounds:encapsulateBounds(bounds)
    self:setMinMax(bounds.min, bounds.max)
end

function Bounds:contain(p)
    if p.x < self.min.x or p.x > self.max.x or p.y < self.min.y or p.y > self.max.y then
        return false
    end
    return true
end

function Bounds:intersects(b)
    if self.min.x > b.max.x or self.max.x < b.min.x or self.min.y > b.max.y or self.max.y < b.min.y then
        return false
    end
    return true
end

return Bounds