local base = Shape
---@class Circle:Shape
local Circle = CreateClass("Circle", base)

function Circle:__ctor(local_x, local_y, radius)
    self.type = SHAPE_TYPE.CIRCLE
    ---@type Vec2
    self.c = Vec2.New(local_x or 0, local_y or 0)
    self.r = radius
    ---@type Vec2
    self.tc = Vec2.zero

    self:finishVertices()
end

function Circle:finishVertices()
    self.r = math.abs(self.r)
end

function Circle:duplicate()
    return Circle.New(self.c.x, self.c.y, self.r)
end

function Circle:transform(trans)
    self.c = trans:transform(self.c)
end

function Circle:invTransform(trans)
    self.c = trans:invTransform(self.c)
end

function Circle:area()
    return utils.areaForCircle(self.r, 0)
end

function Circle:centroid()
    return self.c:duplicate()
end

function Circle:inertia(mass)
    return utils.inertiaForCircle(mass, self.c, self.r, 0)
end

---@param trans Transform
function Circle:cacheData(trans)
    self.tc = trans:transform(self.c);
    self.bounds.min:set(self.tc.x - self.r, self.tc.y - self.r)
    self.bounds.max:set(self.tc.x + self.r, self.tc.y + self.r)
end

function Circle:pointQuery(p)
    return Vec2.distSq(self.tc, p) < (self.r * self.r)
end

function Circle:findVertexByPoint(p, minDist)
    local dsq = minDist * minDist
    if Vec2.distSq(self.tc, p) < dsq then
        return 1
    end
    return -1
end

function Circle:distOnPlane(n, d)
    return Vec2.dot(n, self.tc) - self.r - d
end

function Circle:draw(dl, color)
    local c = color or Color.green
    local center = Vector3(self.tc.x, self.tc.y)
    local up = Vec2.New(0, 1)
    for i = 1, 10 do
        local rad = math.pi * 2 * i / 10
        local val = Vec2.rotate(up, rad)
        dl(center, center + self.r * Vector3(val.x, val.y), c)
    end
end

return Circle