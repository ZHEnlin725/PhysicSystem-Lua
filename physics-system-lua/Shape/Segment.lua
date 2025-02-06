local base = Shape
---@class Segment:Shape
local Segment = CreateClass("Segment", base)

function Segment:__ctor(a, b, radius)
    self.type = SHAPE_TYPE.SEGMENT

    self.a = a:duplicate()
    self.b = b:duplicate()
    self.r = radius
    self.n = Vec2.perp(Vec2.sub(b, a))
    self.n:normalize()
    self.ta = Vec2.zero
    self.tb = Vec2.zero
    self.tn = Vec2.zero
    self:finishVertices()
end

function Segment:finishVertices()
    self.n = Vec2.perp(Vec2.sub(self.b, self.a))
    self.n:normalize()
    self.r = math.abs(self.r)
end

function Segment:duplicate()
    return Segment.New(self.a, self.b, self.r)
end

---@param trans Transform
function Segment:transform(trans)
    self.a = trans:transform(self.a)
    self.b = trans:transform(self.b)
end

---@param trans Transform
function Segment:invTransform(trans)
    self.a = trans:invTransform(self.a)
    self.b = trans:invTransform(self.b)
end

function Segment:area()
    return utils.areaForSegment(self.a, self.b, self.r)
end

function Segment:centroid()
    return utils.centroidForSegment(self.a, self.b)
end

function Segment:inertia(mass)
    return utils.inertiaForSegment(mass, self.a, self.b)
end

function Segment:cacheData(trans)
    self.ta = trans:transform(self.a)
    self.tb = trans:transform(self.b)
    self.tn = Vec2.perp(Vec2.sub(self.tb, self.ta)):normalize()

    local l, r, b, t
    if (self.ta.x < self.tb.x) then
        l = self.ta.x
        r = self.tb.x
    else
        l = self.tb.x
        r = self.ta.x
    end

    if (self.ta.y < self.tb.y) then
        b = self.ta.y
        t = self.tb.y
    else
        b = self.tb.y
        t = self.ta.y
    end

    self.bounds.min:set(l - self.r, b - self.r)
    self.bounds.max:set(r + self.r, t + self.r)
end

function Segment:pointQuery(p)
    if not self.bounds:contain(p) then
        return
    end
    local dn = Vec2.dot(self.tn, p) - Vec2.dot(self.ta, self.tn)
    local dist = math.abs(dn)
    if (dist > self.r) then
        return false
    end

    local dt = Vec2.cross(p, self.tn)
    local dta = Vec2.cross(self.ta, self.tn)
    local dtb = Vec2.cross(self.tb, self.tn)
    if (dt <= dta) then
        if (dt < dta - self.r) then
            return false
        end

        return Vec2.distSq(self.ta, p) < (self.r * self.r)

    elseif (dt > dtb) then
        if (dt > dtb + self.r) then
            return false
        end

        return Vec2.distSq(self.tb, p) < (self.r * self.r)
    end

    return true
end

function Segment:findVertexByPoint(p, minDist)
    local dsq = minDist * minDist

    if (Vec2.distSq(self.ta, p) < dsq) then
        return 1
    end

    if (Vec2.distSq(self.tb, p) < dsq) then
        return 2
    end

    return -1
end

function Segment:distOnPlane(n, d)
    local a = Vec2.dot(n, self.ta) - self.r
    local b = Vec2.dot(n, self.tb) - self.r

    return math.min(a, b) - d
end

function Segment:draw(dl, color)
    local c = color or Color.green
    local p0 = Vector3(self.ta.x, self.ta.y)
    local p1 = Vector3(self.tb.x, self.tb.y)
    dl(p0, p1, c)
end

return Segment