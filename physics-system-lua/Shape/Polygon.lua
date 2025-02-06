local base = Shape
---@class Polygon:Shape
local Polygon = CreateClass("Polygon", base)

function Polygon:__ctor(vertices)
    self.type = SHAPE_TYPE.POLY
    self.vertices = {}
    self.planes = {}

    self.tvertices = false
    self.tplanes = false
    if vertices then
        for i, vert in ipairs(vertices) do
            self.vertices[i] = vert:duplicate()
            --        self.tvertices[i] = self.vertices[i]
            --
            --        self.tplanes[i] = {}
            --        self.tplanes[i].n = Vec2.zero
            --        self.tplanes[i].d = 0
        end
    end
    self:finishVertices()
end

function Polygon:duplicate()
    return Polygon.New(self.vertices)
end

function Polygon:finishVertices()
    local length = self.vertices and #self.vertices or 0
    if length < 2 then
        self.convexity = false
        self.planes = {}
        return
    end

    self.convexity = true
    self.tvertices = {}
    self.tplanes = {}
    -- Must be counter-clockwise vertices
    for i = 1, length do
        local a = self.vertices[i]
        local nextI = i + 1
        if nextI > length then
            nextI = 1
        end
        local b = self.vertices[nextI]
        local n = Vec2.normalize(Vec2.perp(Vec2.sub(a, b)))

        self.planes[i] = {}
        self.planes[i].n = n
        self.planes[i].d = Vec2.dot(n, a)

        self.tvertices[i] = self.vertices[i]

        self.tplanes[i] = {}
        self.tplanes[i].n = Vec2.zero
        self.tplanes[i].d = 0
    end
    for i = 1, length do
        local next2I = i + 2
        if next2I > length then
            next2I = 1
        end
        local b = self.vertices[next2I]
        local n = self.planes[i].n
        local d = self.planes[i].d

        if (Vec2.dot(n, b) - d > 0) then
            self.convexity = false
        end
    end
end

---@param t Transform
function Polygon:transform(t)
    local len = #self.vertices
    for i = 1, len do
        self.vertices[i] = t:transform(self.vertices[i])
    end
end

---@param t Transform
function Polygon:invTransform(t)
    local len = #self.vertices
    for i = 1, len do
        self.vertices[i] = t:invTransform(self.vertices[i])
    end
end

function Polygon:area()
    return utils.areaForPoly(self.vertices)
end

function Polygon:centroid()
    return utils.centroidForPoly(self.vertices)
end

function Polygon:inertia(mass)
    return utils.inertiaForPoly(mass, self.vertices, Vec2.zero)
end

function Polygon:cacheData(trans)
    self.bounds:reset()
    local len = #self.vertices
    if len == 0 then
        return
    end
    for i = 1, len do
        self.tvertices[i] = trans:transform(self.vertices[i])
    end
    if len < 2 then
        self.bounds:encapsulate(self.tvertices[1])
        return
    end
    for i = 1, len do
        local nextI = i + 1
        if nextI > len then
            nextI = 1
        end
        local a = self.tvertices[i]
        local b = self.tvertices[nextI]
        local n = Vec2.normalize(Vec2.perp(Vec2.sub(a, b)))
        self.tplanes[i].n = n
        self.tplanes[i].d = Vec2.dot(n, a)
        self.bounds:encapsulate(a)
    end
end

function Polygon:pointQuery(p)
    if not self.bounds:contain(p) then
        return false
    end
    return self:containPoint(p)
end

function Polygon:findVertexByPoint(p, minDist)
    local dsq = minDist * minDist
    for i, vert in ipairs(self.vertices) do
        if Vec2.distSq(vert, p) < dsq then
            return i
        end
    end
    return -1
end

function Polygon:findEdgeByPoint(p, minDist)
    local dsq = minDist * minDist
    local len = #self.tvertices
    for i = 1, len do
        local nextI = i + 1
        if nextI > len then
            nextI = 1
        end
        local v1 = self.tvertices[i];
        local v2 = self.tvertices[nextI]
        local n = self.tplanes[i].n
        local dtv1 = Vec2.cross(v1, n)
        local dtv2 = Vec2.cross(v2, n)
        local dt = Vec2.cross(p, n)
        if dt > dtv1 then
            if (Vec2.distSq(v1, p) < dsq) then
                return i
            end
        elseif dt < dtv2 then
            if (Vec2.distSq(v2, p) < dsq) then
                return i
            end
        else
            local dist = Vec2.dot(n, p) - Vec2.dot(n, v1)
            if (dist * dist < dsq) then
                return i
            end
        end
    end
    return -1
end

function Polygon:distOnPlane(n, d)
    local min = math.huge
    local len = #self.vertices
    for i = 1, len do
        min = math.min(min, Vec2.dot(n, self.tvertices[i]))
    end
    return min - d
end

function Polygon:containPoint(p)
    for _, plane in ipairs(self.tplanes) do
        if Vec2.dot(plane.n, p) - plane.d > 0 then
            return false;
        end
    end
    return true
end

function Polygon:containPointPartial(p, n)
    for _, plane in ipairs(self.tplanes) do
        if Vec2.dot(plane.n, n) >= 0.0001 then
            if Vec2.dot(plane.n, p) - plane.d > 0 then
                return false
            end
        end
    end
    return true
end

function Polygon:draw(dl, color)
    local c = color or Color.green
    local len = #self.vertices
    for i = 1, len do
        local nextI = i + 1
        if nextI > len then
            nextI = 1
        end
        dl(Vector3(self.tvertices[i].x, self.tvertices[i].y), Vector3(self.tvertices[nextI].x, self.tvertices[nextI].y), c)
    end
end

return {
    New = Polygon.New,

    ---@return Poly
    NewBox = function(local_x, local_y, width, height)
        local hw = width * .5
        local hh = height * .5
        local vertices = {
            Vec2.New(-hw + local_x, hh + local_y),
            Vec2.New(-hw + local_x, -hh + local_y),
            Vec2.New(hw + local_x, -hh + local_y),
            Vec2.New(hw + local_x, hh + local_y)
        }
        return Polygon.New(vertices)
    end,

    ---@return Poly
    NewTriangle = function(p1, p2, p3)
        local vertices = { p1, p2, p3 }
        return Polygon.New(vertices)
    end
}