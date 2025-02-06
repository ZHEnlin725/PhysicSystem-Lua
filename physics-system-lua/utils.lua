local function centroidForPoly(verts)
    local area = 0
    local vsum = Vec2.New(0, 0)
    local len = #verts
    for i = 1, len do
        local v1 = verts[i]
        local nextI = i + 1
        if nextI > len then
            nextI = 1
        end
        local v2 = verts[nextI]
        local cross = Vec2.cross(v1, v2)
        area = area + cross
        vsum:add(Vec2.scale(Vec2.add(v1, v2), cross))
    end
    return Vec2.scale(vsum, 1 / (3 * area))
end

local function centroidForSegment(a, b)
    return Vec2.scale(Vec2.add(a, b), 0.5)
end

local function inertiaForPoly(mass, verts, offset)
    local sum1 = 0
    local sum2 = 0
    local len = #verts
    for i = 1, len do
        local v1 = Vec2.add(verts[i], offset)
        local nextI = i + 1
        if nextI > len then
            nextI = 1
        end
        local v2 = Vec2.add(verts[nextI], offset)
        local a = Vec2.cross(v2, v1)
        local b = Vec2.dot(v1, v1) + Vec2.dot(v1, v2) + Vec2.dot(v2, v2)
        sum1 = sum1 + a * b
        sum2 = sum2 + a
    end
    return (mass * sum1) / (6 * sum2)
end

---@param center Vec2
local function inertiaForCircle(mass, center, radius_outer, radius_inner)
    return mass * ((radius_outer * radius_outer + radius_inner * radius_inner) * 0.5 + center:lengthSq())
end

local function inertiaForSegment(mass, a, b)
    local distsq = Vec2.distSq(b, a)
    local offset = Vec2.scale(Vec2.add(a, b), 0.5)
    return mass * (distsq / 12 + offset:lengthSq())
end

local function inertiaForBox(mass, w, h)
    return mass * (w * w + h * h) / 12
end

local function areaForCircle(radius_outer, radius_inner)
    return math.pi * (radius_outer * radius_outer - radius_inner * radius_inner)
end

local function areaForSegment(a, b, radius)
    return radius * (math.pi * radius + 2 * Vec2.dist(a, b))
end

local function areaForPoly(verts)
    local area = 0
    local len = verts and #verts or 0
    for i = 1, len do
        local nextI = i + 1
        if nextI > len then
            nextI = 1
        end
        area = area + Vec2.cross(verts[i], verts[nextI])
    end
    return area / 2
end

---@param points Vec2[]
local function createConvexHull(points)
    -- Find the right most point on the hull
    local i0 = 1
    local x0 = points[1].x
    local len = #points
    for i = 2, len do
        local x = points[i].x
        if x > x0 or (x == x0 and points[i].y < points[i0].y) then
            i0 = i
            x0 = x
        end
    end

    local hull = {}
    local m = 1
    local ih = i0
    while true do
        hull[m] = ih

        local ie = 1
        for i = 2, len do
            if ie == ih then
                ie = i
            else
                local r = Vec2.sub(points[ie], points[hull[m]])
                local v = Vec2.sub(points[i], points[hull[m]])
                local c = Vec2.cross(r, v)
                if c < 0 then
                    ie = i
                end
                if c == 0 and v:lengthSq() > r:lengthSq() then
                    ie = i
                end
            end
        end
        m = m + 1
        ih = ie
        if ie == i0 then
            break
        end
    end

    -- Copy vertices
    local newPoints = {}
    for i = 1, m do
        table.insert(newPoints[hull[i]])
    end
    return newPoints
end

math.clamp = function(val, min, max)
    return math.min(max, math.max(min, val))
end

return {
    centroidForPoly = centroidForPoly,
    centroidForSegment = centroidForSegment,

    inertiaForPoly = inertiaForPoly,
    inertiaForCircle = inertiaForCircle,
    inertiaForSegment = inertiaForSegment,
    inertiaForBox = inertiaForBox,

    areaForPoly = areaForPoly,
    areaForSegment = areaForSegment,
    areaForCircle = areaForCircle,

    createConvexHull = createConvexHull,
}