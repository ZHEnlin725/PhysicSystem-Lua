---@class Shape
local Shape = CreateClass("Shape")
local id_alloc

SHAPE_TYPE = {
    CIRCLE = 1,
    SEGMENT = 2,
    POLY = 4,
}

function Shape:__ctor()
    id_alloc = id_alloc or 0
    self.id = id_alloc
    id_alloc = id_alloc + 1

    self.type = false

    -- Coefficient of restitution (elasticity)
    self.e = 0.0

    -- Frictional coefficient
    self.u = 1.0

    --  Mass density
    self.density = 1

    -- Axis-aligned bounding box
    ---@type Bounds
    self.bounds = Bounds.New()
end

function Shape:pointQuery(p)
    return false
end

function Shape:centroid()
    return Vec2.zero
end

function Shape:area()
    return 0
end

function Shape:findVertexByPoint(p, minDist)
    return -1
end

return Shape