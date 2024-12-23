local base = Polygon
---@class Segment:Polygon
local Segment = Create("Segment", base)

function Segment:__ctor()
    self.shapeType = ShapeType.Poly
    self.mass = 0
    self.inertia = 0
end

function Segment:initMassData()
    local v0 = self.vertices[1]
    local v1 = self.vertices[2]

    self.originalCentroid = { (v0[1] + v1[1]) / 2, (v0[2] + v1[2]) / 2 }
    self.density = self.density or 1
    self.area = 0

    self:setMass(self.mass)
    self.originalInertia = (self.originalInertia or 0) * self.density
    self:setInertia(self.inertia ~= nil and self.inertia or self.originalInertia)
end

function Segment:containPoint(x, y)
    return false
end

function Segment:pointRayCasting(x, y)
    return false
end

return Segment