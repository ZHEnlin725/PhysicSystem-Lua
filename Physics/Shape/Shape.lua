local base = Body
---@class Shape:Body
local Shape = Create("Shape", base)

function Shape:__ctor()
    self.shapeType = nil
    self.aabbExtension = 0
    self.aabb = nil
    self._updatedCount = 0
end

function Shape:updateNormals()
end

function Shape:setAngle(angle)
    base.setAngle(self, angle)
    self.cos = math.cos(angle)
    self.sin = math.sin(angle)
end

function Shape:inAABB(x, y)
    return self.aabb[1] < x and x < self.aabb[3] and self.aabb[2] < y and y < self.aabb[4]
end

return Shape