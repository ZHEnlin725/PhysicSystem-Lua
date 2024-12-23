local base = Shape
---@class Composition:Shape
local Composition = Create("Composition", base)

function Composition:__ctor(shapes)
    self.shapeType = ShapeType.Comp
    ---@type Shape[]
    self.shapes = shapes
end

function Composition:init()
    self.shapes = self.shapes or {}
    local pMass = nil
    local shapeLength = self.shapes and #self.shapes or 0
    if (self.mass ~= nil and shapeLength > 0) then
        pMass = self.mass / shapeLength
    end
    for _, shape in ipairs(self.shapes) do
        shape.bodyType = self.bodyType
        if pMass then
            shape.mass = pMass
        end
        shape:init()
    end
    self:initMassData()
    local x = self.originalCentroid[1]
    local y = self.originalCentroid[2]
    for _, shape in ipairs(self.shapes) do
        local sx, sy = shape.x, shape.y
        shape:translateCentroid(x - sx, y - sy)
        shape.body = self
    end

    self.x = x
    self.y = y
    self.originalX = self.x
    self.originalY = self.y

    base.init(self)
    --self.initLocalData()
end

function Composition:initMassData()
    local mass, inertia, area = 0, 0, 0
    local shapes = self.shapes
    local len = shapes and #shapes or 0
    local x, y = 0, 0
    for i = 1, len do
        local s = shapes[i]
        mass = mass + s.mass
        inertia = inertia + s.inertia
        area = area + s.area
        x = x + s.x
        y = y + s.y
    end
    x = x / len
    y = y / len
    local vertices = {}
    for i = 1, len do
        local s = shapes[i]
        table.insert(vertices, { [1] = s.x - x, [2] = s.y - y })
    end
    local c = self:computeCentroid(vertices)
    c[1] = c[1] + x
    c[2] = c[2] + y

    self.area = area
    self.originalCentroid = c

    self:setMass(mass)
    self:setInertia(inertia)
end

function Composition:computeCentroid(vertices)
    local len = #vertices
    local c
    local x, y = 0, 0
    for _, v in ipairs(vertices) do
        x = x + v[1]
        y = y + v[2]
    end
    c = { [1] = x / len, [2] = y / len }
    return c
end

function Composition:translateCentroid(x, y)
    self.x = self.x + x
    self.y = self.y + y
    local localVertices = self.localVertices
    local len = localVertices and #localVertices or 0
    for i = 1, len do
        local vert = localVertices[i]
        vert[1] = vert[1] - x
        vert[2] = vert[2] - y
    end
end

function Composition:updateVertices()
    local minX, minY = math.huge, math.huge
    local maxX, maxY = -minX, -minY
    local shapes = self.shapes
    local len = shapes and #shapes or 0
    for i = 1, len do
        local s = shapes[i]
        s:updateVertices()
        local aabb = s.aabb
        minX = math.min(aabb[1], minX)
        maxX = math.max(aabb[3], maxX)
        minY = math.min(aabb[2], minY)
        maxY = math.max(aabb[4], maxY)
    end
    self.aabb[1] = minX
    self.aabb[2] = minY
    self.aabb[3] = maxX
    self.aabb[4] = maxY
end

function Composition:updateNormals()
    if self.shapes then
        for _, shape in ipairs(self.shapes) do
            shape:updateNormals()
        end
    end
end

function Composition:update(timeStep)
    base.update(self, timeStep)
    self._updatedCount = self._updatedCount + 1

    self:setAngle(self.angle)
    self:updateVertices()
    self:updateNormals()
end

function Composition:setPos(x, y)
    base.setPos(self, x, y)
    for _, shape in ipairs(self.shapes) do
        shape:setPos(x, y)
    end
end

function Composition:setAngle(angle)
    base.setAngle(self, angle)
    for _, shape in ipairs(self.shapes) do
        shape:setAngle(angle)
    end
end

---@param shape Shape
function Composition:addShape(shape)
    self.shapes = self.shapes or {}
    table.insert(self.shapes, shape)
    shape.body = self
end

function Composition:containPoint(x, y)
    for _, shape in ipairs(self.shapes) do
        if shape:containPoint(x, y) then
            return true
        end
    end
    return false
end

function Composition:drawaabb(color)
    base.drawaabb(self, color)
    for _, shape in ipairs(self.shapes) do
        shape:drawaabb(color)
    end
end

return Composition