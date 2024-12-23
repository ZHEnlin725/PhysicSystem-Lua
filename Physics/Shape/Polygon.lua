local base = Shape
---@class Polygon:Shape
local Polygon = Create("Polygon", base)

function Polygon:__ctor()
    self.shapeType = ShapeType.Poly
    self.vertices = nil
    self.normals = nil
    self.localVertices = nil
    self.localNormals = nil
    self.vertexCount = 0
    self._updatedCount = 0
end

function Polygon:init()
    self:initMassData()

    if self.x == nil then
        self.x = self.originalCentroid[1]
    end
    if self.y == nil then
        self.y = self.originalCentroid[2]
    end
    self.originalX = self.x
    self.originalY = self.y
    self:setAngle(self.angle or 0)

    self.vertexCount = #self.vertices
    self.normals = Polygon.computeNormals(self.vertices)

    self.last = {}
    self.aabb = { 0, 0, 0, 0 }
    self:initLocalData()
end

function Polygon:initLocalData()
    local minX, minY = math.huge, math.huge
    local maxX, maxY = -minX, -minY

    local vc = self.vertexCount
    self.localVertices = self.localVertices or {}
    for i = 1, vc do
        local v = self.vertices[i]
        local ox = v[1] - self.x
        local oy = v[2] - self.y
        local x = ox * self.cos + oy * self.sin
        local y = -ox * self.sin + oy * self.cos
        if i <= #self.localVertices then
            self.localVertices[i][1] = x
            self.localVertices[i][2] = y
        else
            table.insert(self.localVertices, { x, y })
        end

        if v[1] < minX then
            minX = v[1]
        end
        if v[1] > maxX then
            maxX = v[1]
        end
        if v[2] < minY then
            minY = v[2]
        end
        if v[2] > maxY then
            maxY = v[2]
        end
    end
    local oc = self.originalCentroid
    local ox = oc[1] - self.x
    local oy = oc[2] - self.y
    local x = ox * self.cos + oy * self.sin
    local y = -ox * self.sin + oy * self.cos
    if not self.localOriginalCentroid then
        self.localOriginalCentroid = { x, y }
    else
        self.localOriginalCentroid[1] = x
        self.localOriginalCentroid[2] = y
    end
    self.localNormals = Polygon.computeNormals(self.localVertices, self.localNormals or {})
    self.aabb[1] = minX - self.aabbExtension
    self.aabb[2] = minY - self.aabbExtension
    self.aabb[3] = maxX + self.aabbExtension
    self.aabb[4] = maxY + self.aabbExtension
end

function Polygon:initMassData()
    local vertices = self.vertices
    local len = #vertices

    local ac = { 0, 0 }
    for i = 1, len do
        local v = vertices[i]
        ac[1] = ac[1] + v[1]
        ac[2] = ac[2] + v[2]
    end
    ac[1] = ac[1] / len
    ac[2] = ac[2] / len

    local c = { 0, 0 }
    local area2 = 0
    local originalInertia = 0

    local v0 = vertices[len]
    local x1 = v0[1] - ac[1]
    local y1 = v0[2] - ac[2]
    for i = 1, len do
        local v1 = vertices[i]
        local x2 = v1[1] - ac[1]
        local y2 = v1[2] - ac[2]

        local triangleArea2 = (x1 * y2 - y1 * x2)
        area2 = area2 + triangleArea2

        c[1] = c[1] + triangleArea2 * (x1 + x2)
        c[2] = c[2] + triangleArea2 * (y1 + y2)

        local inertiaX = (x1 * x1 + x1 * x2 + x2 * x2)
        local inertiaY = (y1 * y1 + y1 * y2 + y2 * y2)

        originalInertia = originalInertia + triangleArea2 * (inertiaX + inertiaY)

        x1 = x2
        y1 = y2
    end

    c[1] = c[1] / (area2 * 3) + ac[1]
    c[2] = c[2] / (area2 * 3) + ac[2]
    self.originalCentroid = c

    self.area = area2 / 2
    self.density = self.density or 1
    self:setMass(self.mass)
    self.originalInertia = originalInertia / 12 * self.density
    self:setInertia(self.inertia ~= nil and self.inertia or self.originalInertia)
end

function Polygon:translateCentroid(x, y)
    self.x = self.x + x
    self.y = self.y + y
    local localVertices = self.localVertices
    local len = #localVertices
    for i = 1, len do
        local vertex = localVertices[i]
        vertex[1] = vertex[1] - x
        vertex[2] = vertex[2] - y
    end
end

function Polygon:updateVertices()
    local minX, minY = math.huge, math.huge
    local maxX, maxY = -minX, -minY
    local len = self.vertexCount
    for i = 1, len do
        local ov = self.localVertices[i]
        local ox = ov[1]
        local oy = ov[2]
        local x = ox * self.cos - oy * self.sin
        local y = ox * self.sin + oy * self.cos
        local v = self.vertices[i]
        v[1] = x + self.x
        v[2] = y + self.y

        if v[1] < minX then
            minX = v[1]
        end
        if v[1] > maxX then
            maxX = v[1]
        end
        if v[2] < minY then
            minY = v[2]
        end
        if v[2] > maxY then
            maxY = v[2]
        end
    end
    self.aabb[1] = minX - self.aabbExtension
    self.aabb[2] = minY - self.aabbExtension
    self.aabb[3] = maxX + self.aabbExtension
    self.aabb[4] = maxY + self.aabbExtension
end

function Polygon:updateNormals()
    for i = 1, self.vertexCount do
        local on = self.localNormals[i]
        local x = self.cos * on[1] - self.sin * on[2]
        local y = self.sin * on[1] + self.cos * on[2]

        local n = self.normals[i]
        n[1] = x
        n[2] = y

        local v = self.vertices[i]
        n[3] = x * v[1] + y * v[2]
    end
end

function Polygon:update(timeStep)
    base.update(self, timeStep)
    self._updatedCount = self._updatedCount + 1
    self:setAngle(self.angle)
    self:updateVertices()
    self:updateNormals()
end

function Polygon:updateAABB()
    local minX, minY = math.huge, math.huge
    local maxX, maxY = -minX, -minY

    for i = 1, self.vertexCount do
        local v = self.vertices[i]

        if v[1] < minX then
            minX = v[1]
        end
        if v[1] > maxX then
            maxX = v[1]
        end
        if v[2] < minY then
            minY = v[2]
        end
        if v[2] > maxY then
            maxY = v[2]
        end
    end
    self.aabb[1] = minX - self.aabbExtension
    self.aabb[2] = minY - self.aabbExtension
    self.aabb[3] = maxX + self.aabbExtension
    self.aabb[4] = maxY + self.aabbExtension
end

function Polygon:containPoint(x, y)
    if not self:inAABB(x, y) then
        return false
    end
    local vertices = self.vertices
    local len = #vertices
    local p = vertices[len]
    local px, py = p[1], p[2]
    local normals = self.normals
    local found = 0

    local n = normals[len]

    for i = 1, len do
        local q = vertices[i]
        local qx, qy = q[1], q[2]

        local minX, maxX
        if px < qx then
            minX = px
            maxX = qx
        else
            minX = qx
            maxX = px
        end

        if x >= minX and x <= maxX then
            local det = n[1] * x + n[2] * y - n[3]
            if det > 0 then
                return false
            end
            if found == 1 then
                return true
            end
            found = found + 1
        end
        n = normals[i]
        px = qx
        py = qy
    end

    return false
end

function Polygon:pointRayCasting(x, y)
    local vertices = self.vertices
    local len = #vertices
    local p = vertices[len]
    local px, py = p[1], p[2]

    local flag = false
    for i = 1, len do
        local q = vertices[i]
        local qx, qy = q[1], q[2]

        if (qx == x and qy == y) or (px == x and py == y) then
            return 'on'
        end
        if (qy < y and py >= y) or (qy >= y and py < y) then
            local _x = qx + (y - qy) * (px - qx) / (py - qy)
            if _x == x then
                return 'on'
            end
            if _x > x then
                flag = not flag
            end
        end
        px = qx
        py = qy
    end
    return flag and 'in' or 'out'
end

function Polygon.projectPointToEdge(p, e0, e1)
    local v = { p[1] - e0[1], p[2] - e0[2] }
    local edge = { e1[1] - e0[1], e1[2] - e0[2] }

    local t = (edge[1] * v[1] + edge[2] * v[2]) / (edge[1] * edge[1] + edge[2] * edge[2])

    if t < 0 then
        t = 0
    elseif t > 1 then
        t = 1
    end
    local contact = { e0[1] + edge[1] * t, e0[2] + edge[2] * t }

    return contact
end

function Polygon.computeNormal(v0, v1)
    local nx = v1[2] - v0[2]
    local ny = v0[1] - v1[1]

    local length = math.sqrt(nx * nx + ny * ny)
    nx = nx / length
    ny = ny / length

    local np = nx * v0[1] + ny * v0[2]
    return { nx, ny, np }
end

function Polygon.computeNormals(vertices, normals)
    local len = #vertices
    normals = normals or {}
    for i = 1, len do
        normals[i] = nil
    end
    local normal, vertex
    local lastVertex = vertices[len]
    for i = 1, len do
        vertex = vertices[i]
        normal = Polygon.computeNormal(lastVertex, vertex)
        table.insert(normals, normal)
        lastVertex = vertex
    end
    table.insert(normals, table.remove(normals, 1))
    return normals
end

return Polygon