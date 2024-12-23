---@class CollideManager
local CollideManager = Create("CollideManager")

function CollideManager:__ctor()
    self.gridCellSize = 128
    self.gridCol = 512
    self.collideMethodMap = {
        [1] = "circleCircle",
        [2] = "polyPoly",
        [3] = "polyCircle",
        [8] = "compComp",
        [9] = "compSingle",
        [10] = "compSingle",
    }
end

function CollideManager:init(world)
    self.world = world
    self.penetrated = {}
    ---@type Arbiter[]
    self.arbiters = {}
    self.arbiterCount = 0
    self.collide = self.collide or self.collideSimple
end

function CollideManager:collideGrid(timeStep)
    local cc = 0
    local gridCellSize = self.gridCellSize
    local gridCol = self.gridCol
    local grid = {}

    local bodies = self.world.bodies

    for i = 1, #bodies do
        local bodyA = bodies[i]
        if not bodyA.disabled then
            local box1 = bodyA.aabb
            local colMin = math.floor(box1[1] / gridCellSize)
            local rowMin = math.floor(box1[2] / gridCellSize)
            local colMax = math.floor(box1[3] / gridCellSize)
            local rowMax = math.floor(box1[4] / gridCellSize)

            local checked = {}
            local startIdx = rowMin * gridCol + colMin

            for row = rowMin, rowMax do
                local idx = startIdx
                for col = colMin, colMax do
                    local group = grid[idx]
                    if not group then
                        grid[idx] = { bodyA }
                    else
                        for c = 1, #group do
                            local bodyB = group[c]
                            if not bodyB.disabled then
                                if not checked[bodyB.id] then
                                    checked[bodyB.id] = true
                                    self:collideTowBodies(bodyA, bodyB, timeStep)
                                    cc = cc + 1
                                end
                            end
                        end
                        table.insert(group, bodyA)
                    end
                    idx = idx + 1
                end
                startIdx = startIdx + gridCol
            end
        end
    end
    return cc
end

function CollideManager:collideSimple(timeStep)
    local cc = 0
    local bodies = self.world.bodies

    for i = 1, #bodies - 1 do
        local bodyA = bodies[i]
        if not bodyA.disabled then
            for j = i + 1, #bodies do
                local bodyB = bodies[j]
                if not bodyB.disabled then
                    self:collideTowBodies(bodyA, bodyB, timeStep)
                    cc = cc + 1
                end
            end
        end
    end
    return cc
end

---@param bodyA Body
---@param bodyB Body
function CollideManager:collideTowBodies(bodyA, bodyB, timeStep)
    if bodyA.sleeping and bodyB.sleeping then
        return nil
    end
    if bodyA.invMass == 0 and bodyB.invMass == 0 then
        return nil
    end
    local contactKey = bodyA.id .. "_&_" .. bodyB.id
    local arbiter = false
    local boxA = bodyA.aabb
    local boxB = bodyB.aabb
    if boxA[1] < boxB[3] and boxA[3] > boxB[1] and boxA[2] < boxB[4] and boxA[4] > boxB[2] then
        arbiter = self[self.collideMethodMap[bodyA.shapeType | bodyB.shapeType]](self, bodyA, bodyB)
    end

    if arbiter then
        self:onColliding(bodyA, bodyB, arbiter, timeStep)
        if not self.penetrated[contactKey] then
            self:onCollided(bodyA, bodyB, arbiter, timeStep)
            self.penetrated[contactKey] = 1
        else
            self.penetrated[contactKey] = self.penetrated[contactKey] + 1
        end
    else
        if self.penetrated[contactKey] then
            self:onSeparated(bodyA, bodyB, timeStep)
            self.penetrated[contactKey] = nil
        end
    end
    return arbiter
end

---@param bodyA Body
---@param bodyB Body
function CollideManager:hasCollided(bodyA, bodyB)
    local contactKey = bodyA.id .. "_&_" .. bodyB.id
    return self.penetrated[contactKey]
end

function CollideManager:getArbiter(bodyA, bodyB)
    local arbiters = self.arbiters
    local arbiterCount = self.arbiterCount
    if not arbiters[arbiterCount + 1] then
        arbiters[arbiterCount + 1] = Arbiter.New()
    end
    local arbiter = arbiters[arbiterCount + 1]
    self.arbiterCount = arbiterCount + 1
    return arbiter
end

function CollideManager:update(timeStep)
    self.arbiterCount = 0
    local cc = self:collide(timeStep)
end

function CollideManager:preSolve(timeStep)
    local arbiters = self.arbiters
    local arbiterCount = self.arbiterCount
    for i = 1, arbiterCount do
        local arbiter = arbiters[i]
        if not arbiter:isDisabled() then
            arbiter:preSolve(timeStep)
        end
    end
end

function CollideManager:solve(timeStep, iterations, iter)
    local arbiters = self.arbiters
    local arbiterCount = self.arbiterCount
    for i = 1, arbiterCount do
        local arbiter = arbiters[i]
        if not arbiter:isDisabled() then
            local solved = arbiter:solve(timeStep, iterations, iter)
            if solved then
                self:onCollideSolve(arbiter, timeStep, iterations, iter)
            end
        end
    end
end

function CollideManager:onCollided(bodyA, bodyB, arbiter, timeStep)
    --printError("onCollided", bodyA, bodyB)
end

function CollideManager:onSeparated(bodyA, bodyB, timeStep)
    --printError("onSeparated", bodyA, bodyB)
end

function CollideManager:onColliding(bodyA, bodyB, arbiter, timeStep)
    --printError("onColliding", bodyA, bodyB)
end

function CollideManager:onCollideSolve(arbiter, timeStep, iterations, iter)
end

---@param bodyA Body
---@param bodyB Body
function CollideManager:circleCircle(bodyA, bodyB)
    local centerA = bodyA.center
    local centerB = bodyB.center

    local dx = centerB[1] - centerA[1]
    local dy = centerB[2] - centerA[2]
    local distanceSq = dx * dx + dy * dy
    local rt = bodyA.radius + bodyB.radius
    if distanceSq > rt * rt then
        return false
    end
    if bodyA.isTrigger or bodyB.isTrigger then
        return true
    end
    local distance = math.sqrt(distanceSq)

    local normalA = { dx / distance, dy / distance }
    local ratioA = bodyA.radius / distance
    local contactOnA = {
        centerA[1] + (centerB[1] - centerA[1]) * ratioA,
        centerA[2] + (centerB[2] - centerA[2]) * ratioA
    }

    local normalB = { -normalA[1], -normalA[2] }
    local ratioB = bodyB.radius / distance
    local contactOnB = {
        centerB[1] + (centerA[1] - centerB[1]) * ratioB,
        centerB[2] + (centerA[2] - centerB[2]) * ratioB
    }
    local overlap = rt - distance

    local a = bodyA.body or bodyA
    local b = bodyB.body or bodyB

    ---@type Arbiter
    local arbiter = self:getArbiter(a, b)
    arbiter.overlap = overlap
    arbiter:set(a, b, normalA)

    arbiter:addContact(contactOnA, contactOnB, overlap)

    return arbiter
end

---@param bodyA Body
---@param bodyB Body
function CollideManager:polyCircle(bodyA, bodyB)
    local poly, circle
    if bodyA.shapeType == ShapeType.Poly then
        poly = bodyA
        circle = bodyB
    else
        poly = bodyB
        circle = bodyA
    end

    local result = self:featurePairPolyCircle(poly, circle)
    if not result then
        return false
    end

    local facePoly = result.facePoly
    local vertPoly = result.vertPoly
    local faceIdx = result.faceIdx
    local faceNormal = result.faceNormal
    local vertexIdx = result.closestIdx
    local overlap = result.minDist

    local contactOnFace0
    local contactOnVert0

    local center = circle.center

    if facePoly == circle then
        local r = circle.radius
        contactOnVert0 = result.closest
        contactOnFace0 = { faceNormal[1] * r + center[1], faceNormal[2] * r + center[2] }
    else
        local faceV0 = facePoly.vertices[faceIdx]
        --local faceV1 = facePoly.vertices[(faceIdx + 1) % facePoly.vertexCount]
        local faceIdx_add = faceIdx + 1
        local faceV1 = facePoly.vertices[faceIdx_add > facePoly.vertexCount and 1 or faceIdx_add]

        contactOnVert0 = result.closest
        contactOnFace0 = Polygon.projectPointToEdge(contactOnVert0, faceV0, faceV1)
    end

    local a = facePoly.body or facePoly
    local b = vertPoly.body or vertPoly

    local arbiter = self:getArbiter(a, b)
    arbiter.overlap = overlap
    arbiter:set(a, b, faceNormal)

    arbiter:addContact(contactOnFace0, contactOnVert0)

    return arbiter
end

function CollideManager:featurePairPolyCircle(poly, circle)
    local result = {}

    local center = circle.center
    local cx = center[1]
    local cy = center[2]
    local radius = circle.radius

    local allOutside = true

    local min = -math.huge
    local faceIdx = -1
    local normal = nil

    for i = 1, poly.vertexCount do
        local n = poly.normals[i]
        local nx = n[1]
        local ny = n[2]
        local NdotP = n[3]

        local dist = nx * cx + ny * cy - NdotP
        if dist > radius then
            return false
        end
        if dist > min then
            allOutside = false
            min = dist
            faceIdx = i
            normal = n
        end
    end
    if allOutside then
        return false
    end

    if min >= math.huge then
        local vertIdx0 = faceIdx
        --local vertIdx1 = (vertIdx0 + 1) % poly.vertexCount
        local vertIdx0_add = vertIdx0 + 1
        local vertIdx1 = vertIdx0_add > poly.vertexCount and 1 or vertIdx0_add
        local v0 = poly.vertices[vertIdx0]
        local v1 = poly.vertices[vertIdx1]

        local dx = v1[1] - v0[1]
        local dy = v1[2] - v0[2]

        local disX0 = cx - v0[1]
        local disY0 = cy - v0[2]
        local disX1 = cx - v1[1]
        local disY1 = cy - v1[2]

        local p0 = disX0 * dx + disY0 * dy
        local p1 = -disX1 * dx - disY1 * dy
        local radiusSq = circle.radiusSq

        if p0 <= 0 then
            local disSq = disX0 * disX0 + disY0 * disY0
            if disSq > radiusSq then
                return false
            end
            local dis = math.sqrt(disSq)
            local nx = -disX0 / dis
            local ny = -disY0 / dis

            normal = { nx, ny }

            result.facePoly = circle
            result.vertPoly = poly
            result.minDist = min
            result.faceNormal = normal
            result.faceIdx = nil
            result.closest = v0
            result.closestIdx = vertIdx0

            return result
        elseif p1 <= 0 then
            local disSq = disX1 * disX1 + disY1 * disY1
            if disSq > radiusSq then
                return false
            end
            local dis = math.sqrt(disSq)
            local nx = -disX1 / dis
            local ny = -disY1 / dis

            normal = { nx, ny }

            result.facePoly = circle
            result.vertPoly = poly
            result.minDist = min
            result.faceNormal = normal
            result.faceIdx = nil
            result.closest = v1
            result.closestIdx = vertIdx1

            return result
        end
    end

    result.facePoly = poly
    result.vertPoly = circle
    result.minDist = min
    result.faceNormal = normal
    result.faceIdx = faceIdx

    local closest = { -normal[1] * radius + cx, -normal[2] * radius + cy }
    result.closest = closest
    result.closestIdx = nil

    return result
end

function CollideManager:polyPoly(bodyA, bodyB)
    local vertPoly, facePoly, faceIdx, faceNormal
    local vertV0, vertV1, faceV0, faceV1
    local face, vertex, fpcFlag

    local result = self:featurePairPolyPoly(bodyA, bodyB)
    if not result then
        return false
    end
    facePoly = result.facePoly
    vertPoly = result.vertPoly
    faceIdx = result.faceIdx
    faceNormal = result.faceNormal
    local vertexIdx = result.closestIdx
    local overlap = result.minDist

    local va = vertPoly.vertices[vertexIdx == 1 and vertPoly.vertexCount or vertexIdx - 1]
    local vb = vertPoly.vertices[vertexIdx]
    --local vc = vertPoly.vertices[(vertexIdx % vertPoly.vertexCount) + 1]
    local vertexIdx_add = vertexIdx + 1
    local vc = vertPoly.vertices[vertexIdx_add > vertPoly.vertexCount and 1 or vertexIdx_add]

    local vaDist = va[1] * faceNormal[1] + va[2] * faceNormal[2] - faceNormal[3]
    local vcDist = vc[1] * faceNormal[1] + vc[2] * faceNormal[2] - faceNormal[3]

    if vaDist < 0 or vcDist < 0 then
        if vaDist < vcDist then
            vertV0 = va
            vertV1 = vb
        else
            vertV0 = vb
            vertV1 = vc
        end
    end

    faceV0 = facePoly.vertices[faceIdx]
    --faceV1 = facePoly.vertices[(faceIdx % facePoly.vertexCount) + 1]
    local faceIdx_add = faceIdx + 1
    faceV1 = facePoly.vertices[faceIdx_add > facePoly.vertexCount and 1 or faceIdx_add]

    local contactOnFace0, contactOnVert0, contactOnFace1, contactOnVert1

    if vertV0 then
        contactOnFace0 = Polygon.projectPointToEdge(vertV0, faceV0, faceV1)
        contactOnVert0 = Polygon.projectPointToEdge(faceV1, vertV0, vertV1)

        contactOnFace1 = Polygon.projectPointToEdge(vertV1, faceV0, faceV1)
        contactOnVert1 = Polygon.projectPointToEdge(faceV0, vertV0, vertV1)
    else
        contactOnVert0 = vertPoly.vertices[vertexIdx]
        contactOnFace0 = Polygon.projectPointToEdge(contactOnVert0, faceV0, faceV1)
    end

    local a = facePoly.body or facePoly
    local b = vertPoly.body or vertPoly

    local arbiter = self:getArbiter(a, b)
    arbiter.overlap = overlap
    arbiter:set(a, b, faceNormal)

    arbiter:addContact(contactOnFace0, contactOnVert0)
    if contactOnFace1 then
        arbiter:addContact(contactOnFace1, contactOnVert1)
    end

    return arbiter
end

function CollideManager:featurePairPolyPoly(bodyA, bodyB)
    local result = {}

    local facePoly = bodyA
    local vertPoly = bodyB

    local faceLen = facePoly.vertexCount
    local vertLen = vertPoly.vertexCount

    local faceNormals = facePoly.normals
    local vertVertices = vertPoly.vertices

    local min1, closestIdx1, closest1, faceIdx1, normal1
    local first = true

    while true do
        local min = -math.huge
        local closestIdx = -1
        local closest = nil
        local faceIdx = -1
        local normal = nil
        local dist

        for i = 1, faceLen do
            local n = faceNormals[i]
            local nx = n[1]
            local ny = n[2]
            local NdotP = n[3]

            local allOutside = true

            local _min = 0
            local _closestIdx = -1
            local _closest, _faceIdx, _normal

            for j = 1, vertLen do
                local v = vertVertices[j]
                local vx = v[1]
                local vy = v[2]
                dist = nx * vx + ny * vy - NdotP
                if dist < _min then
                    allOutside = false
                    _min = dist
                    _closestIdx = j
                    _closest = v
                    _faceIdx = i
                    _normal = n
                end
            end

            if allOutside then
                return false
            end

            if _min > min then
                min = _min
                closestIdx = _closestIdx
                closest = _closest
                faceIdx = _faceIdx
                normal = _normal
            end
        end

        if not first then
            if min1 == min then
                local dx = closest1[1] - vertPoly.x
                local dy = closest1[2] - vertPoly.y
                local centerDist1 = dx * dx + dy * dy

                dx = closest[1] - facePoly.x
                dy = closest[2] - facePoly.y
                local centerDist2 = dx * dx + dy * dy

                if centerDist1 < centerDist2 then
                    min = min1 - 1
                end
            end
            if min1 > min then
                result.facePoly = bodyA
                result.vertPoly = bodyB
                result.minDist = min1
                result.closestIdx = closestIdx1
                result.closest = closest1
                result.faceIdx = faceIdx1
                result.faceNormal = normal1
            else
                result.facePoly = bodyB
                result.vertPoly = bodyA
                result.minDist = min
                result.closestIdx = closestIdx
                result.closest = closest
                result.faceIdx = faceIdx
                result.faceNormal = normal
            end
            break
        end

        min1 = min
        closestIdx1 = closestIdx
        closest1 = closest
        faceIdx1 = faceIdx
        normal1 = normal

        facePoly, vertPoly = bodyB, bodyA

        faceLen, vertLen = vertLen, faceLen

        faceNormals = facePoly.normals
        vertVertices = vertPoly.vertices

        first = false
    end

    return result
end

function CollideManager:compSingle(bodyA, bodyB)
    local comp, single
    if bodyB.shapeType == ShapeType.Comp then
        comp = bodyB
        single = bodyA
    else
        comp = bodyA
        single = bodyB
    end
    local shapes = comp.shapes

    local arbiter = false
    local boxB = single.aabb
    for i = 1, #shapes do
        local _shape = shapes[i]
        local boxA = _shape.aabb
        if boxA[1] < boxB[3] and boxA[3] > boxB[1] and boxA[2] < boxB[4] and boxA[4] > boxB[2] then
            local arb = self[self.collideMethodMap[single.shapeType | _shape.shapeType]](self, single, _shape)
            if arb then
                arbiter = arb
            end
        end
    end
    return arbiter
end

---@param bodyA Body
---@param bodyB Body
function CollideManager:compComp(bodyA, bodyB)
    local shapesA = bodyA.shapes
    local shapesB = bodyB.shapes

    local arbiter = false
    for i = 1, #shapesA do
        local _shapeA = shapesA[i]
        local boxA = _shapeA.aabb

        for j = 1, #shapesB do
            local _shapeB = shapesB[j]
            local boxB = _shapeB.aabb

            if boxA[1] < boxB[3] and boxA[3] > boxB[1] and boxA[2] < boxB[4] and boxA[4] > boxB[2] then
                local arb = self[self.collideMethodMap[_shapeA.shapeType | _shapeB.shapeType]](self, _shapeA, _shapeB)
                if arb then
                    arbiter = arb
                end
            end
        end
    end
    return arbiter
end

return CollideManager