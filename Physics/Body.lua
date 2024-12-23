---@class Body
local Body = Create("Body")
function Body:__ctor()
    self.bodyType = BodyType.Dynamic
    self.disabled = false
    self.sleeping = false
    self.sleepTime = 0
    self.autoSleep = true
    self.x = nil
    self.y = nil
    self.angle = 0
    self.velX = 0
    self.velY = 0
    self.velAng = 0
    self.forceX = 0
    self.forceY = 0
    self.torque = 0
    self.damping = 0.3
    self.dampingAng = 0.3
    self.mass = nil
    self.invMass = nil
    self.inertia = nil
    self.invInertia = nil
    self.fixedRotation = false -- true ===> inertia=0
    self.ignoreG = false
    self.gravityX = 0
    self.gravityY = 0
    self.friction = 0.2 --0.5
    self.restitution = 0.25
    self.density = 0.5
end

function Body:__delete()
end

function Body:init()
    self.aabb = { 0, 0, 0, 0 }
    self.last = {}
end

function Body:saveStatus()
    self.last.velX = self.velX
    self.last.velY = self.velY
    self.last.velY = self.velY
    self.last.x = self.x
    self.last.y = self.y
end

function Body:setMass(mass)
    local invMass
    if (not mass and mass ~= 0) then
        mass = self.density * self.area
    end
    if (mass <= 0 or mass == MathConst.Infinity) then
        mass = MathConst.Infinity
        invMass = 0
        self.inertia = MathConst.Infinity
        self.invInertia = 0
    else
        invMass = 1 / mass
    end
    self.mass = mass
    self.invMass = invMass
end

function Body:setInertia(inertia)
    local invInertia
    if (not inertia and inertia ~= 0) then
        self.inertiaFactor = self.inertiaFactor or (self.mass * 2) * self.area
        inertia = self.density * self.inertiaFactor / 12
    end
    if (inertia <= 0 or inertia == MathConst.Infinity or self.mass <= 0 or self.mass == MathConst.Infinity) then
        inertia = MathConst.Infinity
        invInertia = 0
    else
        invInertia = 1 / inertia
    end
    self.inertia = inertia
    self.invInertia = invInertia
end

function Body:setPos(x, y)
    local _x, _y = x, y
    --if self.clampAABB then
    --    local aabb = self.aabb
    --    local halfW = (aabb[3] - aabb[1]) * 0.5
    --    local halfH = (aabb[4] - aabb[2]) * 0.5
    --    _x = math.max(self.clampAABB[1] - halfW, math.min(self.clampAABB[3] + halfW, x))
    --    _y = math.max(self.clampAABB[2] - halfH, math.min(self.clampAABB[4] + halfH, y))
    --end
    self.x = _x
    self.y = _y
end

function Body:setAngle(angle)
    self.angle = angle
end

function Body:setForce(x, y)
    self.forceX = x
    self.forceY = y
end

function Body:getVelSq()
    return self.velX * self.velX + self.velY * self.velY
end

function Body:applyImpulse(x, y, point)
    if (self.bodyType ~= BodyType.Dynamic) then
        return
    end
    self:awake()
    self.velX = self.velX + self.invMass * x
    self.velY = self.velY + self.invMass * y
    if (point) then
        self.velAng = self.velAng + ((point[1] - self.x) * y - (point[2] - self.y) * x) * self.invInertia
    end
end

function Body:applyForce(x, y, point)
    if (self.bodyType ~= BodyType.Dynamic) then
        return
    end
    self:awake()
    self.forceX = self.forceX + x
    self.forceY = self.forceY + y

    if (point) then
        local torque = ((point[1] - self.x) * y - (point[2] - self.y) * x)
        self.torque = self.torque + torque
    end
end

function Body:applyForceTorque(x, y, point)
    if (self.bodyType ~= BodyType.Dynamic) then
        return
    end
    self:awake()
    local torque = ((point[1] - self.x) * y - (point[2] - self.y) * x)
    self.torque = self.torque + torque
end

function Body:applyTorque(torque)
    if (self.bodyType ~= BodyType.Dynamic) then
        return
    end
    self:awake()
    self.torque = self.torque + torque
end

function Body:applyDamping(timeStep)
    local d = 1 - timeStep * self.damping
    d = math.min(1, math.max(0, d))
    self.velX = self.velX * d
    self.velY = self.velY * d
end

function Body:applyDampingAng(timeStep)
    local d = 1 - timeStep * self.dampingAng
    d = math.min(1, math.max(0, d))
    self.velAng = self.velAng * d
end

function Body:awake()
    self.sleeping = false
    self.sleepTime = 0
end

function Body:update(timeStep)
end

function Body:integrate(timeStep)
    self.velX = self.velX + (self.forceX * self.invMass) * timeStep
    self.velY = self.velY + (self.forceY * self.invMass) * timeStep
    -- self:integrateVel(timeStep)
    -- self:integrateVelAngle(timeStep);
    self:integrateAngle(timeStep)
    self:integratePos(timeStep)
end

function Body:integrateVel(timeStep)
    self.lastVelX = self.velX
    self.lastVelY = self.velY
    self.velX = self.velX + (self.gravityX + self.forceX * self.invMass) * timeStep
    self.velY = self.velY + (self.gravityY + self.forceY * self.invMass) * timeStep
    if (self.damping ~= 0) then
        local d = math.min(1, math.max(0, 1 - self.damping * timeStep))
        self.velX = self.velX * d
        self.velY = self.velY * d
    end
end

function Body:integrateVelAngle(timeStep)
    self.velAng = self.velAng + (self.torque * self.invMass) * timeStep
    if (self.dampingAng ~= 0) then
        self.velAng = self.velAng * math.min(1, math.max(0, 1 - self.dampingAng * timeStep));
    end
end

local velAngEpsilon = 0.2
function Body:integrateAngle(timeStep)
    --避免抖动
    if math.abs(self.velAng) < velAngEpsilon then
        return
    end
    self:setAngle(self.angle + self.velAng * timeStep)
end

local velEpsilon = 0.01
function Body:integratePos(timeStep)
    --避免抖动
    local x = (math.abs(self.velX) < velEpsilon) and self.x or (self.x + self.velX * timeStep)
    local y = (math.abs(self.velY) < velEpsilon) and self.y or (self.y + self.velY * timeStep)
    self:setPos(x, y)
end

local drawline = CS.UnityEngine.Debug.DrawLine
function Body:drawaabb(color)
    local box = self.aabb
    local minX, maxX = box[1], box[3]
    local minY, maxY = box[2], box[4]
    local c = color or (self.sleeping and Color.blue or Color.red)
    drawline(Vector3(minX, minY), Vector3(maxX, minY), c)
    drawline(Vector3(maxX, minY), Vector3(maxX, maxY), c)
    drawline(Vector3(maxX, maxY), Vector3(minX, maxY), c)
    drawline(Vector3(minX, maxY), Vector3(minX, minY), c)
end

return Body