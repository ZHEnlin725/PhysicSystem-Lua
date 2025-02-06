---@class Body
local Body = CreateClass("Body")

BODY_TYPE = {
    STATIC = 1,
    DYNAMIC = 2,
    KINETIC = 3,
}

local id_alloc
function Body:__ctor(type, pos, angle)
    id_alloc = id_alloc or 0
    self.id = id_alloc
    id_alloc = id_alloc + 1

    -- Identifier
    --self.name = "body_" .. tostring(self.id)

    -- STATIC or DYNAMIC
    self.type = type

    -- Default values
    pos = pos or Vec2.New(0, 0)
    angle = angle or 0

    -- Local to world transform
    ---@type Transform
    self.xf = Trans.New(pos, angle)

    -- Local center of mass
    ---@type Vec2
    self.centroid = Vec2.New(0, 0)

    -- World position of centroid
    ---@type Vec2
    self.p = Vec2.New(pos.x, pos.y)

    -- Velocity
    ---@type Vec2
    self.v = Vec2.New(0, 0)

    -- Force
    ---@type Vec2
    self.f = Vec2.New(0, 0)

    -- Orientation (angle)
    self.a = angle

    -- Angular velocity
    self.w = 0

    -- Torque
    self.t = 0

    -- Linear damping
    self.linearDamping = 0

    -- Angular damping
    self.angularDamping = 0

    -- Sleep time
    self.sleepTime = 0

    -- Awaked flag
    self.awaked = false

    -- Shape list for self body
    ---@type Shape[]
    self.shapeArr = {}

    -- Joint hash for self body
    ---@type Joint[]
    self.jointArr = {}
    self.jointHash = {}

    -- Bounds of all shapes
    ---@type Bounds
    self.bounds = Bounds.New()

    self.fixedRotation = false

    self.layer = 0x0001
    self.layerMask = 0xFFFF

    self.stepCount = 0
end

---@type Body
function Body:duplicate()
    local body = Body.New(self.type, self.xf.t, self.a)
    for _, shape in ipairs(self.shapeArr) do
        body:addShape(shape:duplicate())
    end
    body:resetMassData()
    return body
end

function Body:addShape(shape)
    shape.body = self
    table.insert(self.shapeArr, shape)
end

function Body:removeShape(shape)
    for i, v in ipairs(self.shapeArr) do
        if v == shape then
            local removed = table.remove(self.shapeArr, i)
            removed.body = nil
            break
        end
    end
end

function Body:setMass(mass)
    self.m = mass
    self.m_inv = mass > 0 and 1 / mass or 0
end

function Body:setInertia(inertia)
    self.i = inertia
    self.i_inv = inertia > 0 and 1 / inertia or 0
end

function Body:setTransform(pos, rad)
    self.xf:set(pos, rad)
    local old_p = self.p
    self.p = self.xf:transform(self.centroid)
    self.a = rad

    old_p:Delete()
end

function Body:syncTransform()
    self.xf:setRotation(self.a)
    local rotate = self.xf:rotate(self.centroid)
    local sub = Vec2.sub(self.p, rotate)
    self.xf:setPosition(sub)

    sub:Delete()
    rotate:Delete()
end

function Body:getWorldPoint(localPoint)
    return self.xf:transform(localPoint)
end

function Body:getWorldVector(localVector)
    return self.xf:rotate(localVector)
end

function Body:getLocalPoint(worldPoint)
    return self.xf:invTransform(worldPoint)
end

function Body:getLocalVector(worldVector)
    return self.xf:invRotate(worldVector)
end

function Body:setFixedRotation(flag)
    self.fixedRotation = flag
    self:resetMassData()
end

function Body:resetMassData()
    self.centroid:set(0, 0)
    self.m = 0
    self.m_inv = 0
    self.i = 0
    self.i_inv = 0

    if not self:isDynamic() then
        self.p = self.xf:transform(self.centroid)
        return
    end

    local totalMassCentroid = Vec2.New(0, 0)
    local totalMass = 0
    local totalInertia = 0

    local numShapes = self.shapeArr and #self.shapeArr or 0
    for i = 1, numShapes do
        local shape = self.shapeArr[i]
        local centroid = shape:centroid()
        local mass = shape:area() * shape.density
        local inertia = shape:inertia(mass)

        totalMassCentroid:mad(centroid, mass)
        totalMass = totalMass + mass
        totalInertia = totalInertia + inertia
    end

    local scale = Vec2.scale(totalMassCentroid, 1 / totalMass)
    self.centroid:copy(scale)
    self:setMass(totalMass)

    if not self.fixedRotation then
        self:setInertia(totalInertia - totalMass * Vec2.dot(self.centroid, self.centroid))
    end
    --console.log("mass = " + self.m + " inertia = " + self.i);

    -- Move center of mass
    local old_p = self.p
    self.p = self.xf:transform(self.centroid)

    -- Update center of mass velocity ??
    local sub = Vec2.sub(self.p, old_p)
    local perp = Vec2.perp(sub)
    self.v:mad(perp, self.w)

    totalMassCentroid:Delete()
    scale:Delete()
    sub:Delete()
    perp:Delete()
    old_p:Delete()
end

function Body:resetJointAnchors()
    for _, joint in ipairs(self.jointArr) do
        local anchor1 = joint:getWorldAnchor1()
        local anchor2 = joint:getWorldAnchor2()

        joint:setWorldAnchor1(anchor1)
        joint:setWorldAnchor2(anchor2)
    end
end

function Body:cacheData()
    self.bounds:reset()

    for _, shape in ipairs(self.shapeArr) do
        shape:cacheData(self.xf)
        self.bounds:encapsulateBounds(shape.bounds)
    end
end

function Body:updateVelocity(gravity, timeStep, damping)
    local mad = Vec2.mad(self.ignoreG and Vec2.zero or gravity, self.f, self.m_inv)
    self.v:mad(mad, timeStep)
    self.w = self.w + self.t * self.i_inv * timeStep

    -- Apply damping.
    -- ODE: dv/timeStep + c * v = 0
    -- Solution: v(t) = v0 * exp(-c * t)
    -- Time step: v(t + timeStep) = v0 * exp(-c * (t + timeStep)) = v0 * exp(-c * t) * exp(-c * timeStep) = v * exp(-c * timeStep)
    -- v2 = exp(-c * timeStep) * v1
    -- Taylor expansion:
    -- v2 = (1.0f - c * timeStep) * v1
    self.v:scale(math.clamp(1 - timeStep * (damping + self.linearDamping), 0, 1))
    self.w = self.w * math.clamp(1 - timeStep * (damping + self.angularDamping), 0, 1)

    self.f:set(0, 0)
    self.t = 0

    mad:Delete()
end

function Body:updatePosition(timeStep)
    self.p:mad(self.v, timeStep)
    self.a = self.a + self.w * timeStep
end

function Body:resetForce()
    self.f:set(0, 0)
    self.t = 0
end

function Body:applyForce(force, p)
    if not self:isDynamic() then
        return
    end
    if not self:isAwake() then
        self:awake(true)
    end
    self.f:add(force)
    if p then
        local sub = Vec2.sub(p, self.p)
        self.t = self.t + Vec2.cross(sub, force)

        sub:Delete()
    end
end

function Body:applyTorque(torque)
    if not self:isDynamic() then
        return
    end
    if not self:isAwake() then
        self:awake(true)
    end

    self.t = self.t + torque
end

function Body:applyLinearImpulse(impulse, p)
    if not self:isDynamic() then
        return
    end
    if not self:isAwake() then
        self:awake(true)
    end
    self.v:mad(impulse, self.m_inv)
    local sub = Vec2.sub(p, self.p)
    self.w = self.w + Vec2.cross(sub, impulse) * self.i_inv

    sub:Delete()
end

function Body:applyAngularImpulse(impulse)
    if not self:isDynamic() then
        return
    end
    if not self:isAwake() then
        self:awake(true)
    end
    self.w = self.w + impulse * self.i_inv
end

function Body:kineticEnergy()
    local vsq = self.v:dot(self.v)
    local wsq = self.w * self.w
    return 0.5 * (self.m * vsq + self.i * wsq)
end

function Body:isDynamic()
    return self.type == BODY_TYPE.DYNAMIC
end

function Body:isStatic()
    return self.type == BODY_TYPE.STATIC
end

function Body:isKinetic()
    return self.type == BODY_TYPE.KINETIC
end

function Body:isAwake()
    return self.awaked
end

function Body:awake(flag)
    self.awaked = flag
    if flag then
        self.sleepTime = 0
    else
        self.v:set(0, 0)
        self.w = 0
        self.f:set(0, 0)
        self.t = 0
    end
end

function Body:isCollidable(other)
    if self == other then
        return false
    end

    if not self:isDynamic() and not other:isDynamic() then
        return false
    end

    if (self.layerMask & other.layer == 0) or (other.layerMask & self.layer == 0) then
        return false
    end

    for _, joint in ipairs(self.jointArr) do
        if not joint.collideConnected and other.jointHash[joint.id] ~= nil then
            return false
        end
    end

    return true
end

return Body