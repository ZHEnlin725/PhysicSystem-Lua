local base = Joint
---@class DistanceJoint:Joint
local DistanceJoint = CreateClass("DistanceJoint", base)

function DistanceJoint:__ctor(_, _, anchor1, anchor2)
    self.type = Joint.TYPE_DISTANCE

    -- Local anchor points
    self.anchor1 = self.body1:getLocalPoint(anchor1)
    self.anchor2 = self.body2:getLocalPoint(anchor2)

    -- Rest length
    self.restLength = Vec2.dist(anchor1, anchor2)

    -- Soft constraint coefficients
    self.gamma = 0
    self.beta_c = 0

    -- Spring coefficients
    self.frequencyHz = 0
    self.dampingRatio = 0

    -- Accumulated impulse
    self.lambda_acc = 0
end

function DistanceJoint:setWorldAnchor1(anchor1)
    local old_anchor1 = self.anchor1
    self.anchor1 = self.body1:getLocalPoint(anchor1)
    local anchor2 = self:getWorldAnchor2()
    self.restLength = Vec2.dist(anchor1, anchor2)
    if old_anchor1 ~= self.anchor1 then
        old_anchor1:Delete()
    end
    anchor2:Delete()
end

function DistanceJoint:setWorldAnchor2(anchor2)
    local old_anchor2 = self.anchor2
    self.anchor2 = self.body2:getLocalPoint(anchor2)
    local anchor1 = self:getWorldAnchor1()
    self.restLength = Vec2.dist(anchor2, anchor1)
    if old_anchor2 ~= self.anchor2 then
        old_anchor2:Delete()
    end
    anchor1:Delete()

end

function DistanceJoint:setSpringDampingRatio(value)
    self.dampingRatio = value
end

function DistanceJoint:setSpringFrequencyHz(value)
    self.frequencyHz = value
end

function DistanceJoint:initSolver(timeStep, warmStarting)
    local body1 = self.body1
    local body2 = self.body2

    local sub1 = Vec2.sub(self.anchor1, body1.centroid)
    local sub2 = Vec2.sub(self.anchor2, body2.centroid)
    local old_r1 = self.r1
    local old_r2 = self.r2
    self.r1 = Vec2.rotate(sub1, body1.a)
    self.r2 = Vec2.rotate(sub2, body2.a)

    -- Delta vector between two world anchors
    local lhs = Vec2.add(body2.p, self.r2)
    local rhs = Vec2.add(body1.p, self.r1)
    local d = Vec2.sub(lhs, rhs)

    -- Distance between two anchors
    local dist = d:length()

    local old_u = self.u
    -- Unit delta vector
    if (dist > Joint.LINEAR_SLOP) then
        self.u = Vec2.scale(d, 1 / dist)
    else
        self.u = Vec2.zero
    end

    -- s1, s2
    self.s1 = Vec2.cross(self.r1, self.u)
    self.s2 = Vec2.cross(self.r2, self.u)

    -- invEM = J * invM * JT
    local em_inv = body1.m_inv + body2.m_inv + body1.i_inv * self.s1 * self.s1 + body2.i_inv * self.s2 * self.s2
    self.em = em_inv == 0 and 0 or 1 / em_inv

    -- Compute soft constraint parameters
    if (self.frequencyHz > 0) then
        -- Frequency
        local omega = 2 * math.pi * self.frequencyHz

        -- Spring stiffness
        local k = self.em * omega * omega

        -- Damping coefficient
        local c = self.em * 2 * self.dampingRatio * omega

        -- Soft constraint formulas
        -- gamma and beta are divided by timeStep to reduce computation
        self.gamma = (c + k * timeStep) * timeStep
        self.gamma = self.gamma == 0 and 0 or 1 / self.gamma
        local beta = timeStep * k * self.gamma

        -- Position constraint
        local pc = dist - self.restLength
        self.beta_c = beta * pc

        -- invEM = invEM + gamma * I (to reduce calculation)
        em_inv = em_inv + self.gamma
        self.em = em_inv == 0 and 0 or 1 / em_inv

    else
        self.gamma = 0
        self.beta_c = 0
    end

    if (warmStarting) then
        -- linearImpulse = JT * lambda
        local impulse = Vec2.scale(self.u, self.lambda_acc)

        -- Apply cached constraint impulses
        -- V += JT * lambda * invM
        body1.v:mad(impulse, -body1.m_inv)
        body1.w = body1.w - self.s1 * self.lambda_acc * body1.i_inv

        body2.v:mad(impulse, body2.m_inv)
        body2.w = body2.w + self.s2 * self.lambda_acc * body2.i_inv

        impulse:Delete()
    else
        self.lambda_acc = 0
    end

    sub1:Delete()
    sub2:Delete()
    if old_r1 then
        old_r1:Delete()
    end
    if old_r2 then
        old_r2:Delete()
    end
    if old_u and old_u ~= Vec2.zero then
        old_u:Delete()
    end
    lhs:Delete()
    rhs:Delete()
    d:Delete()
end

function DistanceJoint:solveVelocityConstraints()
    local body1 = self.body1
    local body2 = self.body2

    -- Compute lambda for velocity constraint
    -- Solve J * invM * JT * lambda = -(J * V + beta * C + gamma * (lambda_acc + lambda))
    local sub = Vec2.sub(body2.v, body1.v)
    local cdot = self.u:dot(sub) + self.s2 * body2.w - self.s1 * body1.w
    local soft = self.beta_c + self.gamma * self.lambda_acc
    local lambda = -self.em * (cdot + soft)

    -- Accumulate lambda
    self.lambda_acc = self.lambda_acc + lambda

    -- linearImpulse = JT * lambda
    local impulse = Vec2.scale(self.u, lambda)

    -- Apply constraint impulses
    -- V += JT * lambda * invM
    body1.v:mad(impulse, -body1.m_inv)
    body1.w = body1.w - self.s1 * lambda * body1.i_inv

    body2.v:mad(impulse, body2.m_inv)
    body2.w = body2.w + self.s2 * lambda * body2.i_inv

    sub:Delete()
    impulse:Delete()
end

function DistanceJoint:solvePositionConstraints()
    if self.frequencyHz > 0 then
        return true
    end

    local body1 = self.body1
    local body2 = self.body2
    -- Transformed r1, r2
    local sub1 = Vec2.sub(self.anchor1, body1.centroid)
    local sub2 = Vec2.sub(self.anchor2, body2.centroid)
    local r1 = Vec2.rotate(sub1, body1.a)
    local r2 = Vec2.rotate(sub2, body2.a)

    -- Delta vector between two anchors
    local lhs = Vec2.add(body2.p, r2)
    local rhs = Vec2.add(body1.p, r1)
    local d = Vec2.sub(lhs, rhs)

    -- Distance between two anchors
    local dist = d:length()

    -- Unit delta vector
    local u = Vec2.scale(d, 1 / dist)

    -- Position constraint
    local c = dist - self.restLength
    local correction = math.clamp(c, -Joint.MAX_LINEAR_CORRECTION, Joint.MAX_LINEAR_CORRECTION)

    -- Compute lambda for correction
    -- Solve J * invM * JT * lambda = -C / dt
    local s1 = Vec2.cross(r1, u)
    local s2 = Vec2.cross(r2, u)
    local em_inv = body1.m_inv + body2.m_inv + body1.i_inv * s1 * s1 + body2.i_inv * s2 * s2
    local lambda_dt = em_inv == 0 and 0 or -correction / em_inv

    -- Apply constraint impulses
    -- impulse = JT * lambda
    -- X += impulse * invM * dt
    local impulse_dt = Vec2.scale(u, lambda_dt)

    body1.p:mad(impulse_dt, -body1.m_inv)
    body1.a = body1.a - s1 * lambda_dt * body1.i_inv

    body2.p:mad(impulse_dt, body2.m_inv)
    body2.a = body2.a + s2 * lambda_dt * body2.i_inv


    sub1:Delete()
    sub2:Delete()
    r1:Delete()
    r2:Delete()
    lhs:Delete()
    rhs:Delete()
    d:Delete()
    u:Delete()
    impulse_dt:Delete()

    return math.abs(c) < Joint.LINEAR_SLOP
end

function DistanceJoint:draw(dl, color)
    local c = color or Color.white
    local anchor1 = self.body1:getWorldPoint(self.anchor1)
    local anchor2 = self.body2:getWorldPoint(self.anchor2)
    dl(Vector3(anchor1.x, anchor1.y), Vector3(anchor2.x, anchor2.y), c)

    anchor1:Delete()
    anchor2:Delete()
end

return DistanceJoint