local base = Joint
---@class RevoluteJoint:Joint
local RevoluteJoint = CreateClass("RevoluteJoint", base)

function RevoluteJoint:__ctor(body1, body2, anchor)
    self.type = Joint.TYPE_REVOLUTE

    self.anchor1 = self.body1:getLocalPoint(anchor)
    self.anchor2 = self.body2:getLocalPoint(anchor)

    -- Initial angle difference
    self.refAngle = body2.a - body1.a

    -- Accumulated lambda
    self.lambda_acc = Vec3.New(0, 0, 0)
    self.motorLambda_acc = 0

    -- Angle limit
    self.limitEnabled = false
    self.limitLowerAngle = 0
    self.limitUpperAngle = 0
    self.limitState = Joint.LIMIT_STATE_INACTIVE

    -- Motor
    self.motorEnabled = false
    self.motorSpeed = 0
    self.maxMotorTorque = 0
end

function RevoluteJoint:setLimits(lower, upper)
    self.limitLowerAngle = lower
    self.limitUpperAngle = upper
end

function RevoluteJoint:enableLimit(flag)
    self.limitEnabled = flag
end

function RevoluteJoint:setMaxMotorTorque(torque)
    self.maxMotorTorque = torque
end

function RevoluteJoint:setMotorSpeed(speed)
    self.motorSpeed = speed
end

function RevoluteJoint:enableMotor(flag)
    self.motorEnabled = flag
end

function RevoluteJoint:setWorldAnchor1(anchor1)
    local old_anchor1 = self.anchor1
    local old_anchor2 = self.anchor2
    self.anchor1 = self.body1:getLocalPoint(anchor1)
    self.anchor2 = self.body2:getLocalPoint(anchor1)
    if old_anchor1 ~= self.anchor1 then
        old_anchor1:Delete()
    end
    if old_anchor2 ~= self.anchor2 then
        old_anchor2:Delete()
    end
end

function RevoluteJoint:setWorldAnchor2(anchor2)
    local old_anchor1 = self.anchor1
    local old_anchor2 = self.anchor2
    self.anchor1 = self.body1:getLocalPoint(anchor2)
    self.anchor2 = self.body2:getLocalPoint(anchor2)
    if old_anchor1 ~= self.anchor1 then
        old_anchor1:Delete()
    end
    if old_anchor2 ~= self.anchor2 then
        old_anchor2:Delete()
    end
end

function RevoluteJoint:initSolver(dt, warmStarting)
    local body1 = self.body1
    local body2 = self.body2

    self.maxImpulse = self.maxForce * dt

    if (not self.motorEnabled) then
        self.motorLambda_acc = 0
    else
        self.maxMotorImpulse = self.maxMotorTorque * dt
    end

    if (self.limitEnabled) then
        local da = body2.a - body1.a - self.refAngle
        if (math.abs(self.limitUpperAngle - self.limitLowerAngle) < Joint.ANGULAR_SLOP) then
            self.limitState = Joint.LIMIT_STATE_EQUAL_LIMITS
        elseif (da <= self.limitLowerAngle) then
            if (self.limitState ~= Joint.LIMIT_STATE_AT_LOWER) then
                self.lambda_acc.z = 0
            end
            self.limitState = Joint.LIMIT_STATE_AT_LOWER
        elseif (da >= self.limitUpperAngle) then
            if (self.limitState ~= Joint.LIMIT_STATE_AT_UPPER) then
                self.lambda_acc.z = 0
            end
            self.limitState = Joint.LIMIT_STATE_AT_UPPER
        else
            self.limitState = Joint.LIMIT_STATE_INACTIVE
            self.lambda_acc.z = 0
        end
    else
        self.limitState = Joint.LIMIT_STATE_INACTIVE
    end

    local sub1 = Vec2.sub(self.anchor1, body1.centroid)
    local sub2 = Vec2.sub(self.anchor2, body2.centroid)
    local old_r1 = self.r1
    local old_r2 = self.r2

    self.r1 = body1.xf:rotate(sub1)
    self.r2 = body2.xf:rotate(sub2)

    -- invEM = J * invM * JT
    local sum_m_inv = body1.m_inv + body2.m_inv
    local r1 = self.r1
    local r2 = self.r2
    local r1x_i = r1.x * body1.i_inv
    local r1y_i = r1.y * body1.i_inv
    local r2x_i = r2.x * body2.i_inv
    local r2y_i = r2.y * body2.i_inv
    local k11 = sum_m_inv + r1.y * r1y_i + r2.y * r2y_i
    local k12 = -r1.x * r1y_i - r2.x * r2y_i
    local k13 = -r1y_i - r2y_i
    local k22 = sum_m_inv + r1.x * r1x_i + r2.x * r2x_i
    local k23 = r1x_i + r2x_i
    local k33 = body1.i_inv + body2.i_inv
    local old_em_inv = self.em_inv
    self.em_inv = Matrix3x3.New(k11, k12, k13, k12, k22, k23, k13, k23, k33)

    self.em2 = k33 == 0 and 0 or 1 / k33

    if warmStarting then
        -- Apply cached constraint impulses
        -- V += JT * lambda
        local lambda_xy = Vec2.New(self.lambda_acc.x, self.lambda_acc.y)
        local lambda_z = self.lambda_acc.z + self.motorLambda_acc

        body1.v:mad(lambda_xy, -body1.m_inv)
        body1.w = body1.w - (Vec2.cross(self.r1, lambda_xy) + lambda_z) * body1.i_inv

        body2.v:mad(lambda_xy, body2.m_inv)
        body2.w = body2.w + (Vec2.cross(self.r2, lambda_xy) + lambda_z) * body2.i_inv

        lambda_xy:Delete()
    else
        self.lambda_acc:set(0, 0, 0)
        self.motorLambda_acc = 0
    end

    if old_r1 then
        old_r1:Delete()
    end

    if old_r2 then
        old_r2:Delete()
    end
    sub1:Delete()
    sub2:Delete()
    if old_em_inv then
        old_em_inv:Delete()
    end
end

function RevoluteJoint:solveVelocityConstraints()
    local body1 = self.body1
    local body2 = self.body2

    if (self.motorEnabled and self.limitState ~= Joint.LIMIT_STATE_EQUAL_LIMITS) then
        -- Compute motor impulse
        local cdot = body2.w - body1.w - self.motorSpeed
        local lambda = -self.em2 * cdot
        local motorLambdaOld = self.motorLambda_acc
        self.motorLambda_acc = math.clamp(self.motorLambda_acc + lambda, -self.maxMotorImpulse, self.maxMotorImpulse)
        lambda = self.motorLambda_acc - motorLambdaOld

        -- Apply motor constraint impulses
        body1.w = body1.w - lambda * body1.i_inv
        body2.w = body2.w + lambda * body2.i_inv
    end

    if (self.limitEnabled and self.limitState ~= Joint.LIMIT_STATE_INACTIVE) then
        -- Compute lambda for velocity constraint
        -- Solve J * invM * JT * lambda = -J * V
        -- in 2D: cross(w, r) = perp(r) * w
        local v1 = Vec2.mad(body1.v, Vec2.perp(self.r1), body1.w)
        local v2 = Vec2.mad(body2.v, Vec2.perp(self.r2), body2.w)
        local cdot1 = Vec2.sub(v2, v1)
        local cdot2 = body2.w - body1.w
        local cdot = Vec3.fromVec2(cdot1, cdot2)
        local lambda = self.em_inv:solve(cdot:neg())

        if (self.limitState == Joint.LIMIT_STATE_EQUAL_LIMITS) then
            ---- Accumulate lambda
            self.lambda_acc:add(lambda)
        elseif (self.limitState == Joint.LIMIT_STATE_AT_LOWER or self.limitState == Joint.LIMIT_STATE_AT_UPPER) then
            ---- Accumulated new lambda.z
            local newLambda_z = self.lambda_acc.z + lambda.z

            local lowerLimited = self.limitState == Joint.LIMIT_STATE_AT_LOWER and newLambda_z < 0
            local upperLimited = self.limitState == Joint.LIMIT_STATE_AT_UPPER and newLambda_z > 0

            if (lowerLimited or upperLimited) then
                -- Modify last equation to get lambda_acc.z to 0
                -- That is, lambda.z have to be equal -lambda_acc.z
                -- rhs = -J * V - (K_13, K_23, K_33) * (lambda.z + lambda_acc.z)
                -- Solve J * invM * JT * reduced_lambda = rhs
                local rhs = Vec2.add(cdot1, Vec2.scale(Vec2.New(self.em_inv._13, self.em_inv._23), newLambda_z))
                local reduced = self.em_inv:solve2x2(rhs:neg())
                lambda.x = reduced.x
                lambda.y = reduced.y
                lambda.z = -self.lambda_acc.z

                -- Accumulate lambda
                self.lambda_acc.x = self.lambda_acc.x + lambda.x
                self.lambda_acc.y = self.lambda_acc.y + lambda.y
                self.lambda_acc.z = 0

                reduced:Delete()
            else
                ---- Accumulate lambda
                self.lambda_acc:add(lambda)
            end
        end

        ---- Apply constraint impulses
        ---- V += JT * lambda * invM
        local lambda_xy = Vec2.New(lambda.x, lambda.y)

        body1.v:mad(lambda_xy, -body1.m_inv)
        body1.w = body1.w - (Vec2.cross(self.r1, lambda_xy) + lambda.z) * body1.i_inv

        body2.v:mad(lambda_xy, body2.m_inv)
        body2.w = body2.w + (Vec2.cross(self.r2, lambda_xy) + lambda.z) * body2.i_inv

        v1:Delete()
        v2:Delete()

        cdot1:Delete()
        lambda_xy:Delete()

        cdot:Delete()
        lambda:Delete()
    else
        -- Solve point-to-point constraint
        -- Compute lambda for velocity constraint
        -- Solve J1 * invM * J1T * lambda = -J1 * V
        -- in 2D: cross(w, r) = perp(r) * w
        local perp1 = Vec2.perp(self.r1)
        local perp2 = Vec2.perp(self.r2)
        local v1 = Vec2.mad(body1.v, perp1, body1.w)
        local v2 = Vec2.mad(body2.v, perp2, body2.w)
        local cdot = Vec2.sub(v2, v1)
        local lambda = self.em_inv:solve2x2(cdot:neg())

        -- Accumulate lambda
        self.lambda_acc:add(Vec3.fromVec2(lambda, 0))

        -- Apply constraint impulses
        -- V += J1T * lambda * invM
        body1.v:mad(lambda, -body1.m_inv)
        body1.w = body1.w - Vec2.cross(self.r1, lambda) * body1.i_inv

        body2.v:mad(lambda, body2.m_inv)
        body2.w = body2.w + Vec2.cross(self.r2, lambda) * body2.i_inv

        perp1:Delete()
        perp2:Delete()
        v1:Delete()
        v2:Delete()
        cdot:Delete()
        lambda:Delete()
    end
end

function RevoluteJoint:solvePositionConstraints()
    local body1 = self.body1
    local body2 = self.body2

    local angularError = 0
    local positionError = 0

    -- Solve limit constraint
    if (self.limitEnabled and self.limitState ~= Joint.LIMIT_STATE_INACTIVE) then
        local da = body2.a - body1.a - self.refAngle

        -- angular lambda = -EM * C / dt
        local angularImpulseDt = 0

        if (self.limitState == Joint.LIMIT_STATE_EQUAL_LIMITS) then
            local c = math.clamp(da - self.limitLowerAngle, -Joint.MAX_ANGULAR_CORRECTION, Joint.MAX_ANGULAR_CORRECTION)

            angularError = math.abs(c)
            angularImpulseDt = -self.em2 * c

        elseif (self.limitState == Joint.LIMIT_STATE_AT_LOWER) then
            local c = da - self.limitLowerAngle

            angularError = -c
            c = math.clamp(c + Joint.ANGULAR_SLOP, -Joint.MAX_ANGULAR_CORRECTION, 0)
            angularImpulseDt = -self.em2 * c
        elseif (self.limitState == Joint.LIMIT_STATE_AT_UPPER) then
            local c = da - self.limitUpperAngle

            angularError = c
            c = math.clamp(c - Joint.ANGULAR_SLOP, 0, Joint.MAX_ANGULAR_CORRECTION)
            angularImpulseDt = -self.em2 * c
        end

        body1.a = body1.a - angularImpulseDt * body1.i_inv
        body2.a = body2.a + angularImpulseDt * body2.i_inv
    end

    -- Solve point-to-point constraint
    -- Transformed r1, r2
    local sub1 = Vec2.sub(self.anchor1, body1.centroid)
    local sub2 = Vec2.sub(self.anchor2, body2.centroid)
    local r1 = Vec2.rotate(sub1, body1.a)
    local r2 = Vec2.rotate(sub2, body2.a)

    -- Position constraint
    local lhs = Vec2.add(body2.p, r2)
    local rhs = Vec2.add(body1.p, r1)
    local c = Vec2.sub(lhs, rhs)
    local correction = Vec2.truncate(c, Joint.MAX_LINEAR_CORRECTION)
    positionError = correction:length()

    -- Compute lambda for position constraint
    -- Solve J1 * invM * J1T * lambda = -C / dt
    local sum_m_inv = body1.m_inv + body2.m_inv
    local r1y_i = r1.y * body1.i_inv
    local r2y_i = r2.y * body2.i_inv
    local k11 = sum_m_inv + r1.y * r1y_i + r2.y * r2y_i
    local k12 = -r1.x * r1y_i - r2.x * r2y_i
    local k22 = sum_m_inv + r1.x * r1.x * body1.i_inv + r2.x * r2.x * body2.i_inv
    local em_inv = Matrix2x2.New(k11, k12, k12, k22)
    local lambda_dt = em_inv:solve(correction:neg())

    -- Apply constraint impulses
    -- impulse = J1T * lambda
    -- X += impulse * invM * dt
    body1.p:mad(lambda_dt, -body1.m_inv)
    body1.a = body1.a - Vec2.cross(r1, lambda_dt) * body1.i_inv

    body2.p:mad(lambda_dt, body2.m_inv)
    body2.a = body2.a + Vec2.cross(r2, lambda_dt) * body2.i_inv

    sub1:Delete()
    sub2:Delete()
    r1:Delete()
    r2:Delete()

    lhs:Delete()
    rhs:Delete()
    c:Delete()
    correction:Delete()
    em_inv:Delete()
    lambda_dt:Delete()

    return positionError < Joint.LINEAR_SLOP and angularError < Joint.ANGULAR_SLOP
end

function RevoluteJoint:getReactionForce(invTimeStep)
    return Vec2.scale(self.lambda_acc, invTimeStep)
end

function RevoluteJoint:getReactionTorque(_)
    return 0
end

function RevoluteJoint:draw(dl, color)
    local c = color or Color.red
    local anchor1 = self.body1:getWorldPoint(self.anchor1)
    local center = Vector3(anchor1.x, anchor1.y)
    local up = Vec2.New(0, 1)
    for i = 1, 10 do
        local rad = math.pi * 2 * i / 10
        local val = Vec2.rotate(up, rad)
        dl(center, center + 0.1 * Vector3(val.x, val.y), c)
        val:Delete()
    end
    up:Delete()
    anchor1:Delete()
end

return RevoluteJoint