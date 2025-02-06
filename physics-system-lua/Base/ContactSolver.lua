---@class ContactSolver
local ContactSolver = CreateClass("ContactSolver")

ContactSolver.COLLISION_SLOP = 0.0008
ContactSolver.BAUMGARTE = 0.28
ContactSolver.MAX_LINEAR_CORRECTION = 1--Infinity

function ContactSolver:__ctor(shape1, shape2)
    -- Contact shapes
    self.shape1 = shape1
    self.shape2 = shape2

    -- Contact list
    ---@type Contact[]
    self.contactArr = {}

    -- Coefficient of restitution (elasticity)
    self.e = 1

    -- Frictional coefficient
    self.u = 1
end

function ContactSolver:update(newContactArr)
    for _, newContact in ipairs(newContactArr) do
        local k = -1
        for j, contact in ipairs(self.contactArr) do
            if newContact.hash == contact.hash then
                k = j
                break
            end
        end
        if k > 0 then
            newContact.lambda_n_acc = self.contactArr[k].lambda_n_acc
            newContact.lambda_t_acc = self.contactArr[k].lambda_t_acc
        end
    end
    self.contactArr = newContactArr
end

function ContactSolver:initSolver(invTimeStep)
    ---@type Body
    local body1 = self.shape1.body
    ---@type Body
    local body2 = self.shape2.body

    local sum_m_inv = body1.m_inv + body2.m_inv
    for _, con in ipairs(self.contactArr) do
        -- Transformed r1, r2
        local old_r1 = con.r1
        local old_r2 = con.r2
        con.r1 = Vec2.sub(con.p, body1.p)
        con.r2 = Vec2.sub(con.p, body2.p)

        local old_r1_l = con.r1_local
        local old_r2_l = con.r2_local
        -- Local r1, r2
        con.r1_local = body1.xf:invRotate(con.r1)
        con.r2_local = body2.xf:invRotate(con.r2)

        local n = con.n
        local t = Vec2.perp(con.n)

        -- invEMn = J * invM * JT
        -- J = [ -n, -cross(r1, n), n, cross(r2, n) ]
        local sn1 = Vec2.cross(con.r1, n)
        local sn2 = Vec2.cross(con.r2, n)
        local emn_inv = sum_m_inv + body1.i_inv * sn1 * sn1 + body2.i_inv * sn2 * sn2
        con.emn = emn_inv == 0 and 0 or 1 / emn_inv

        -- invEMt = J * invM * JT
        -- J = [ -t, -cross(r1, t), t, cross(r2, t) ]
        local st1 = Vec2.cross(con.r1, t)
        local st2 = Vec2.cross(con.r2, t)
        local emt_inv = sum_m_inv + body1.i_inv * st1 * st1 + body2.i_inv * st2 * st2
        con.emt = emt_inv == 0 and 0 or 1 / emt_inv

        -- Linear velocities at contact point
        -- in 2D: cross(w, r) = perp(r) * w
        local perp1 = Vec2.perp(con.r1)
        local perp2 = Vec2.perp(con.r2)
        local v1 = Vec2.mad(body1.v, perp1, body1.w)
        local v2 = Vec2.mad(body2.v, perp2, body2.w)

        -- relative velocity at contact point
        local rv = Vec2.sub(v2, v1)

        -- bounce velocity dot n
        con.bounce = Vec2.dot(rv, con.n) * self.e

        if old_r1 then
            old_r1:Delete()
        end

        if old_r2 then
            old_r2:Delete()
        end

        if old_r1_l then
            old_r1_l:Delete()
        end

        if old_r2_l then
            old_r2_l:Delete()
        end

        t:Delete()
        perp1:Delete()
        perp2:Delete()
        v1:Delete()
        v2:Delete()
        rv:Delete()
    end
end

function ContactSolver:warmStart()
    ---@type Body
    local body1 = self.shape1.body
    ---@type Body
    local body2 = self.shape2.body

    for i, con in ipairs(self.contactArr) do
        local n = con.n
        local lambda_n = con.lambda_n_acc
        local lambda_t = con.lambda_t_acc
        -- Apply accumulated impulses
        --local impulse = Vec2.rotate_vec(new Vec2(lambda_n, lambda_t), n)
        local impulse = Vec2.New(lambda_n * n.x - lambda_t * n.y, lambda_t * n.x + lambda_n * n.y)
        body1.v:mad(impulse, -body1.m_inv)
        body1.w = body1.w - Vec2.cross(con.r1, impulse) * body1.i_inv
        body2.v:mad(impulse, body2.m_inv)
        body2.w = body2.w + Vec2.cross(con.r2, impulse) * body2.i_inv

        impulse:Delete()
    end
end

function ContactSolver:solveVelocityConstraints()
    ---@type Body
    local body1 = self.shape1.body
    ---@type Body
    local body2 = self.shape2.body

    local m1_inv = body1.m_inv
    local i1_inv = body1.i_inv
    local m2_inv = body2.m_inv
    local i2_inv = body2.i_inv

    for i, con in ipairs(self.contactArr) do

        local n = con.n
        local t = Vec2.perp(n)
        local r1 = con.r1
        local r2 = con.r2

        -- Linear velocities at contact point
        -- in 2D: cross(w, r) = perp(r) * w
        local perp1 = Vec2.perp(r1)
        local perp2 = Vec2.perp(r2)
        local v1 = Vec2.mad(body1.v, perp1, body1.w)
        local v2 = Vec2.mad(body2.v, perp2, body2.w)

        -- Relative velocity at contact point
        local rv = Vec2.sub(v2, v1)

        -- Compute normal constraint impulse + adding bounce as a velocity bias
        -- lambda_n = -EMn * J * V
        local lambda_n = -con.emn * (Vec2.dot(n, rv) + con.bounce)

        -- Accumulate and clamp
        local lambda_n_old = con.lambda_n_acc
        con.lambda_n_acc = math.max(lambda_n_old + lambda_n, 0)
        lambda_n = con.lambda_n_acc - lambda_n_old

        -- Compute frictional constraint impulse
        -- lambda_t = -EMt * J * V
        local lambda_t = -con.emt * Vec2.dot(t, rv)

        -- Max friction constraint impulse (Coulomb's Law)
        local lambda_t_max = con.lambda_n_acc * self.u

        -- Accumulate and clamp
        local lambda_t_old = con.lambda_t_acc
        con.lambda_t_acc = math.clamp(lambda_t_old + lambda_t, -lambda_t_max, lambda_t_max)
        lambda_t = con.lambda_t_acc - lambda_t_old

        -- Apply the final impulses
        --local impulse = Vec2.rotate_vec(new Vec2(lambda_n, lambda_t), n)
        local impulse = Vec2.New(lambda_n * n.x - lambda_t * n.y, lambda_t * n.x + lambda_n * n.y)

        body1.v:mad(impulse, -m1_inv)
        body1.w = body1.w - Vec2.cross(r1, impulse) * i1_inv

        body2.v:mad(impulse, m2_inv)
        body2.w = body2.w + Vec2.cross(r2, impulse) * i2_inv

        t:Delete()
        perp1:Delete()
        perp2:Delete()
        v1:Delete()
        v2:Delete()
        rv:Delete()
        impulse:Delete()
    end
end

function ContactSolver:solvePositionConstraints()
    ---@type Body
    local body1 = self.shape1.body
    ---@type Body
    local body2 = self.shape2.body

    local m1_inv = body1.m_inv
    local i1_inv = body1.i_inv
    local m2_inv = body2.m_inv
    local i2_inv = body2.i_inv
    local sum_m_inv = m1_inv + m2_inv

    local max_penetration = 0

    for i, con in ipairs(self.contactArr) do
        local n = con.n

        -- Transformed r1, r2
        local r1 = Vec2.rotate(con.r1_local, body1.a)
        local r2 = Vec2.rotate(con.r2_local, body2.a)

        -- Contact points (corrected)
        local p1 = Vec2.add(body1.p, r1)
        local p2 = Vec2.add(body2.p, r2)

        -- Corrected delta vector
        local dp = Vec2.sub(p2, p1)

        -- Position constraint
        local c = Vec2.dot(dp, n) + con.d
        local correction = math.clamp(ContactSolver.BAUMGARTE * (c + ContactSolver.COLLISION_SLOP), -ContactSolver.MAX_LINEAR_CORRECTION, 0)
        if (correction ~= 0) then
            -- We don't need max_penetration less than or equal slop
            max_penetration = math.max(max_penetration, -c)

            -- Compute lambda for position constraint
            -- Solve (J * invM * JT) * lambda = -C / dt
            local sn1 = Vec2.cross(r1, n)
            local sn2 = Vec2.cross(r2, n)
            local em_inv = sum_m_inv + body1.i_inv * sn1 * sn1 + body2.i_inv * sn2 * sn2
            local lambda_dt = em_inv == 0 and 0 or -correction / em_inv

            -- Apply correction impulses
            local impulse_dt = Vec2.scale(n, lambda_dt)

            body1.p:mad(impulse_dt, -m1_inv)
            body1.a = body1.a - sn1 * lambda_dt * i1_inv

            body2.p:mad(impulse_dt, m2_inv)
            body2.a = body2.a + sn2 * lambda_dt * i2_inv

            impulse_dt:Delete()
        end

        r1:Delete()
        r2:Delete()
        p1:Delete()
        p2:Delete()
        dp:Delete()
    end

    return max_penetration <= ContactSolver.COLLISION_SLOP * 3
end

return ContactSolver