---@class World
local World = CreateClass("World")
local Collision = require 'Collision'

World.TIME_TO_SLEEP = 0.5
World.SLEEP_LINEAR_TOLERANCE = 0.5
World.SLEEP_ANGULAR_TOLERANCE = math.rad(2)

local function refreshHashArr(startIndex, arr, hash)
    for i = startIndex, #arr do
        hash[arr[i].id] = i
    end
end

function World:__ctor()
    ---@type Body[]
    self.bodyArr = {}
    self.bodyHash = {}

    ---@type Joint[]
    self.jointArr = {}
    self.jointHash = {}

    self.numContacts = 0
    ---@type ContactSolver[]
    self.contactSolverArr = {}

    self.postSolve = nil
    self.gravity = Vec2.New(0, -10)
    self.damping = 0
    self.stepCount = 0
end

function World:clear()
    --Shape.id_counter = 0
    --Body.id_counter = 0
    --Joint.id_counter = 0

    for i = #self.bodyArr, 1, -1 do
        self:removeBody(self.bodyArr[i])
    end

    self.bodyArr = {}
    self.bodyHash = {}

    self.jointArr = {}
    self.jointHash = {}

    self.contactSolverArr = {}
    self.stepCount = 0
end

---@param body Body
function World:addBody(body)
    if self.bodyHash[body.id] then
        return
    end

    table.insert(self.bodyArr, body)
    local index = #self.bodyArr
    self.bodyHash[body.id] = index

    body:awake(true)
    body.world = self
    body:cacheData()
end

---@param body Body
function World:removeBody(body)
    if not self.bodyHash[body.id] then
        return
    end

    local len = #body.jointArr
    for i = len, 1, -1 do
        self:removeJoint(body.jointArr[i])
    end

    body.world = nil
    local index = self.bodyHash[body.id]
    self.bodyHash[body.id] = nil
    table.remove(self.bodyArr, index)
    refreshHashArr(index, self.bodyArr, self.bodyHash)
end

---@param joint Joint
function World:addJoint(joint)
    if self.jointHash[joint.id] then
        return
    end

    joint.body1:awake(true)
    joint.body2:awake(true)

    table.insert(self.jointArr, joint)
    self.jointHash[joint.id] = #self.jointArr

    table.insert(joint.body1.jointArr, joint)
    joint.body1.jointHash[joint.id] = #joint.body1.jointArr

    table.insert(joint.body2.jointArr, joint)
    joint.body2.jointHash[joint.id] = #joint.body2.jointArr
end

---@param joint Joint
function World:removeJoint(joint)
    if not self.jointHash[joint.id] then
        return
    end

    joint.body1:awake(true)
    joint.body2:awake(true)

    local index = joint.body1.jointHash[joint.id]
    table.remove(joint.body1.jointArr, index)
    joint.body1.jointHash[joint.id] = nil
    refreshHashArr(index, joint.body1.jointArr, joint.body1.jointHash)

    index = joint.body2.jointHash[joint.id]
    table.remove(joint.body2.jointArr, index)
    joint.body2.jointHash[joint.id] = nil
    refreshHashArr(index, joint.body2.jointArr, joint.body2.jointHash)

    index = self.jointHash[joint.id]
    table.remove(self.jointArr, index)
    self.jointHash[joint.id] = nil
    refreshHashArr(index, self.jointArr, self.jointHash)
end

function World:findContactSolver(shape1, shape2)
    for _, con in ipairs(self.contactSolverArr) do
        if con.shape1 == shape1 and con.shape2 == shape2 then
            return con
        end
    end
end

function World:genTemporalContactSolvers()
    local newContactArr = {}
    self.numContacts = 0

    local len = #self.bodyArr
    for body1_index = 1, len do
        local body1 = self.bodyArr[body1_index]
        body1.stepCount = self.stepCount

        for body2_index = body1_index + 1, len do
            local body2 = self.bodyArr[body2_index]
            if body1.stepCount ~= body2.stepCount then
                local active1 = body1:isAwake() and not body1:isStatic()
                local active2 = body2:isAwake() and not body2:isStatic()
                if active1 or active2 then
                    if body1:isCollidable(body2) and body1.bounds:intersects(body2.bounds) then
                        for _, body1_shape in ipairs(body1.shapeArr) do
                            for _, body2_shape in ipairs(body2.shapeArr) do
                                local shape1 = body1_shape
                                local shape2 = body2_shape
                                if body1_shape.type > body2_shape.type then
                                    shape1 = body2_shape
                                    shape2 = body1_shape
                                end
                                local result, contactArr = Collision.collide(shape1, shape2)
                                if result then
                                    self.numContacts = self.numContacts + #contactArr
                                    local solver = self:findContactSolver(shape1, shape2)
                                    if solver then
                                        solver:update(contactArr)
                                        table.insert(newContactArr, solver)
                                    else
                                        body1:awake(true)
                                        body2:awake(true)
                                        local newContactSolver = ContactSolver.New(shape1, shape2)
                                        newContactSolver.contactArr = contactArr
                                        newContactSolver.e = math.max(shape1.e, shape2.e)
                                        newContactSolver.u = math.sqrt(shape1.u * shape2.u)
                                        table.insert(newContactArr, newContactSolver)
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end

    return newContactArr
end

function World:initSolver(timeStep, invTimeStep, warmStarting)
    for _, contactSolver in ipairs(self.contactSolverArr) do
        contactSolver:initSolver(invTimeStep)
    end

    for _, joint in ipairs(self.jointArr) do
        if joint:isAwake() then
            joint:initSolver(timeStep, warmStarting)
        end
    end

    if warmStarting then
        for _, contactSolver in ipairs(self.contactSolverArr) do
            contactSolver:warmStart()
        end
    end
end

function World:velocitySolver(iteration)
    for _ = 1, iteration do
        for _, joint in ipairs(self.jointArr) do
            if joint:isAwake() then
                joint:solveVelocityConstraints()
            end
        end

        for _, contactSolver in ipairs(self.contactSolverArr) do
            contactSolver:solveVelocityConstraints()
        end
    end
end

function World:positionSolver(iteration)
    local positionSolved = false
    for _ = 1, iteration do
        local contactsOk = true
        local jointsOk = true
        for _, contactSolver in ipairs(self.contactSolverArr) do
            contactsOk = contactsOk and contactSolver:solvePositionConstraints()
        end
        for _, joint in ipairs(self.jointArr) do
            if joint:isAwake() then
                jointsOk = jointsOk and joint:solvePositionConstraints()
            end
        end
        if contactsOk and jointsOk then
            positionSolved = true
            break
        end
    end
    return positionSolved
end

function World:step(timeStep, vel_iteration, pos_iteration, warmStarting, allowSleep)
    local invTimeStep = timeStep > 0 and 1 / timeStep or 0
    self.stepCount = self.stepCount + 1
    self.contactSolverArr = self:genTemporalContactSolvers()

    self:initSolver(timeStep, invTimeStep, warmStarting)

    for _, body in ipairs(self.bodyArr) do
        if body:isDynamic() and body:isAwake() then
            body:updateVelocity(self.gravity, timeStep, self.damping)
        end
    end

    for _, joint in ipairs(self.jointArr) do
        if joint:isAwake() then
            local body1 = joint.body1
            local body2 = joint.body2

            local awake1 = body1:isAwake() and not body1:isStatic()
            local awake2 = body2:isAwake() and not body2:isStatic()
            if awake1 ~= awake2 then
                if not awake1 then
                    body1:awake(true)
                end
                if not awake2 then
                    body2:awake(true)
                end
            end
        end
    end
    self:velocitySolver(vel_iteration)

    for _, body in ipairs(self.bodyArr) do
        if body:isDynamic() and body:isAwake() then
            body:updatePosition(timeStep)
        end
    end
    --[[    --// Process breakable joint
        --for (var i = 0; i < this.jointArr.length; i++) {
        --var joint = this.jointArr[i];
        --if (!joint) {
        --continue;
        --}
        --
        --if (joint.breakable) {
        --if (joint.getReactionForce(dt_inv).lengthsq() >= joint.maxForce * joint.maxForce)
        --this.removeJoint(joint);
        --}
        --}]]
    local positionSolved = self:positionSolver(pos_iteration)

    for _, body in ipairs(self.bodyArr) do
        body:syncTransform()
    end

    if type(self.postSolve) == "function" then
        for _, contactSolver in ipairs(self.contactSolverArr) do
            self.postSolve(contactSolver)
        end
    end

    for _, body in ipairs(self.bodyArr) do
        if body:isDynamic() and body:isAwake() then
            body:cacheData()
        end
    end
    -- Process sleeping
    if (allowSleep) then
        local minSleepTime = 999999
        local linTolSqr = World.SLEEP_LINEAR_TOLERANCE * World.SLEEP_LINEAR_TOLERANCE
        local angTolSqr = World.SLEEP_ANGULAR_TOLERANCE * World.SLEEP_ANGULAR_TOLERANCE
        for _, body in ipairs(self.bodyArr) do
            if body:isDynamic() then
                if body.w * body.w > angTolSqr or body.v:dot(body.v) > linTolSqr then
                    body.sleepTime = 0
                    minSleepTime = 0
                else
                    body.sleepTime = body.sleepTime + timeStep
                    minSleepTime = math.min(minSleepTime, body.sleepTime)
                end
            end
        end
        if positionSolved and minSleepTime >= World.TIME_TO_SLEEP then
            for _, body in ipairs(self.bodyArr) do
                body:awake(false)
            end
        end
    end
end

local dl = CS.UnityEngine.Debug.DrawLine
function World:drawBodies()
    for _, body in ipairs(self.bodyArr) do
        for _, shape in ipairs(body.shapeArr) do
            if shape.draw then
                shape:draw(dl)
            end
        end
    end
end

function World:drawJoints()
    for _, joint in ipairs(self.jointArr) do
        if joint.draw then
            joint:draw(dl)
        end
    end
end

return World