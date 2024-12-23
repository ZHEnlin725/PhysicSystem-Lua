---@class World
local World = Create("World")

function World:__ctor(onCollidingCallback, onCollideCallback, onSeparatedCallback)
    self.bodies = nil
    self.gravityX = 0
    self.gravityY = 0
    self.forceX = 0
    self.forceY = 0
    self.allowSleep = true
    self.timeToSleep = 0.5
    self.minSleepVelSq = 0.01  --*0.01
    self.minSleepVelAng = 0.2
    self.solveIterations = 10
    self.BODY_SN_SEED = 1
    self.onColliding = onCollidingCallback
    self.onCollided = onCollideCallback
    self.onSeparated = onSeparatedCallback

    self.bodies = {}

    self:initCollideManager()

    self:onInit()
end

function World:onInit()
end

function World:initCollideManager()
    ---@type CollideManager
    local cm = self.collideManager or CollideManager.New()
    cm.onColliding = self.onColliding or cm.onColliding
    cm.onCollided = self.onCollided or cm.onCollided
    cm.onSeparated = self.onSeparated or cm.onSeparated
    cm.onCollideSolve = self.onCollideSolve or cm.onCollideSolve
    cm:init(self)
    self.collideManager = cm
end

---@param body Body
function World:addBody(body)
    body._sn = self.BODY_SN_SEED
    self.BODY_SN_SEED = self.BODY_SN_SEED + 1

    body.id = body.id or body._sn

    if (body.bodyType ~= BodyType.Static and not body.ignoreG) then
        body.gravityX = self.gravityX
        body.gravityY = self.gravityY
    end

    --self.bodies.push(body)
    table.insert(self.bodies, body)
    return body
end

---@param body Body
function World:removeBody(body)
    body._to_remove_ = true
end

---@param body Body
function World:checkSleep(body, timeStep)
    if (math.abs(body.velAng) <= self.minSleepVelAng and body:getVelSq() <= self.minSleepVelSq) then
        body.sleepTime = body.sleepTime + timeStep
        if (body.sleepTime >= self.timeToSleep) then
            body.sleeping = true
            return true
        end
    else
        --// console.log(Math.abs(body.velAng), body.getVelSq())
        body:awake()
    end
    return false
end

function World:preSolve(timeStep)
    --local iterations = self.solveIterations
    local collideManager = self.collideManager
    --for i = 1, iterations do
    collideManager:preSolve(timeStep)
    --end
end

function World:solve(timeStep)
    local iterations = self.solveIterations
    local collideManager = self.collideManager
    for i = 1, iterations do
        collideManager:solve(timeStep, iterations, i)
    end
end

function World:step(timeStep)
    ---@type Body[]
    local bodies = self.bodies
    local i, len = 1, #bodies
    while i <= len do
        local body = bodies[i]
        if not body.body then
            if body._to_remove_ then
                len = len - 1
                table.remove(bodies, i)
            else
                body:saveStatus()
            end
            if not body.sleeping then
                body:integrateVelAngle(timeStep)
                body:integrateVel(timeStep)
            end
        end
        i = i + 1
    end
    self.collideManager:update(timeStep)
    self:solve(timeStep)

    i = 1
    while i <= len do
        local body = bodies[i]
        if not body.body then
            if not body.sleeping then
                body:integrate(timeStep)
                body:update(timeStep)
            end
            if (self.allowSleep and body.autoSleep) then
                self:checkSleep(body, timeStep)
            end
            body:setForce(0, 0)
            body.torque = 0
            i = i + 1
        end
    end
end

function World:drawgizmos()
    local bodies = self.bodies
    local i, len = 1, #bodies
    while i <= len do
        local body = bodies[i]
        body:drawaabb()
        i = i + 1
    end
end

return World