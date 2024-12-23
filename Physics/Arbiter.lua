---@class Arbiter
local Arbiter = Create("Arbiter")

local BodyType = BodyType

function Arbiter:__ctor(bodyA, bodyB)
    self.bodyA = nil
    self.bodyB = nil
    self.normal = nil
    self.restitution = 0
    self.friction = 0
    self.contacts = {}
    self.sensor = false
    self.restitutionVelocity = 1
    self.splittingFrame = 3
    self.collisionSlop = 0.01
end


---@param bodyA Body
---@param bodyB Body
function Arbiter:set(bodyA, bodyB, normalA)
    self.bodyA = bodyA
    self.bodyB = bodyB
    self.key = bodyA.id .. "_&_" .. bodyB.id
    self.normal = normalA
    self.tangent = { -normalA[2], normalA[1] }

    self.restitution = math.max(bodyA.restitution, bodyB.restitution)
    self.friction = math.sqrt(bodyA.friction * bodyB.friction)
    self.sensor = bodyA.sensor or bodyB.sensor

    self.contactCount = 0
    table.clear(self.contacts)

    self.lastDataA = {}
    self.lastDataB = {}
    --self.disabled = ((bodyA.isTrigger or bodyB.isTrigger) or
    --        (bodyA.bodyType == BodyType.Static and bodyB.bodyType == BodyType.Static))
end

function Arbiter:addContact(contactOnA, contactOnB, depth)
    local normal = self.normal
    local tangent = self.tangent
    local bodyA = self.bodyA
    local bodyB = self.bodyB

    if depth == nil then
        depth = (contactOnA[1] - contactOnB[1]) * normal[1] + (contactOnA[2] - contactOnB[2]) * normal[2]
    end

    depth = depth - self.collisionSlop

    local armA = { contactOnA[1] - bodyA.x, contactOnA[2] - bodyA.y }
    local armB = { contactOnB[1] - bodyB.x, contactOnB[2] - bodyB.y }

    local armACrossN = armA[1] * normal[2] - armA[2] * normal[1]
    armACrossN = armACrossN * armACrossN
    local armBCrossN = armB[1] * normal[2] - armB[2] * normal[1]
    armBCrossN = armBCrossN * armBCrossN

    local armACrossT = armA[1] * tangent[2] - armA[2] * tangent[1]
    armACrossT = armACrossT * armACrossT
    local armBCrossT = armB[1] * tangent[2] - armB[2] * tangent[1]
    armBCrossT = armBCrossT * armBCrossT

    local denom = bodyA.invMass + bodyB.invMass + (armACrossN * bodyA.invInertia) + (armBCrossN * bodyB.invInertia)
    local normalMass = denom == 0 and 0 or 1 / denom

    denom = bodyA.invMass + bodyB.invMass + (armACrossT * bodyA.invInertia) + (armBCrossT * bodyB.invInertia)
    local tangentMass = denom == 0 and 0 or 1 / denom

    local velocityBias = nil

    local contact = {
        contactOnA = contactOnA,
        contactOnB = contactOnB,
        armA = armA,
        armB = armB,
        depth = depth,
        normalImpulse = 0,
        tangentImpulse = 0,
        normalMass = normalMass,
        tangentMass = tangentMass,
        velocityBias = velocityBias,
    }
    table.insert(self.contacts, contact)
    self.contactCount = self.contactCount + 1
end

function Arbiter:isFirstContact()
    -- Implementation needed
end

function Arbiter:isDisabled()
    return ((self.bodyA.isTrigger or self.bodyB.isTrigger) or
            (self.bodyA.bodyType == BodyType.Static and self.bodyB.bodyType == BodyType.Static))
end

function Arbiter:preSolve(timeStep)
    if self:isFirstContact() then
        return
    end

    local bodyA = self.bodyA
    local bodyB = self.bodyB
    local normal = self.normal

    for i = 1, #self.contacts do
        local con = self.contacts[i]

        local armA = con.armA
        local armB = con.armB
        local nx = normal[1]
        local ny = normal[2]
        local impX = nx * con.normalImpulse - ny * con.tangentImpulse
        local impY = nx * con.tangentImpulse + ny * con.normalImpulse

        bodyA.velX = bodyA.velX + (impX * bodyA.invMass)
        bodyA.velY = bodyA.velY + (impY * bodyA.invMass)
        bodyA.velAng = bodyA.velAng + (armA[1] * impY - armA[2] * impX) * bodyA.invInertia
        bodyB.velX = bodyB.velX - (impX * bodyB.invMass)
        bodyB.velY = bodyB.velY - (impY * bodyB.invMass)
        bodyB.velAng = bodyB.velAng - (armB[1] * impY - armB[2] * impX) * bodyB.invInertia
    end
end

function Arbiter:solve(timeStep, iterations, iter)
    local bodyA = self.bodyA
    local bodyB = self.bodyB
    local normal = self.normal
    local tangent = self.tangent
    local restitution = self.restitution
    local friction = self.friction
    local sensor = self.sensor
    local lastDataA, lastDataB
    local solved = false

    local contacts = self.contacts
    local contactCount = #contacts

    local rvN = nil
    for k = 1, contactCount do
        local contact = contacts[k]

        local contactOnA = contact.contactOnA
        local contactOnB = contact.contactOnB
        local armA = contact.armA
        local armB = contact.armB
        local depth = contact.depth
        local normalMass = contact.normalMass
        local tangentMass = contact.tangentMass

        if not solved then
            solved = true
            self.lastDataA = {
                velX = bodyA.velX,
                velY = bodyA.velY,
                velAng = bodyA.velAng,
            }
            self.lastDataB = {
                velX = bodyB.velX,
                velY = bodyB.velY,
                velAng = bodyB.velAng,
            }
        end

        local relativeVel = {
            (bodyA.velX - bodyA.velAng * armA[2]) - (bodyB.velX - bodyB.velAng * armB[2]),
            (bodyA.velY + bodyA.velAng * armA[1]) - (bodyB.velY + bodyB.velAng * armB[1])
        }
        local normalRelativeVel = relativeVel[1] * normal[1] + relativeVel[2] * normal[2]

        if contact.velocityBias == nil then
            contact.velocityBias = -restitution * normalRelativeVel / iterations / contactCount
        end

        if depth > 0 then
            if bodyA.bodyType ~= BodyType.Static and bodyB.bodyType ~= BodyType.Static then
                -- bodyA:awake()
                -- bodyB:awake()
            end
            local dt = timeStep
            local dd = depth
            normalRelativeVel = normalRelativeVel + dd / dt
            contact.depth = contact.depth - dd / iterations
        end

        local impN = normalMass * -(normalRelativeVel + contact.velocityBias)
        local normalImpulse = contact.normalImpulse
        contact.normalImpulse = math.min(impN + normalImpulse, 0)
        impN = contact.normalImpulse - normalImpulse

        local tangentRelativeVel = relativeVel[1] * tangent[1] + relativeVel[2] * tangent[2]

        local impT = tangentMass * -tangentRelativeVel
        local frictionImp = math.abs(contact.normalImpulse * friction)
        local tangentImpulse = contact.tangentImpulse
        contact.tangentImpulse = math.max(-frictionImp, math.min(impT + tangentImpulse, frictionImp))
        impT = contact.tangentImpulse - tangentImpulse

        local impX = normal[1] * impN - normal[2] * impT
        local impY = normal[1] * impT + normal[2] * impN
        bodyA.velX = bodyA.velX + (impX * bodyA.invMass)
        bodyA.velY = bodyA.velY + (impY * bodyA.invMass)
        bodyA.velAng = bodyA.velAng + (armA[1] * impY - armA[2] * impX) * bodyA.invInertia
        bodyB.velX = bodyB.velX - (impX * bodyB.invMass)
        bodyB.velY = bodyB.velY - (impY * bodyB.invMass)
        bodyB.velAng = bodyB.velAng - (armB[1] * impY - armB[2] * impX) * bodyB.invInertia
    end

    return solved
end

return Arbiter