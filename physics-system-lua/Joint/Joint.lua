---@class Joint
local Joint = CreateClass("Joint")

Joint.TYPE_ANGLE = 0
Joint.TYPE_REVOLUTE = 1
Joint.TYPE_WELD = 2
Joint.TYPE_WHEEL = 3
Joint.TYPE_PRISMATIC = 4
Joint.TYPE_DISTANCE = 5
Joint.TYPE_ROPE = 6
Joint.TYPE_MOUSE = 7

Joint.LINEAR_SLOP = 0.0008
Joint.ANGULAR_SLOP = math.rad(2)
Joint.MAX_LINEAR_CORRECTION = 0.5
Joint.MAX_ANGULAR_CORRECTION = math.rad(8)

Joint.LIMIT_STATE_INACTIVE = 0
Joint.LIMIT_STATE_AT_LOWER = 1
Joint.LIMIT_STATE_AT_UPPER = 2
Joint.LIMIT_STATE_EQUAL_LIMITS = 3
local id_alloc

function Joint:__ctor(body1, body2)
    id_alloc = id_alloc or 1
    self.id = id_alloc
    id_alloc = id_alloc + 1

    self.type = false
    self.collideConnected = false

    ---@type Body
    self.body1 = body1
    ---@type Body
    self.body2 = body2

    self.maxForce = math.huge

    self.breakable = false

    self.awaked = true
end

function Joint:getWorldAnchor1()
    return self.body1:getWorldPoint(self.anchor1)
end

function Joint:getWorldAnchor2()
    return self.body2:getWorldPoint(self.anchor2)
end

function Joint:setWorldAnchor1(anchor1)
    self.anchor1 = self.body1:getLocalPoint(anchor1)
end

function Joint:setWorldAnchor2(anchor2)
    self.anchor2 = self.body2:getLocalPoint(anchor2)
end

function Joint:isAwake()
    return self.awaked
end

function Joint:awake(flag)
    self.awaked = flag
end

return Joint