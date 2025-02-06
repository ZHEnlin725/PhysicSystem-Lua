local base = Reusable
---@class Vec2
local Vector2 = CreateClass("Vector2", base)

local GetOrCreate = function(x, y)
    local inst = base.Get(Vector2.__cname)
    if not inst then
        inst = Vector2.New(x, y)
    else
        inst:set(x, y)
    end
    return inst
end

local function add(v1, v2)
    return GetOrCreate(v1.x + v2.x, v1.y + v2.y)
end

local function sub(v1, v2)
    return GetOrCreate(v1.x - v2.x, v1.y - v2.y)
end

local function dot(v1, v2)
    return v1.x * v2.x + v1.y * v2.y
end

local function cross(v1, v2)
    return v1.x * v2.y - v1.y * v2.x
end

local function scale(v, s)
    return GetOrCreate(v.x * s, v.y * s)
end

local function mad(v1, v2, s)
    return GetOrCreate(v1.x + v2.x * s, v1.y + v2.y * s)
end

local function neg(v)
    return GetOrCreate(-v.x, -v.y)
end

local function perp(v)
    return GetOrCreate(-v.y, v.x)
end

local function rperp(v)
    return GetOrCreate(v.y, -v.x)
end

local function distSq(v1, v2)
    return (v1.x - v2.x) ^ 2 + (v1.y - v2.y) ^ 2
end

local function dist(v1, v2)
    return math.sqrt(distSq(v1, v2))
end

local function lerp(v1, v2, t)
    local x = v1.x * (1 - t) + v2.x * t
    local y = v1.y * (1 - t) + v2.y * t
    return GetOrCreate(x, y)
end

local function rotate(v, rad)
    local c = math.cos(rad)
    local s = math.sin(rad)
    return GetOrCreate(v.x * c - v.y * s, v.x * s + v.y * c)
end

local function rotation(rad)
    return GetOrCreate(math.cos(rad), math.sin(rad))
end

local function normalize(v)
    local inv = (v.x ~= 0 or v.y ~= 0) and
            1 / math.sqrt(v.x * v.x + v.y * v.y) or
            0
    v.x = v.x * inv
    v.y = v.y * inv
    return v
end

local function angle(v)
    return math.atan(v.y, v.x)
end

local function truncate(v, length)
    local ret = v:duplicate()
    local length_sq = v:lengthSq()
    if length_sq > length * length then
        ret:scale(length / math.sqrt(length_sq))
    end
    return ret
end

function Vector2:__ctor(x, y)
    self.x = x or 0
    self.y = y or 0
end

function Vector2:set(x, y)
    self.x = x
    self.y = y
end

function Vector2:dot(v)
    return self.x * v.x + self.y * v.y
end

function Vector2:cross(v)
    return cross(self, v)
end

function Vector2:rotation(rad)
    self.x = math.cos(rad)
    self.y = math.sin(rad)
    return self
end

function Vector2:rotate(rad)
    local c = math.cos(rad)
    local s = math.sin(rad)
    return self:set(self.x * c - self.y * s, self.x * s + self.y * c)
end

function Vector2:add(v)
    self.x = self.x + v.x
    self.y = self.y + v.y
    return self
end

function Vector2:sub(v)
    self.x = self.x - v.x
    self.y = self.y - v.y
    return self
end

function Vector2:scale(s)
    self.x = self.x * s
    self.y = self.y * s
    return self
end

function Vector2:neg()
    self.x = -self.x
    self.y = -self.y

    return self
end

function Vector2:mad(v, s)
    self.x = self.x + v.x * s
    self.y = self.y + v.y * s
    return self
end

function Vector2:equal(v)
    return self.x == v.x and self.y == v.y
end

function Vector2:lengthSq()
    return self.x * self.x + self.y * self.y
end

function Vector2:length()
    return math.sqrt(self:lengthSq())
end

function Vector2:normalize()
    local inv = (self.x ~= 0 or self.y ~= 0) and
            1 / math.sqrt(self.x * self.x + self.y * self.y) or
            0
    self.x = self.x * inv
    self.y = self.y * inv

    return self
end

function Vector2:copy(v)
    self.x = v.x
    self.y = v.y
end

function Vector2:duplicate()
    return GetOrCreate(self.x, self.y)
end

function Vector2:tostring()
    return string.format("(%s,%s)", tostring(self.x), tostring(self.y))
end

return {
    New = GetOrCreate,

    zero = Vector2.New(0, 0),

    add = add,
    sub = sub,
    dot = dot,
    cross = cross,
    scale = scale,
    mad = mad,
    neg = neg,
    perp = perp,
    rperp = rperp,
    dist = dist,
    distSq = distSq,
    lerp = lerp,
    normalize = normalize,
    angle = angle,
    rotate = rotate,
    rotation = rotation,
    truncate = truncate,
}