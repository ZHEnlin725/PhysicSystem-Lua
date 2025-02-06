local base = Reusable
---@class Vec3
local Vector3 = CreateClass("Vector3", base)

local GetOrCreate = function(x, y, z)
    local inst = base.Get(Vector3.__cname)
    if not inst then
        inst = Vector3.New(x, y, z)
    else
        inst:set(x, y, z)
    end
    return inst
end

local function add(v1, v2)
    return GetOrCreate(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)
end

local function sub(v1, v2)
    return GetOrCreate(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)
end

local function dot(v1, v2)
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z
end

local function scale(v, s)
    return GetOrCreate(v.x * s, v.y * s, v.z * s)
end

local function mad(v1, v2, s)
    return GetOrCreate(v1.x + v2.x * s, v1.y + v2.y * s, v1.z + v2.z * s)
end

local function neg(v)
    return GetOrCreate(-v.x, -v.y, -v.z)
end

local function dist(v1, v2)
    return math.sqrt((v1.x - v2.x) ^ 2 + (v1.y - v2.y) ^ 2 + (v1.z - v2.z) ^ 2)
end

local function lerp(v1, v2, t)
    local x = v1.x * (1 - t) + v2.x * t
    local y = v1.y * (1 - t) + v2.y * t
    local z = v1.z * (1 - t) + v2.z * t
    return GetOrCreate(x, y, z)
end

local function truncate(v, length)
    local ret = v:duplicate()
    local length_sq = v:lengthSq()
    if length_sq > length * length then
        ret:scale(length / math.sqrt(length_sq))
    end
    return ret
end

---@param vec2 Vec2
local function fromVec2(vec2, z)
    return GetOrCreate(vec2.x, vec2.y, z)
end

function Vector3:__ctor(x, y, z)
    self.x = x or 0
    self.y = y or 0
    self.z = z or 0
end

function Vector3:set(x, y, z)
    self.x = x
    self.y = y
    self.z = z
end

function Vector3:dot(v)
    return self.x * v.x + self.y * v.y + self.z * v.z
end

function Vector3:add(v)
    self.x = self.x + v.x
    self.y = self.y + v.y
    self.z = self.z + v.z
    return self
end

function Vector3:sub(v)
    self.x = self.x - v.x
    self.y = self.y - v.y
    self.z = self.z - v.z
    return self
end

function Vector3:scale(s)
    self.x = self.x * s
    self.y = self.y * s
    self.z = self.z * s
    return self
end

function Vector3:neg()
    self.x = -self.x
    self.y = -self.y
    self.z = -self.z
    return self
end

function Vector3:mad(v, s)
    self.x = self.x + v.x * s
    self.y = self.y + v.y * s
    self.z = self.z + v.z * s
    return self
end

function Vector3:equal(v)
    return self.x == v.x and self.y == v.y and self.z == v.z
end

function Vector3:lengthSq()
    return self.x * self.x + self.y * self.y + self.z * self.z
end

function Vector3:length()
    return math.sqrt(self:lengthSq())
end

function Vector3:normalize()
    local inv = (self.x ~= 0 or self.y ~= 0 or self.z ~= 0) and
            (1 / self:length()) or
            0
    self.x = self.x * inv
    self.y = self.y * inv
    return self
end

function Vector3:copy(v)
    self.x = v.x
    self.y = v.y
    self.z = v.z
end

function Vector3:duplicate()
    return GetOrCreate(self.x, self.y, self.z)
end

---@return Vec2
function Vector3:toVec2()
    return Vec2.New(self.x, self.y)
end

function Vector3:tostring()
    return string.format("(%s,%s,%s)", tostring(self.x), tostring(self.y), tostring(self.z))
end

return {
    New = GetOrCreate,

    add = add,
    sub = sub,
    dot = dot,
    scale = scale,
    mad = mad,
    neg = neg,
    dist = dist,
    lerp = lerp,
    truncate = truncate,
    fromVec2 = fromVec2,
    zero = Vector3.New(0, 0, 0),
}