local base = Reusable
local Matrix3x3 = CreateClass("Matrix3x3", base)

local GetOrCreate = function(_11, _12, _13, _21, _22, _23, _31, _32, _33)
    local inst = base.Get(Matrix3x3.__cname)
    if not inst then
        inst = Matrix3x3.New(_11, _12, _13, _21, _22, _23, _31, _32, _33)
    else
        inst:set(_11, _12, _13, _21, _22, _23, _31, _32, _33)
    end
    return inst
end

local function mul(m1, m2)
    return GetOrCreate(
            m1._11 * m2._11 + m1._12 * m2._21 + m1._13 * m2._31,
            m1._11 * m2._12 + m1._12 * m2._22 + m1._13 * m2._32,
            m1._11 * m2._13 + m1._12 * m2._23 + m1._13 * m2._33,
            m1._21 * m2._11 + m1._22 * m2._21 + m1._23 * m2._31,
            m1._21 * m2._12 + m1._22 * m2._22 + m1._23 * m2._32,
            m1._21 * m2._13 + m1._22 * m2._23 + m1._23 * m2._33,
            m1._31 * m2._11 + m1._32 * m2._21 + m1._33 * m2._31,
            m1._31 * m2._12 + m1._32 * m2._22 + m1._33 * m2._32,
            m1._31 * m2._13 + m1._32 * m2._23 + m1._33 * m2._33)
end

function Matrix3x3:__ctor(_11, _12, _13, _21, _22, _23, _31, _32, _33)
    self._11 = _11 or 0
    self._12 = _12 or 0
    self._13 = _13 or 0
    self._21 = _21 or 0
    self._22 = _22 or 0
    self._23 = _23 or 0
    self._31 = _31 or 0
    self._32 = _32 or 0
    self._33 = _33 or 0
end

function Matrix3x3:set(_11, _12, _13, _21, _22, _23, _31, _32, _33)
    self._11 = _11 or 0
    self._12 = _12 or 0
    self._13 = _13 or 0
    self._21 = _21 or 0
    self._22 = _22 or 0
    self._23 = _23 or 0
    self._31 = _31 or 0
    self._32 = _32 or 0
    self._33 = _33 or 0
    return self
end

function Matrix3x3:copy(m)
    return self:set(m._11, m._12, m._13,
            m._21, m._22, m._23,
            m._31, m._32, m._33)
end

function Matrix3x3:mul(m)
    return self:set(self._11 * m._11 + self._12 * m._21 + self._13 * m._31,
            self._11 * m._12 + self._12 * m._22 + self._13 * m._32,
            self._11 * m._13 + self._12 * m._23 + self._13 * m._33,
            self._21 * m._11 + self._22 * m._21 + self._23 * m._31,
            self._21 * m._12 + self._22 * m._22 + self._23 * m._32,
            self._21 * m._13 + self._22 * m._23 + self._23 * m._33,
            self._31 * m._11 + self._32 * m._21 + self._33 * m._31,
            self._31 * m._12 + self._32 * m._22 + self._33 * m._32,
            self._31 * m._13 + self._32 * m._23 + self._33 * m._33)
end

function Matrix3x3:mulVec(v)
    return Vec2.New(
            self._11 * v.x + self._12 * v.y + self._13 * v.z,
            self._21 * v.x + self._22 * v.y + self._23 * v.z,
            self._31 * v.x + self._32 * v.y + self._33 * v.z)
end

function Matrix3x3:solve2x2(b)
    local det = self._11 * self._22 - self._12 * self._21
    if (det ~= 0) then
        det = 1 / det
    end
    return Vec2.New(
            det * (self._22 * b.x - self._12 * b.y),
            det * (self._11 * b.y - self._21 * b.x))
end

function Matrix3x3:solve(b)
    local det2_11 = self._22 * self._33 - self._23 * self._32
    local det2_12 = self._23 * self._31 - self._21 * self._33
    local det2_13 = self._21 * self._32 - self._22 * self._31

    local det = self._11 * det2_11 + self._12 * det2_12 + self._13 * det2_13
    if (det ~= 0) then
        det = 1 / det
    end

    local det2_21 = self._13 * self._32 - self._12 * self._33
    local det2_22 = self._11 * self._33 - self._13 * self._31
    local det2_23 = self._12 * self._31 - self._11 * self._32
    local det2_31 = self._12 * self._23 - self._13 * self._22
    local det2_32 = self._13 * self._21 - self._11 * self._23
    local det2_33 = self._11 * self._22 - self._12 * self._21

    return Vec3.New(
            det * (det2_11 * b.x + det2_12 * b.y + det2_13 * b.z),
            det * (det2_21 * b.x + det2_22 * b.y + det2_23 * b.z),
            det * (det2_31 * b.x + det2_32 * b.y + det2_33 * b.z))
end

return {
    New = GetOrCreate,

    mul = mul,
}