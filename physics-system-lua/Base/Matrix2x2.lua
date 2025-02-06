local base = Reusable
local Matrix2x2 = CreateClass("Matrix2x2", base)

local GetOrCreate = function(_11, _12, _21, _22)
    local inst = base.Get(Matrix2x2.__cname)
    if not inst then
        inst = Matrix2x2.New(_11, _12, _21, _22)
    else
        inst:set(_11, _12, _21, _22)
    end
    return inst
end

local function mul(m1, m2)
    return GetOrCreate(m1._11 * m2._11 + m1._12 * m2._21,
            m1._11 * m2._12 + m1._12 * m2._22,
            m1._21 * m2._11 + m1._22 * m2._21,
            m1._21 * m2._12 + m1._22 * m2._22)
end

function Matrix2x2:__ctor(_11, _12, _21, _22)
    self._11 = _11 or 0
    self._12 = _12 or 0
    self._21 = _21 or 0
    self._22 = _22 or 0
end

function Matrix2x2:set(_11, _12, _21, _22)
    self._11 = _11 or 0
    self._12 = _12 or 0
    self._21 = _21 or 0
    self._22 = _22 or 0
    return self
end

function Matrix2x2:copy(m)
    self:set(m._11, m._12, m._21, m._22)
end

function Matrix2x2:mul(m)
    return self:set(
            self._11 * m._11 + self._12 * m._21,
            self._11 * m._12 + self._12 * m._22,
            self._21 * m._11 + self._22 * m._21,
            self._21 * m._12 + self._22 * m._22)
end

function Matrix2x2:mulVec(v)
    return Vec2.New(self._11 * v.x + self._12 * v.y,
            self._21 * v.x + self._22 * v.y)
end

function Matrix2x2:solve(b)
    local det = self._11 * self._22 - self._12 * self._21
    if det ~= 0 then
        det = 1 / det
    end

    return Vec2.New(det * (self._22 * b.x - self._12 * b.y),
            det * (self._11 * b.y - self._21 * b.x))
end

return {
    New = GetOrCreate,

    mul = mul
}