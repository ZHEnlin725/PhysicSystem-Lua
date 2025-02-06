local function _circle2Circle(c1, r1, c2, r2, contactArr)
    local rmax = r1 + r2
    local t = Vec2.sub(c2, c1)
    local distsq = t:lengthSq()

    if (distsq > rmax * rmax) then
        t:Delete()
        return false
    end

    local dist = math.sqrt(distsq)

    local p = Vec2.mad(c1, t, 0.5 + (r1 - r2) * 0.5 / dist)
    local scaled = (dist ~= 0)
    local n = scaled and Vec2.scale(t, 1 / dist) or Vec2.zero
    local d = dist - rmax

    t:Delete()
    p:Delete()
    if scaled then
        n:Delete()
    end

    table.insert(contactArr, Contact.New(p, n, d, 0))
    return true
end

---@param poly Polygon
local function findMSA(poly, planes, num)
    local min_dist = -math.huge
    local min_index = -1
    for i = 1, num do
        local dist = poly:distOnPlane(planes[i].n, planes[i].d)
        if (dist > 0) then
            -- no collision
            return 0, -1
        elseif (dist > min_dist) then
            min_dist = dist
            min_index = i
        end
    end

    return min_dist, min_index
end

---@param poly1 Polygon
---@param poly2 Polygon
local function findVertsFallback(contactArr, poly1, poly2, n, dist)
    local num = 0
    for i, v in ipairs(poly1.tvertices) do
        if poly2:containPointPartial(v, n) then
            table.insert(contactArr, Contact.New(v, n, dist, (poly1.id << 16) | i))
            num = num + 1
        end
    end
    for i, v in ipairs(poly2.tvertices) do
        if poly1:containPointPartial(v, n) then
            table.insert(contactArr, Contact.New(v, n, dist, (poly2.id << 16) | i))
            num = num + 1
        end
    end

    return num > 0
end

--Find the overlapped vertices.
---@param poly1 Polygon
---@param poly2 Polygon
local function findVerts(contactArr, poly1, poly2, n, dist)
    local num = 0
    for i, v in ipairs(poly1.tvertices) do
        if poly2:containPoint(v) then
            table.insert(contactArr, Contact.New(v, n, dist, (poly1.id << 16) | i))
            num = num + 1
        end
    end

    for i, v in ipairs(poly2.tvertices) do
        if poly1:containPoint(v) then
            table.insert(contactArr, Contact.New(v, n, dist, (poly2.id << 16) | i))
            num = num + 1
        end
    end

    return num > 0 or findVertsFallback(contactArr, poly1, poly2, n, dist)
end

local function findPointsBehindSeg(contactArr, seg, poly, dist, coef)
    local dta = Vec2.cross(seg.tn, seg.ta)
    local dtb = Vec2.cross(seg.tn, seg.tb)
    local n = Vec2.scale(seg.tn, coef)

    for i, v in ipairs(poly.tvertices) do
        if Vec2.dot(v, n) < Vec2.dot(seg.tn, seg.ta) * coef + seg.r then
            local dt = Vec2.cross(seg.tn, v)
            if dta >= dt and dt >= dtb then
                table.insert(contactArr, Contact.New(v, n, dist, (poly.id << 16) | i))
            end
        end
    end

    n:Delete()
end

---@param seg Segment
local function segmentPointDistanceSq(seg, p)
    local w = Vec2.sub(p, seg.ta)
    local d = Vec2.sub(seg.tb, seg.ta)
    local proj = w:dot(d)
    local result
    if proj <= 0 then
        result = w:dot(w)
    else
        local vsq = d:dot(d)
        if proj >= vsq then
            result = w:dot(w) - 2 * proj + vsq
        else
            result = w:dot(w) - proj * proj / vsq
        end
    end

    w:Delete()
    d:Delete()
    return result
end

local function segment2Segment(seg1, seg2, contactArr)
    local d = {
        segmentPointDistanceSq(seg1, seg2.ta),
        segmentPointDistanceSq(seg1, seg2.tb),
        segmentPointDistanceSq(seg2, seg1.ta),
        segmentPointDistanceSq(seg2, seg1.tb),
    }
    local idx1 = d[1] < d[2] and 1 or 2
    local idx2 = d[3] < d[4] and 3 or 4
    local idxm = d[idx1] < d[idx2] and idx1 or idx2
    local s, t

    local u = Vec2.sub(seg1.tb, seg1.ta)
    local v = Vec2.sub(seg2.tb, seg2.ta)
    if idxm == 1 then
        local sub = Vec2.sub(seg2.ta, seg1.ta)
        s = Vec2.dot(sub, u) / Vec2.dot(u, u)
        s = s < 0 and 0 or (s > 1 and 1 or s)
        t = 0
        sub:Delete()
    elseif idxm == 2 then
        local sub = Vec2.sub(seg2.tb, seg1.ta)
        s = Vec2.dot(sub, u) / Vec2.dot(u, u)
        s = s < 0 and 0 or (s > 1 and 1 or s)
        t = 1
        sub:Delete()
    elseif idxm == 3 then
        s = 0
        local sub = Vec2.sub(seg1.ta, seg2.ta)
        t = Vec2.dot(sub, v) / Vec2.dot(v, v)
        t = t < 0 and 0 or (t > 1 and 1 or t)
        sub:Delete()
    elseif idxm == 4 then
        s = 1
        local sub = Vec2.sub(seg1.tb, seg2.ta)
        t = Vec2.dot(sub, v) / Vec2.dot(v, v)
        t = t < 0 and 0 or (t > 1 and 1 or t)
        sub:Delete()
    end
    local minp1 = Vec2.mad(seg1.ta, u, s)
    local minp2 = Vec2.mad(seg2.ta, v, t)
    local result = _circle2Circle(minp1, seg1.r, minp2, seg2.r, contactArr)

    minp1:Delete()
    minp2:Delete()

    return result
end

---@param seg Segment
---@param poly Polygon
local function segment2Poly(seg, poly, contactArr)
    local seg_td = Vec2.dot(seg.tn, seg.ta)
    local seg_d1 = poly:distOnPlane(seg.tn, seg_td) - seg.r
    if (seg_d1 > 0) then
        return false
    end
    local seg_d2 = poly:distOnPlane(Vec2.neg(seg.tn), -seg_td) - seg.r
    if (seg_d2 > 0) then
        return false
    end

    local poly_d = -math.huge
    local poly_i = -1
    for i, plane in ipairs(poly.tplanes) do
        local dist = seg:distOnPlane(plane.n, plane.d)
        if dist > 0 then
            return false
        end
        if dist > poly_d then
            poly_i = i
            poly_d = dist
        end
    end

    local poly_n = Vec2.neg(poly.tplanes[poly_i].n)
    local va = Vec2.mad(seg.ta, poly_n, seg.r)
    local vb = Vec2.mad(seg.tb, poly_n, seg.r)

    if poly:containPoint(va) then
        table.insert(contactArr, Contact.New(va, poly_n, poly_d, (seg.id << 16) | 0))
    end
    if poly:containPoint(vb) then
        table.insert(contactArr, Contact.New(vb, poly_n, poly_d, (seg.id << 16) | 0))
    end

    -- Floating point precision problems here.
    -- This will have to do for now.
    poly_d = poly_d - 0.1
    if (seg_d1 >= poly_d or seg_d2 >= poly_d) then
        if (seg_d1 > seg_d2) then
            findPointsBehindSeg(contactArr, seg, poly, seg_d1, 1)
        else
            findPointsBehindSeg(contactArr, seg, poly, seg_d2, -1)
        end
    end

    -- If no other collision points are found, try colliding endpoints.
    if #contactArr == 0 then
        local poly_a = poly.tvertices[poly_i]
        local nextI = poly_i + 1
        if nextI > #poly.vertices then
            nextI = 1
        end
        local poly_b = poly.tvertices[nextI]
        if _circle2Circle(seg.ta, seg.r, poly_a, 0, contactArr) then
            return true
        end

        if _circle2Circle(seg.tb, seg.r, poly_a, 0, contactArr) then
            return true
        end

        if _circle2Circle(seg.ta, seg.r, poly_b, 0, contactArr) then
            return true
        end

        if _circle2Circle(seg.tb, seg.r, poly_b, 0, contactArr) then
            return true
        end
    end
    return #contactArr > 0
end

local function circle2Circle(circ1, circ2, contactArr)
    return _circle2Circle(circ1.tc, circ1.r, circ2.tc, circ2.r, contactArr)
end

local function circle2Segment(circ, seg, contactArr)
    local rsum = circ.r + seg.r

    -- Normal distance from segment
    local dn = Vec2.dot(circ.tc, seg.tn) - Vec2.dot(seg.ta, seg.tn)
    local dist = (dn < 0 and dn * -1 or dn) - rsum
    if (dist > 0) then
        return false
    end

    -- Tangential distance along segment
    local dt = Vec2.cross(circ.tc, seg.tn)
    local dtMin = Vec2.cross(seg.ta, seg.tn)
    local dtMax = Vec2.cross(seg.tb, seg.tn)

    if (dt < dtMin) then
        if (dt < dtMin - rsum) then
            return false
        end
        return _circle2Circle(circ.tc, circ.r, seg.ta, seg.r, contactArr)
    elseif (dt > dtMax) then
        if (dt > dtMax + rsum) then
            return false
        end
        return _circle2Circle(circ.tc, circ.r, seg.tb, seg.r, contactArr)
    end

    local n = (dn > 0) and seg.tn or Vec2.neg(seg.tn)

    table.insert(contactArr, Contact.New(Vec2.mad(circ.tc, n, -(circ.r + dist * 0.5)), Vec2.neg(n), dist, 0))

    return true
end

local function circle2Poly(circ, poly, contactArr)
    local minDist = -math.huge
    local minIdx = -1
    for i, plane in ipairs(poly.tplanes) do
        local dist = Vec2.dot(circ.tc, plane.n) - plane.d - circ.r
        if dist > 0 then
            return false
        elseif dist > minDist then
            minDist = dist
            minIdx = i
        end
    end

    local n = poly.tplanes[minIdx].n
    local a = poly.tvertices[minIdx]
    local nextI = minIdx + 1
    if nextI > #poly.vertices then
        nextI = 1
    end
    local b = poly.tvertices[nextI]
    local dta = Vec2.cross(a, n)
    local dtb = Vec2.cross(b, n)
    local dt = Vec2.cross(circ.tc, n)

    if (dt > dta) then
        return _circle2Circle(circ.tc, circ.r, a, 0, contactArr)
    elseif (dt < dtb) then
        return _circle2Circle(circ.tc, circ.r, b, 0, contactArr)
    end

    table.insert(contactArr, Contact.New(Vec2.mad(circ.tc, n, -(circ.r + minDist * 0.5)), Vec2.neg(n), minDist, 0))

    return true
end

local function poly2Poly(poly1, poly2, contactArr)
    local dist1, index1 = findMSA(poly2, poly1.tplanes, #poly1.vertices)
    if index1 == -1 then
        return false
    end
    local dist2, index2 = findMSA(poly1, poly2.tplanes, #poly2.vertices)
    if index2 == -1 then
        return false
    end
    if dist1 > dist2 then
        return findVerts(contactArr, poly1, poly2, poly1.tplanes[index1].n, dist1)
    end
    return findVerts(contactArr, poly1, poly2, Vec2.neg(poly2.tplanes[index2].n), dist2)
end

local COLLIDE_FUNCS = {
    [SHAPE_TYPE.CIRCLE] = circle2Circle,
    [SHAPE_TYPE.CIRCLE | SHAPE_TYPE.POLY] = circle2Poly,
    [SHAPE_TYPE.CIRCLE | SHAPE_TYPE.SEGMENT] = circle2Segment,

    [SHAPE_TYPE.SEGMENT] = segment2Segment,
    [SHAPE_TYPE.SEGMENT | SHAPE_TYPE.POLY] = segment2Poly,

    [SHAPE_TYPE.POLY] = poly2Poly,
}

local function collide(a, b, contactArr)
    contactArr = contactArr or {}
    --if a.type > b.type then
    --    local temp = a
    --    a = b
    --    b = temp
    --end
    local result = COLLIDE_FUNCS[a.type | b.type](a, b, contactArr)
    return result, contactArr
end

return {
    circle2Circle = circle2Circle,
    circle2Poly = circle2Poly,
    poly2Poly = poly2Poly,

    collide = collide,
}