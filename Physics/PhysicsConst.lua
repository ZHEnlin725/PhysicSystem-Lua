PI = 3.1415926

MathConst = {
    ZERO = 0.0002,
    DEG_TO_RAD = PI / 180,
    RAD_TO_DEG = 180 / PI,
    HALF_PI = PI / 2,
    DOUBLE_PI = PI * 2,
    Infinity = math.huge
}

Utils = {
    checkAABBCollide = function(aabb1, aabb2)
        return aabb1[1] < aabb2[3] and aabb1[3] > aabb2[1] and aabb1[2] < aabb2[4] and aabb1[4] > aabb2[2]
    end
}

BodyType = {
    Kinematic = 0,

    Static = 1,

    Dynamic = 2,
}

ShapeType = {

    Circle = 1,

    Poly = 2,

    Segment = 4,

    Comp = 8

}

CollideType = {
    Poly_Poly = 1,

    Circle_Circle = 2,

    Poly_Circle = 3,
}