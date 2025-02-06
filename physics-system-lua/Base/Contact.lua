---@class Contact
local Contact = CreateClass("Contact")

function Contact:__ctor(p, n, d, hash)
    self.hash = hash

    -- Contact point
    self.p = p

    -- Contact norma (toward shape2)
    self.n = n

    -- Penetration depth (d < 0)
    self.d = d

    -- Accumulated norma constraint impuse
    self.lambda_n_acc = 0

    -- Accumulated tangentia constraint impuse
    self.lambda_t_acc = 0
end

return Contact