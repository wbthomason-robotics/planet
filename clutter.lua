AT_THRESHOLD = 0.2
local And = And
local Le = Le
local Ge = Ge
local sqrt = math.sqrt
local abs = math.abs

function on(x, y) return And(Le(x.pz - y.pz, 0.15), Ge(x.pz - y.pz, 0.0)) end

function above(m, s)
  -- Test if we're close enough to the SSSP of the surface
  local x_low = s.sssp.region.x.low + s.px
  local x_high = s.sssp.region.x.high + s.px
  local y_low = s.sssp.region.y.low + s.py
  local y_high = s.sssp.region.y.high + s.py
  local z_low = s.sssp.region.z.low + s.pz
  local z_high = s.sssp.region.z.high + s.pz + 0.3
  local in_x = And(Ge(m.px, x_low), Le(m.px, x_high))
  local in_y = And(Ge(m.py, y_low), Le(m.py, y_high))
  local in_z = And(Ge(m.pz, z_low), Le(m.pz, z_high))
  return And(And(in_x, in_y), in_z)
end

function at(m, x)
  local distance = sqrt((m.px - x.px) ^ 2.0 + (m.py - x.py) ^ 2.0 + (m.pz - x.pz) ^ 2.0)
  return Le(distance, AT_THRESHOLD)
end

function manhattandist(x, y)
  local x_dist = abs(x.px - y.px)
  local y_dist = abs(x.py - y.py)
  local z_dist = abs(x.pz - y.pz)
  return x_dist + y_dist + z_dist
end

function manhattanat(m, x) return Le(manhattandist(m, x), AT_THRESHOLD) end

function donthit(l, o) return Ge(manhattandist(l, o), 0.0) end

function donthits(l, s) return Ge(manhattandist(l, s), 0.0) end
