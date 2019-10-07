-- Automatically compute the analytical derivative for the unsatisfaction semantics
-- TODO/NOTE: Some of these derivatives - namely Eq, And, and Or - are discontinuous, so we use
-- smooth approximations

package.path = package.path .. ';lua/libs/?.lua;lua/libs/?/init.lua;lua/libs/?/__init.lua;lua/libs/?/?.lua'

math = require('sci.math').generic
diff = require('sci.diff')
alg  = require('sci.alg')

-- The dual number type used by diff, to make it easier to construct new objects from C++
dn = diff.dn

-- A tiny number used as epsilon
eps = 0.0000000000000001

function True()
  -- Thunk to let us pick the right value based on gradient or boolean
  return 0
end

function False()
  -- Same idea as the above
  return math.huge
end

function Eq(a, b)
  return math.sqrt(eps + math.pow(a - b, 2))
end

function Le(a, b)
  return math.max(a - b, 0)
end

function Lt(a, b)
  return math.max(a - b, 0)
end

function Gt(a, b)
  return math.max(b - a, 0)
end

function Ge(a, b)
  return math.max(b - a, 0)
end

-- NOTE: We don't actually need/use these smooth approximations b/c diff implements better ones
-- under the hood
local function smax(x, y)
  -- Smooth version of max(x, y)
  return 0.5 * (x + y + math.sqrt(math.pow(x - y, 2) + eps))
end

local function smaxz(x)
  -- Smooth version of max(x, 0)
  -- The sqrt stuff is smooth abs()
  return 0.5 * (x + math.sqrt(math.pow(x, 2) + eps))
end

function And(a, b)
  return math.sqrt(math.max(a, 0)^2.0 + math.max(b, 0)^2.0)
end

function Or(a, b)
  return -1.0 * math.max(-a, -b)
end

-- NOTE: This definition is wrong!!!!
function Not(a)
  return -a
end
