package.path = package.path .. ';lua/libs/?.lua;lua/libs/?/init.lua;lua/libs/?/__init.lua;lua/libs/?/?.lua'
-- inspect = require('inspect')

function True()
  -- Thunk to let us pick the right value based on gradient or boolean
  return true
end

function False()
  -- Same idea as the above
  return false
end

function Eq(a, b)
  return a == b
end

function Le(a, b)
  return a <= b
end

function Lt(a, b)
  return a < b
end

function Gt(a, b)
  return a > b
end

function Ge(a, b)
  return a >= b
end

function And(a, b)
  return a and b
end

function Or(a, b)
  return a or b
end

function Not(a)
  return not a
end
