package.path = package.path .. ';lua/libs/?.lua;lua/libs/?/init.lua;lua/libs/?/__init.lua;lua/libs/?/?.lua'
math = require('sci.math')
-- inspect = require('inspect')

function Eq(a, b)
  return math.abs(a - b)
end

function Le(a, b)
  return a - b
end

function Lt(a, b)
  return a - b
end

function Gt(a, b)
  return b - a
end

function Ge(a, b)
  return b - a
end

function And(a, b)
  return math.sqrt(math.pow(math.max(0, a), 2) + math.pow(math.max(0, b), 2))
end

function Or(a, b)
  return math.min(a, b)
end

function Not(a)
  return -a
end
