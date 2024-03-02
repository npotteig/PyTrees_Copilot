-- This example implements a simple home heating system. The system heats
-- when the temperature gets too low, and stops when it is high enough. 

module Heater where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((>), (<), div)

-- External temperature as a float, ranging from -50C to 100C.

-- Temperature in Celsius.
ctemp :: Stream Float
ctemp = extern "temperature" Nothing

spec = do
  -- Triggers that fire when the ctemp is too low or too high,
  -- pass the current ctemp as an argument.
  trigger "heaton"  (ctemp < 18.0) [arg ctemp]
  trigger "heatoff" (ctemp > 21.0) [arg ctemp]

-- Compile the spec
main = reify spec >>= compile "monitor"
